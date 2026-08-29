#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "mouse_odometry/esp_ros_time_synchronizer.hpp"
#include "mouse_odometry/pmw3901_serial_protocol.hpp"

namespace
{

using PacketBytes = std::array<uint8_t, mouse_odometry::kPmw3901PacketSize>;

void putU16(PacketBytes & bytes, const std::size_t offset, const uint16_t value)
{
  bytes[offset] = static_cast<uint8_t>(value);
  bytes[offset + 1] = static_cast<uint8_t>(value >> 8U);
}

void putU32(PacketBytes & bytes, const std::size_t offset, const uint32_t value)
{
  for (uint8_t index = 0; index < 4; ++index) {
    bytes[offset + index] = static_cast<uint8_t>(value >> (index * 8U));
  }
}

void putU64(PacketBytes & bytes, const std::size_t offset, const uint64_t value)
{
  for (uint8_t index = 0; index < 8; ++index) {
    bytes[offset + index] = static_cast<uint8_t>(value >> (index * 8U));
  }
}

PacketBytes makePacket(const uint32_t cycle_id = 42, const uint64_t timestamp_us = 123456789)
{
  PacketBytes bytes{};
  putU16(bytes, 0, mouse_odometry::kPmw3901PacketHeader);
  bytes[2] = mouse_odometry::kPmw3901PacketVersion;
  putU32(bytes, 3, cycle_id);
  putU64(bytes, 7, timestamp_us);
  putU32(bytes, 15, 10000);
  putU16(bytes, 19, static_cast<uint16_t>(-123));
  putU16(bytes, 21, 456);
  bytes[23] = 44;
  bytes[24] = 0x23;
  putU16(bytes, 25, 789);
  putU16(bytes, 27, static_cast<uint16_t>(-321));
  bytes[29] = 55;
  bytes[30] = 0x25;
  putU16(
    bytes, 31,
    mouse_odometry::crc16CcittFalse(
      bytes.data(), mouse_odometry::kPmw3901PacketPayloadSize));
  return bytes;
}

TEST(Pmw3901SerialProtocol, TestAValidPacket)
{
  const PacketBytes bytes = makePacket();
  const auto packet = mouse_odometry::decodePacketV1(bytes.data(), bytes.size());
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->cycle_id, 42U);
  EXPECT_EQ(packet->timestamp_us, 123456789U);
  EXPECT_EQ(packet->measurement_interval_us, 10000U);
  EXPECT_EQ(packet->left.dx, -123);
  EXPECT_EQ(packet->left.dy, 456);
  EXPECT_EQ(packet->left.squal, 44U);
  EXPECT_EQ(packet->left.status, 0x23U);
  EXPECT_EQ(packet->right.squal, 55U);
  EXPECT_EQ(packet->right.status, 0x25U);
}

TEST(Pmw3901SerialProtocol, TestBLittleEndianSignedInt16)
{
  const PacketBytes bytes = makePacket();
  const auto packet = mouse_odometry::decodePacketV1(bytes.data(), bytes.size());
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->left.dx, -123);
  EXPECT_EQ(packet->right.dx, 789);
  EXPECT_EQ(packet->right.dy, -321);
}

TEST(Pmw3901SerialProtocol, TestCCrcFailure)
{
  PacketBytes bytes = makePacket();
  bytes[20] ^= 0x01U;
  mouse_odometry::Pmw3901PacketStreamParser parser;
  EXPECT_TRUE(parser.push(bytes.data(), bytes.size()).empty());
  EXPECT_EQ(parser.statistics().crc_error_count, 1U);
  EXPECT_EQ(parser.statistics().packet_valid_total, 0U);
}

TEST(Pmw3901SerialProtocol, TestDStreamSplit)
{
  const PacketBytes bytes = makePacket();
  mouse_odometry::Pmw3901PacketStreamParser parser;
  EXPECT_TRUE(parser.push(bytes.data(), 10).empty());
  const auto packets = parser.push(bytes.data() + 10, bytes.size() - 10);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].cycle_id, 42U);
}

TEST(Pmw3901SerialProtocol, TestEMultiplePackets)
{
  const PacketBytes first = makePacket(10, 1000000);
  const PacketBytes second = makePacket(11, 1010000);
  std::vector<uint8_t> stream(first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());
  mouse_odometry::Pmw3901PacketStreamParser parser;
  const auto packets = parser.push(stream.data(), stream.size());
  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(packets[0].cycle_id, 10U);
  EXPECT_EQ(packets[1].cycle_id, 11U);
}

TEST(Pmw3901SerialProtocol, TestFGarbageBeforeHeader)
{
  const PacketBytes bytes = makePacket();
  std::vector<uint8_t> stream{0x12, 0x34, 0x56, 0x78};
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  mouse_odometry::Pmw3901PacketStreamParser parser;
  const auto packets = parser.push(stream.data(), stream.size());
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(parser.statistics().header_resync_count, 4U);
}

TEST(Pmw3901SerialProtocol, TestGCorruptedThenValidPacketRecovers)
{
  PacketBytes corrupted = makePacket(20, 2000000);
  corrupted[28] ^= 0x80U;
  const PacketBytes valid = makePacket(21, 2010000);
  std::vector<uint8_t> stream(corrupted.begin(), corrupted.end());
  stream.insert(stream.end(), valid.begin(), valid.end());
  mouse_odometry::Pmw3901PacketStreamParser parser;
  const auto packets = parser.push(stream.data(), stream.size());
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].cycle_id, 21U);
  EXPECT_EQ(parser.statistics().crc_error_count, 1U);
}

TEST(Pmw3901SerialProtocol, TestHTimestampConversion)
{
  mouse_odometry::EspRosTimeSynchronizer synchronizer(1);
  constexpr uint64_t esp_timestamp_us = 10000000U;
  constexpr int64_t ros_receive_time_ns = 1010000000000LL;
  ASSERT_TRUE(synchronizer.observe(esp_timestamp_us, ros_receive_time_ns));
  const auto converted = synchronizer.toRosTimeNanoseconds(esp_timestamp_us);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(*converted, 1010000000000LL);
  EXPECT_NEAR(synchronizer.offsetSeconds(), 1000.0, 1.0e-12);
}

TEST(Pmw3901SerialProtocol, ClockSyncUsesMinimumInitialOffset)
{
  mouse_odometry::EspRosTimeSynchronizer synchronizer(3);
  EXPECT_FALSE(synchronizer.observe(1000000U, 1105000000LL));
  EXPECT_FALSE(synchronizer.observe(1010000U, 1112000000LL));
  EXPECT_TRUE(synchronizer.observe(1020000U, 1124000000LL));
  EXPECT_EQ(synchronizer.collectedSampleCount(), 3U);
  EXPECT_EQ(synchronizer.offsetNanoseconds(), 102000000LL);
}

TEST(Pmw3901SerialProtocol, TestKEspTimestampRollbackRestartsClockSync)
{
  mouse_odometry::EspRosTimeSynchronizer synchronizer(2);
  EXPECT_FALSE(synchronizer.observe(299990000U, 1299990000000LL));
  ASSERT_TRUE(synchronizer.observe(300000000U, 1300000000000LL));
  EXPECT_EQ(*synchronizer.toRosTimeNanoseconds(300000000U), 1300000000000LL);

  EXPECT_FALSE(synchronizer.observe(200000U, 1301000000000LL));
  EXPECT_FALSE(synchronizer.valid());
  EXPECT_EQ(synchronizer.collectedSampleCount(), 1U);
  EXPECT_FALSE(synchronizer.toRosTimeNanoseconds(200000U).has_value());

  ASSERT_TRUE(synchronizer.observe(210000U, 1301010000000LL));
  EXPECT_EQ(*synchronizer.toRosTimeNanoseconds(210000U), 1301010000000LL);
}

TEST(Pmw3901SerialProtocol, TestIEspRebootResetsSession)
{
  mouse_odometry::EspPacketSequenceTracker tracker;
  EXPECT_FALSE(tracker.observe(1000, 100000000U).reboot_detected);
  const auto update = tracker.observe(0, 100000U);
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.reboot_detected);
  EXPECT_EQ(update.dropped_packets, 0U);
}

TEST(Pmw3901SerialProtocol, TestHCycleDropCountsMissingPacket)
{
  mouse_odometry::EspPacketSequenceTracker tracker;
  EXPECT_EQ(tracker.observe(100, 1000000U).dropped_packets, 0U);
  EXPECT_EQ(tracker.observe(101, 1010000U).dropped_packets, 0U);
  const auto update = tracker.observe(103, 1030000U);
  EXPECT_TRUE(update.accepted);
  EXPECT_EQ(update.dropped_packets, 1U);
}

TEST(Pmw3901SerialProtocol, TestJCycleWraparoundIsContinuous)
{
  mouse_odometry::EspPacketSequenceTracker tracker;
  const std::array<uint32_t, 4> cycles{{
    0xFFFFFFFEU, 0xFFFFFFFFU, 0x00000000U, 0x00000001U}};
  uint64_t timestamp_us = 1000000;
  for (const uint32_t cycle : cycles) {
    const auto update = tracker.observe(cycle, timestamp_us);
    EXPECT_TRUE(update.accepted);
    EXPECT_FALSE(update.reboot_detected);
    EXPECT_EQ(update.dropped_packets, 0U);
    timestamp_us += 10000;
  }
}

TEST(Pmw3901SerialProtocol, CrcKnownVector)
{
  const std::array<uint8_t, 9> input{{'1', '2', '3', '4', '5', '6', '7', '8', '9'}};
  EXPECT_EQ(mouse_odometry::crc16CcittFalse(input.data(), input.size()), 0x29B1U);
}

TEST(Pmw3901SerialProtocol, StatusErrorsOverrideValidBit)
{
  using mouse_odometry::isPmw3901SensorStatusValid;
  EXPECT_TRUE(isPmw3901SensorStatusValid(0x01U));
  EXPECT_TRUE(isPmw3901SensorStatusValid(0x01U | 0x02U | 0x20U));
  EXPECT_FALSE(isPmw3901SensorStatusValid(0x00U));
  EXPECT_FALSE(isPmw3901SensorStatusValid(0x01U | 0x08U));
  EXPECT_FALSE(isPmw3901SensorStatusValid(0x01U | 0x10U));
  EXPECT_FALSE(isPmw3901SensorStatusValid(0x01U | 0x40U));
}

}  // namespace

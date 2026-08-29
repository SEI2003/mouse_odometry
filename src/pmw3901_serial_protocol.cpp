#include "mouse_odometry/pmw3901_serial_protocol.hpp"

#include <algorithm>
#include <array>

namespace mouse_odometry
{
namespace
{

constexpr std::array<uint8_t, 2> kWireHeader{{0x55, 0xAA}};

uint16_t readU16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

int16_t readI16(const uint8_t * data)
{
  return static_cast<int16_t>(readU16(data));
}

uint32_t readU32(const uint8_t * data)
{
  uint32_t value = 0;
  for (uint8_t index = 0; index < 4; ++index) {
    value |= static_cast<uint32_t>(data[index]) << (index * 8U);
  }
  return value;
}

uint64_t readU64(const uint8_t * data)
{
  uint64_t value = 0;
  for (uint8_t index = 0; index < 8; ++index) {
    value |= static_cast<uint64_t>(data[index]) << (index * 8U);
  }
  return value;
}

}  // namespace

uint16_t crc16CcittFalse(const uint8_t * data, const std::size_t size)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

bool isPmw3901SensorStatusValid(const uint8_t status)
{
  constexpr uint8_t error_mask =
    kSensorStatusSpiError | kSensorStatusInitError | kSensorStatusObservationError;
  return (status & kSensorStatusValid) != 0U && (status & error_mask) == 0U;
}

std::optional<Pmw3901PacketV1> decodePacketV1(
  const uint8_t * data, const std::size_t size)
{
  if (data == nullptr || size != kPmw3901PacketSize ||
    readU16(data) != kPmw3901PacketHeader ||
    data[2] != kPmw3901PacketVersion)
  {
    return std::nullopt;
  }

  const uint16_t expected_crc = readU16(data + kPmw3901PacketPayloadSize);
  if (crc16CcittFalse(data, kPmw3901PacketPayloadSize) != expected_crc) {
    return std::nullopt;
  }

  Pmw3901PacketV1 packet;
  packet.cycle_id = readU32(data + 3);
  packet.timestamp_us = readU64(data + 7);
  packet.measurement_interval_us = readU32(data + 15);
  packet.left.dx = readI16(data + 19);
  packet.left.dy = readI16(data + 21);
  packet.left.squal = data[23];
  packet.left.status = data[24];
  packet.right.dx = readI16(data + 25);
  packet.right.dy = readI16(data + 27);
  packet.right.squal = data[29];
  packet.right.status = data[30];
  return packet;
}

std::vector<Pmw3901PacketV1> Pmw3901PacketStreamParser::push(
  const uint8_t * data, const std::size_t size)
{
  if (data != nullptr && size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Pmw3901PacketV1> decoded;
  while (!buffer_.empty()) {
    const auto header = std::search(
      buffer_.begin(), buffer_.end(), kWireHeader.begin(), kWireHeader.end());
    if (header == buffer_.end()) {
      const bool preserve_partial_header = buffer_.back() == kWireHeader[0];
      const std::size_t discarded = buffer_.size() - (preserve_partial_header ? 1U : 0U);
      statistics_.header_resync_count += discarded;
      if (preserve_partial_header) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }

    const std::size_t skipped = static_cast<std::size_t>(
      std::distance(buffer_.begin(), header));
    if (skipped != 0U) {
      statistics_.header_resync_count += skipped;
      buffer_.erase(buffer_.begin(), header);
    }

    if (buffer_.size() < kPmw3901PacketSize) {
      break;
    }

    ++statistics_.packet_received_total;
    if (buffer_[2] != kPmw3901PacketVersion) {
      ++statistics_.version_error_count;
      ++statistics_.header_resync_count;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const uint16_t expected_crc = readU16(buffer_.data() + kPmw3901PacketPayloadSize);
    if (crc16CcittFalse(buffer_.data(), kPmw3901PacketPayloadSize) != expected_crc) {
      ++statistics_.crc_error_count;
      ++statistics_.header_resync_count;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const auto packet = decodePacketV1(buffer_.data(), kPmw3901PacketSize);
    if (packet.has_value()) {
      decoded.push_back(*packet);
      ++statistics_.packet_valid_total;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + kPmw3901PacketSize);
  }
  return decoded;
}

void Pmw3901PacketStreamParser::clearBufferedData()
{
  buffer_.clear();
}

PacketSequenceUpdate EspPacketSequenceTracker::observe(
  const uint32_t cycle_id, const uint64_t timestamp_us)
{
  PacketSequenceUpdate update;
  if (!initialized_) {
    initialized_ = true;
    previous_cycle_id_ = cycle_id;
    previous_timestamp_us_ = timestamp_us;
    return update;
  }

  if (timestamp_us < previous_timestamp_us_) {
    update.reboot_detected = true;
    previous_cycle_id_ = cycle_id;
    previous_timestamp_us_ = timestamp_us;
    return update;
  }

  const uint32_t cycle_difference = cycle_id - previous_cycle_id_;
  if (cycle_difference == 0U) {
    update.accepted = false;
    update.duplicate = true;
    return update;
  }
  if (cycle_difference >= 0x80000000U) {
    update.accepted = false;
    update.out_of_order = true;
    return update;
  }

  update.dropped_packets = cycle_difference - 1U;
  previous_cycle_id_ = cycle_id;
  previous_timestamp_us_ = timestamp_us;
  return update;
}

void EspPacketSequenceTracker::reset()
{
  initialized_ = false;
  previous_cycle_id_ = 0;
  previous_timestamp_us_ = 0;
}

}  // namespace mouse_odometry

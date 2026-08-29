#ifndef MOUSE_ODOMETRY__PMW3901_SERIAL_PROTOCOL_HPP_
#define MOUSE_ODOMETRY__PMW3901_SERIAL_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mouse_odometry
{

constexpr uint16_t kPmw3901PacketHeader = 0xAA55;
constexpr uint8_t kPmw3901PacketVersion = 1;
constexpr std::size_t kPmw3901PacketPayloadSize = 31;
constexpr std::size_t kPmw3901PacketSize = 33;

enum Pmw3901SensorStatus : uint8_t
{
  kSensorStatusValid = 1U << 0,
  kSensorStatusMotion = 1U << 1,
  kSensorStatusNoMotion = 1U << 2,
  kSensorStatusSpiError = 1U << 3,
  kSensorStatusInitError = 1U << 4,
  kSensorStatusSqualAvailable = 1U << 5,
  kSensorStatusObservationError = 1U << 6,
  kSensorStatusReserved = 1U << 7,
};

struct Pmw3901SensorPacketData
{
  int16_t dx{0};
  int16_t dy{0};
  uint8_t squal{0};
  uint8_t status{0};
};

struct Pmw3901PacketV1
{
  uint32_t cycle_id{0};
  uint64_t timestamp_us{0};
  uint32_t measurement_interval_us{0};
  Pmw3901SensorPacketData left;
  Pmw3901SensorPacketData right;
};

struct PacketParserStatistics
{
  uint64_t packet_received_total{0};
  uint64_t packet_valid_total{0};
  uint64_t crc_error_count{0};
  uint64_t header_resync_count{0};
  uint64_t version_error_count{0};
};

uint16_t crc16CcittFalse(const uint8_t * data, std::size_t size);

bool isPmw3901SensorStatusValid(uint8_t status);

std::optional<Pmw3901PacketV1> decodePacketV1(
  const uint8_t * data, std::size_t size);

class Pmw3901PacketStreamParser
{
public:
  std::vector<Pmw3901PacketV1> push(const uint8_t * data, std::size_t size);
  void clearBufferedData();

  const PacketParserStatistics & statistics() const {return statistics_;}
  std::size_t bufferedByteCount() const {return buffer_.size();}

private:
  std::vector<uint8_t> buffer_;
  PacketParserStatistics statistics_;
};

struct PacketSequenceUpdate
{
  bool accepted{true};
  bool reboot_detected{false};
  bool duplicate{false};
  bool out_of_order{false};
  uint32_t dropped_packets{0};
};

class EspPacketSequenceTracker
{
public:
  PacketSequenceUpdate observe(uint32_t cycle_id, uint64_t timestamp_us);
  void reset();

private:
  bool initialized_{false};
  uint32_t previous_cycle_id_{0};
  uint64_t previous_timestamp_us_{0};
};

}  // namespace mouse_odometry

#endif  // MOUSE_ODOMETRY__PMW3901_SERIAL_PROTOCOL_HPP_

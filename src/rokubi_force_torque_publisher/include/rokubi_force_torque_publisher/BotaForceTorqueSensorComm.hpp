#ifndef BOTA_FORCE_TORQUE_SENSOR_COMM
#define BOTA_FORCE_TORQUE_SENSOR_COMM

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Rokubi: 0xAA, 34 payload bytes, little-endian CRC-16/X25.
// Decode explicitly instead of aliasing packed unions and unaligned floats.
class BotaForceTorqueSensorComm
{
public:
  struct Data {
    uint16_t status = 0;
    std::array<float, 6> forces{};
    uint32_t timestamp = 0;
    float temperature = 0;
  };
  struct Frame { Data data; } frame;
  enum ReadFrameRes {NO_FRAME, VALID_FRAME, NOT_VALID_FRAME, NOT_ALLIGNED_FRAME};
  explicit BotaForceTorqueSensorComm(int serial_port);
  static uint16_t crc16_x25(const uint8_t* data, size_t len);
  uint32_t get_crc_count() const {return crc_errors_;}
  bool isSynced() const {return synced_;}
  ReadFrameRes readFrame();

private:
  int serial_port_;
  bool synced_ = false;
  uint32_t crc_errors_ = 0;
  std::vector<uint8_t> buffer_;
};
#endif

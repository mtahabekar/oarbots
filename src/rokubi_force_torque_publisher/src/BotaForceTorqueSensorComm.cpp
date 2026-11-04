#include "rokubi_force_torque_publisher/BotaForceTorqueSensorComm.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <system_error>
#include <unistd.h>

namespace {
uint32_t little_endian(const uint8_t* p, size_t size) {
  uint32_t value = 0;
  for (size_t i = 0; i < size; ++i) value |= uint32_t(p[i]) << (8 * i);
  return value;
}
float decode_float(const uint8_t* p) {
  static_assert(sizeof(float) == 4, "Protocol requires 32-bit floats");
  uint32_t bits = little_endian(p, 4);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}

BotaForceTorqueSensorComm::BotaForceTorqueSensorComm(int serial_port)
: serial_port_(serial_port) {buffer_.reserve(1024);}

uint16_t BotaForceTorqueSensorComm::crc16_x25(const uint8_t* data, size_t len) {
  uint16_t crc = 0xffff;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i)
      crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return crc ^ 0xffff;
}

BotaForceTorqueSensorComm::ReadFrameRes BotaForceTorqueSensorComm::readFrame() {
  constexpr size_t frame_size = 37;
  // Owner opens O_NONBLOCK. Keep partial frames across calls.
  if (buffer_.size() < frame_size) {
    uint8_t bytes[512];
    const auto count = ::read(serial_port_, bytes, sizeof(bytes));
    if (count > 0) buffer_.insert(buffer_.end(), bytes, bytes + count);
    else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      throw std::system_error(errno, std::generic_category(), "Rokubi serial read");
  }
  auto header = std::find(buffer_.begin(), buffer_.end(), 0xaa);
  if (header != buffer_.begin()) {
    buffer_.erase(buffer_.begin(), header);
    synced_ = false;
    return NOT_ALLIGNED_FRAME;
  }
  if (buffer_.size() < frame_size) return NO_FRAME;
  if (crc16_x25(buffer_.data() + 1, 34) != little_endian(buffer_.data() + 35, 2)) {
    ++crc_errors_;
    synced_ = false;
    buffer_.erase(buffer_.begin());
    return NOT_VALID_FRAME;
  }
  frame.data.status = little_endian(buffer_.data() + 1, 2);
  for (size_t i = 0; i < 6; ++i) frame.data.forces[i] = decode_float(buffer_.data() + 3 + 4*i);
  frame.data.timestamp = little_endian(buffer_.data() + 27, 4);
  frame.data.temperature = decode_float(buffer_.data() + 31);
  buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  synced_ = true;
  return VALID_FRAME;
}

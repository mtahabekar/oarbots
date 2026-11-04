#include <gtest/gtest.h>
#include "rokubi_force_torque_publisher/BotaForceTorqueSensorComm.hpp"
#include <array>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

class SerialTest : public testing::Test {
protected:
  int fds[2];
  void SetUp() override {ASSERT_EQ(pipe2(fds, O_NONBLOCK), 0);}
  void TearDown() override {close(fds[0]); close(fds[1]);}
  std::array<uint8_t,37> packet() {
    std::array<uint8_t,37> p{}; p[0]=0xaa;
    const float value = 12.5f;
    std::memcpy(p.data()+3, &value, 4);
    p[27] = 0x78; p[28] = 0x56; p[29] = 0x34; p[30] = 0x12;
    auto crc = BotaForceTorqueSensorComm::crc16_x25(p.data()+1,34);
    p[35] = crc & 0xff; p[36] = crc >> 8;
    return p;
  }
};
TEST_F(SerialTest, PartialPacketIsRetained) {
  BotaForceTorqueSensorComm parser(fds[0]); auto p=packet();
  ASSERT_EQ(write(fds[1],p.data(),10),10);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::NO_FRAME);
  ASSERT_EQ(write(fds[1],p.data()+10,27),27);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::VALID_FRAME);
  EXPECT_FLOAT_EQ(parser.frame.data.forces[0],12.5);
  EXPECT_EQ(parser.frame.data.timestamp,0x12345678u);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::NO_FRAME);
}
TEST_F(SerialTest, CorruptAndMisalignedFramesRecover) {
  BotaForceTorqueSensorComm parser(fds[0]); auto valid=packet(); auto bad=valid; bad[7]^=1;
  uint8_t garbage[3]={1,2,3}; ASSERT_EQ(write(fds[1],garbage,3),3);
  ASSERT_EQ(write(fds[1],bad.data(),bad.size()),37);
  ASSERT_EQ(write(fds[1],valid.data(),valid.size()),37);
  bool recovered=false;
  for(int i=0;i<10;++i) if(parser.readFrame()==BotaForceTorqueSensorComm::VALID_FRAME) {recovered=true; break;}
  EXPECT_TRUE(recovered); EXPECT_EQ(parser.get_crc_count(),1u);
}
TEST_F(SerialTest, ConsecutiveFramesRemainAvailable) {
  BotaForceTorqueSensorComm parser(fds[0]); auto p=packet();
  ASSERT_EQ(write(fds[1],p.data(),37),37); ASSERT_EQ(write(fds[1],p.data(),37),37);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::VALID_FRAME);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::VALID_FRAME);
  EXPECT_EQ(parser.readFrame(),BotaForceTorqueSensorComm::NO_FRAME);
}
TEST(Protocol, StandardCrcCheckValue) {
  const char* value="123456789";
  EXPECT_EQ(BotaForceTorqueSensorComm::crc16_x25(reinterpret_cast<const uint8_t*>(value),9),0x906eu);
}

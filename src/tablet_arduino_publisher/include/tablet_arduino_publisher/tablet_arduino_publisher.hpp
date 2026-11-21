#ifndef TABLET_ARDUINO_PUBLISHER_HPP
#define TABLET_ARDUINO_PUBLISHER_HPP

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ArduinoReadNode : public rclcpp::Node
{
public:
    ArduinoReadNode();        
    ~ArduinoReadNode();

private:
    void readSerial();
    bool openSerial();
    void closeSerial();
    void handleSerialData(uint8_t raw_byte);

    std::string deadman_topic_;
    std::string estop_topic_;
    std::string com_port_;
    int serial_fd_;
    double timeout_;
    std::chrono::steady_clock::time_point last_valid_{};
    void publish_stop();

    // deadman_mode "toggle" (fabric sessions, 2026-11-21): a press latches the deadman on, the next press
    // turns it off. A press counts only when the switch reads pressed for `toggle_stable_samples` readings in a
    // row after reading released as long (contact bounce), toggles are at least `toggle_min_interval_sec`
    // apart, and after any stop (e-stop, serial fault, stale Arduino) the switch must be seen released before a
    // press can turn it on. "hold" (default): the published deadman is the switch itself.
    bool toggle_ = false;
    bool latched_ = false;
    bool stable_pressed_ = true;      // last stable switch reading; starts "pressed" so a release must be seen
    bool candidate_pressed_ = true;
    int candidate_count_ = 0;
    int stable_samples_ = 3;
    double min_interval_ = 0.3;
    std::chrono::steady_clock::time_point last_toggle_{};
    bool toggleDeadman(bool pressed, bool estop);
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
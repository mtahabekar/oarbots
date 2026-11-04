#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "kinova_msgs/msg/pose_velocity.hpp"
#include <chrono>

class KinovaTeleop : public rclcpp::Node
{
public:
    KinovaTeleop() : Node("kinova_teleop")
    {
        this->latest_message = kinova_msgs::msg::PoseVelocity();
        this->kinova_vel_publisher = this->create_publisher<kinova_msgs::msg::PoseVelocity>("commands/teleop/arm", 10);

        auto handle_input = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void {
            const double values[] = {msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z};
            for (double value : values) if (!std::isfinite(value)) return;
            received_ = std::chrono::steady_clock::now();
            this->latest_message.twist_linear_x = msg->linear.x;
            this->latest_message.twist_linear_y = msg->linear.y;
            this->latest_message.twist_linear_z = msg->linear.z;

            this->latest_message.twist_angular_x = msg->angular.x;
            this->latest_message.twist_angular_y = msg->angular.y;
            this->latest_message.twist_angular_z = msg->angular.z;
        };

        this->input_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/tablet_gui/spacenav/twist", 10, handle_input);

        // Kinova arm needs to be updated at 100 Hz, so we need to publish that frequently
        // https://github.com/RRL-ALeRT/kinova-ros2#:~:text=Note%20on%20publish,the%20requested%20velocity.
        this->timer = create_wall_timer(std::chrono::milliseconds(10), [this]() -> void {
            if (std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() > 0.15)
                latest_message = kinova_msgs::msg::PoseVelocity();
            kinova_vel_publisher->publish(latest_message);
        });
    }
private:
    std::chrono::steady_clock::time_point received_{};
    kinova_msgs::msg::PoseVelocity latest_message;
    rclcpp::Publisher<kinova_msgs::msg::PoseVelocity>::SharedPtr kinova_vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_subscription;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<KinovaTeleop>());

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
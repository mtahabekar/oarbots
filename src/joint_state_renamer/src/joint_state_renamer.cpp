#include "joint_state_renamer/joint_state_renamer.hpp"
#include <algorithm>
#include <cmath>
#include <set>

JointStateRenamer::JointStateRenamer() : Node("joint_state_renamer")
{
    // Delcare parameters
    rcl_interfaces::msg::ParameterDescriptor joint_state_prefix_description = rcl_interfaces::msg::ParameterDescriptor{};
    joint_state_prefix_description.description = "Prefix to append to all joint states";
    this->joint_state_prefix = declare_parameter<std::string>("joint_state_prefix", "", joint_state_prefix_description);

    rcl_interfaces::msg::ParameterDescriptor input_topic_description = rcl_interfaces::msg::ParameterDescriptor{};
    input_topic_description.description = "Topic to take in the joint states from";
    this->input_topic = declare_parameter<std::string>("input_topic", "", input_topic_description);

    rcl_interfaces::msg::ParameterDescriptor output_topic_description = rcl_interfaces::msg::ParameterDescriptor{};
    output_topic_description.description = "Topic to output the prefixed joint states to";
    this->output_topic = declare_parameter<std::string>("output_topic", "", output_topic_description);

    if (input_topic.empty() || output_topic.empty() || input_topic == output_topic)
        throw std::invalid_argument("Distinct, nonempty input and output topics required");
    if (!joint_state_prefix.empty() && (joint_state_prefix.front() == '/' || joint_state_prefix.back() != '/'))
        throw std::invalid_argument("Joint prefix must be empty or have only a trailing slash");
    this->input_subscription = this->create_subscription<sensor_msgs::msg::JointState>(this->input_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::SharedPtr msg) -> void { input_subscription_callback(msg); });
    this->output_publisher = this->create_publisher<sensor_msgs::msg::JointState>(this->output_topic, rclcpp::QoS(10));
}

void JointStateRenamer::input_subscription_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const auto size = msg->name.size();
    auto valid = [size](const auto& values) {
        return (values.empty() || values.size() == size) &&
            std::all_of(values.begin(), values.end(), [](double v){return std::isfinite(v);});
    };
    const std::set<std::string> names(msg->name.begin(), msg->name.end());
    if (!size || names.size() != size || names.count("") || !valid(msg->position) || !valid(msg->velocity) || !valid(msg->effort)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping malformed joint state");
        return;
    }
    rename_joint_states(msg);
    publish_joint_states(msg);
}

void JointStateRenamer::rename_joint_states(const sensor_msgs::msg::JointState::SharedPtr joint_states)
{
    for (std::string &joint_name : joint_states->name)
    {
        if (joint_name.rfind(joint_state_prefix, 0) != 0) joint_name.insert(0, joint_state_prefix);
    }
}

void JointStateRenamer::publish_joint_states(const sensor_msgs::msg::JointState::SharedPtr joint_states)
{
    output_publisher->publish(*joint_states);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JointStateRenamer>());

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
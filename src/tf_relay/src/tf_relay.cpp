#include "tf_relay/tf_relay.hpp"

TfRelay::TfRelay() : Node("tf_relay")
{
    // Delcare both parameters
    rcl_interfaces::msg::ParameterDescriptor robot_namespace_description = rcl_interfaces::msg::ParameterDescriptor{};
    robot_namespace_description.description = "Namespace used by the robot; should be unique";
    this->robot_namespace = declare_parameter<std::string>("robot_namespace", "", robot_namespace_description);

    rcl_interfaces::msg::ParameterDescriptor tf_frame_prefix_description = rcl_interfaces::msg::ParameterDescriptor{};
    tf_frame_prefix_description.description = "Prefix for all tf cordinate frames being published to /tf; should be unique";
    this->tf_frame_prefix = declare_parameter<std::string>("tf_frame_prefix", "", tf_frame_prefix_description);


    auto trim = [](std::string value) {
        auto begin = value.find_first_not_of('/');
        if (begin == std::string::npos) return std::string{};
        return value.substr(begin, value.find_last_not_of('/') - begin + 1);
    };
    robot_namespace = trim(robot_namespace);
    tf_frame_prefix = trim(tf_frame_prefix);
    if (robot_namespace.empty()) throw std::invalid_argument("robot_namespace must not be empty or root");
    if (!tf_frame_prefix.empty()) tf_frame_prefix += '/';

    // Create strings for the namespaced tf and tf_static topics
    std::string namespaced_tf_topic = "/" + this->robot_namespace + "/tf";
    std::string namespaced_tf_static_topic = "/" + this->robot_namespace + "/tf_static";

    // Initialize the publishers
    this->tf_publisher = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    this->tf_static_publisher = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).transient_local().reliable());

    // Initialize the subscribers
    this->namespaced_tf_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(namespaced_tf_topic, rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) -> void { namespaced_tf_callback(msg); });
    this->namespaced_tf_static_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(namespaced_tf_static_topic, rclcpp::QoS(1).transient_local().reliable(), [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) -> void { namespaced_tf_static_callback(msg); });

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully; Relaying %s -> %s and %s -> %s with prefix \"%s\"", namespaced_tf_topic.c_str(), "/tf", namespaced_tf_static_topic.c_str(), "/tf_static", this->tf_frame_prefix.c_str());
}



std::string TfRelay::add_frame_prefix(const std::string &frame) const
{
    const auto begin = frame.find_first_not_of('/');
    if (begin == std::string::npos) return "";
    const auto clean = frame.substr(begin);
    if (tf_frame_prefix.empty() || clean.rfind(tf_frame_prefix, 0) == 0) return clean;
    return tf_frame_prefix + clean;
}

void TfRelay::republish_incoming_message(const tf2_msgs::msg::TFMessage::SharedPtr msg, rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher)
{
    tf2_msgs::msg::TFMessage output = tf2_msgs::msg::TFMessage();

    output.transforms.reserve(msg->transforms.size());

    for (geometry_msgs::msg::TransformStamped transform : msg->transforms)
    {
        transform.header.frame_id = this->add_frame_prefix(transform.header.frame_id);
        transform.child_frame_id = this->add_frame_prefix(transform.child_frame_id);

        if (transform.header.frame_id.empty() || transform.child_frame_id.empty() ||
            transform.header.frame_id == transform.child_frame_id) continue;
        if (publisher == tf_static_publisher) static_transforms_[transform.child_frame_id] = transform;
        else output.transforms.push_back(std::move(transform));
    }

    if (publisher == tf_static_publisher) {
        for (const auto& entry : static_transforms_) output.transforms.push_back(entry.second);
    }
    if (!output.transforms.empty()) publisher->publish(output);
}

void TfRelay::namespaced_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    republish_incoming_message(msg, this->tf_publisher);
}

void TfRelay::namespaced_tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    republish_incoming_message(msg, this->tf_static_publisher);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<TfRelay>());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node returned an error: %s", e.what());

        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
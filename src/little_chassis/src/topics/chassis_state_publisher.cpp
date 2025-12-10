#include "little_chassis/topics/chassis_state_publisher.hpp"

#include <cstring>

namespace little_chassis
{
void ChassisStatePublisher::Init(const std::shared_ptr<LittleChassisNode> &node,
                                 const std::string &topicName,
                                 const rclcpp::QoS &qos)
{
    TopicPublisherBase<std_msgs::msg::ByteMultiArray>::Init(node, ROS_FEEDBACK_STATE, topicName, qos);
}

void ChassisStatePublisher::HandleMcuMessage(const uint8_t *data, std::size_t len)
{
    if (!data || len < sizeof(ChassisStateMessage_t))
        return;

    std_msgs::msg::ByteMultiArray msg;
    msg.data.resize(len);
    std::memcpy(msg.data.data(), data, len);

    auto pub = GetPublisher();
    if (pub)
    {
        pub->publish(msg);
    }
}
} // namespace little_chassis

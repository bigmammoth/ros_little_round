#include "little_chassis/topics/cmd_vel_subscriber.hpp"

namespace little_chassis
{
void CmdVelSubscriber::Init(const std::shared_ptr<LittleChassisNode> &node,
                            const std::string &topicName,
                            const rclcpp::QoS &qos)
{
    node_ = node;
    TopicSubscriberBase<geometry_msgs::msg::Twist>::Init(node, topicName, qos);
}

void CmdVelSubscriber::HandleMessage(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto node = node_.lock();
    if (!node) return;

    VelocityMessage_t velocityMsg{};
    velocityMsg.messageType = ROS_CMD_VELOCITY;
    velocityMsg.messageID = messageIdCounter_.fetch_add(1, std::memory_order_relaxed);
    velocityMsg.success = 0;
    velocityMsg.velocity = static_cast<float>(msg->linear.x);
    velocityMsg.omega = static_cast<float>(msg->angular.z);

    node->SendToMcu(reinterpret_cast<const uint8_t *>(&velocityMsg), sizeof(velocityMsg));
}
} // namespace little_chassis

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <atomic>
#include "little_chassis/topics/topic_subscriber_base.hpp"
#include "little_chassis/little_chassis_node.hpp"
#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    /**
     * @brief Subscribe to cmd_vel and forward velocity command to MCU.
     */
    class CmdVelSubscriber : public TopicSubscriberBase<geometry_msgs::msg::Twist>
    {
    public:
        void Init(const std::shared_ptr<LittleChassisNode> &node,
                  const std::string &topicName = "cmd_vel",
                  const rclcpp::QoS &qos = rclcpp::QoS(10));

    protected:
        void HandleMessage(const geometry_msgs::msg::Twist::SharedPtr msg) override;

    private:
        std::weak_ptr<LittleChassisNode> node_;
        std::atomic<uint32_t> messageIdCounter_{0};
    };
} // namespace little_chassis

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <string>

#include "little_chassis/topics/topic_publisher_base.hpp"
#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    /**
     * @brief Publishes nav_msgs/Odometry from MCU odometry feedback.
     */
    class OdomPublisher : public TopicPublisherBase<nav_msgs::msg::Odometry>
    {
    public:
        void Init(const std::shared_ptr<LittleChassisNode> &node,
                  const std::string &topicName = "odom",
                  const std::string &frameId = "odom",
                  const std::string &childFrameId = "base_link",
                  const rclcpp::QoS &qos = rclcpp::QoS(10));

    protected:
        void HandleMcuMessage(const uint8_t *data, std::size_t len) override;

    private:
        std::string frameId_;
        std::string childFrameId_;
    };
} // namespace little_chassis

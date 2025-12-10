#pragma once

#include <std_msgs/msg/byte_multi_array.hpp>
#include <string>

#include "little_chassis/topics/topic_publisher_base.hpp"
#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    /**
     * @brief Publishes raw MCU chassis state feedback as ByteMultiArray.
     * The payload layout matches ChassisStateMessage_t defined in ros_messages.h.
     */
    class ChassisStatePublisher : public TopicPublisherBase<std_msgs::msg::ByteMultiArray>
    {
    public:
        void Init(const std::shared_ptr<LittleChassisNode> &node,
                  const std::string &topicName = "chassis_state_raw",
                  const rclcpp::QoS &qos = rclcpp::QoS(10));

    protected:
        void HandleMcuMessage(const uint8_t *data, std::size_t len) override;
    };
} // namespace little_chassis

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <utility>

#include "little_chassis/little_chassis_node.hpp"
#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    /**
     * @brief Base template for publishing ROS topics from MCU feedback messages.
     *
     * Init creates the ROS publisher and registers an MCU message callback on the
     * owning LittleChassisNode. Derived classes implement HandleMcuMessage to
     * parse incoming data and publish the appropriate ROS message.
     */
    template <typename MsgT>
    class TopicPublisherBase
    {
    public:
        TopicPublisherBase() = default;
        virtual ~TopicPublisherBase() = default;

        /**
         * @brief Initialize publisher and register MCU message callback.
         *
         * @param node Shared pointer to the chassis node.
         * @param messageType MessageType_t to listen for from the MCU.
         * @param topicName ROS topic name to publish.
         * @param qos QoS profile (default depth 10).
         */
        void Init(const std::shared_ptr<LittleChassisNode> &node,
                  MessageType_t messageType,
                  const std::string &topicName,
                  const rclcpp::QoS &qos = rclcpp::QoS(10))
        {
            node_ = node;
            publisher_ = node->create_publisher<MsgT>(topicName, qos);

            node->RegisterMessageCallback(
                messageType,
                [this](const uint8_t *data, std::size_t len)
                {
                    this->HandleMcuMessage(data, len);
                });
        }

    protected:
        virtual void HandleMcuMessage(const uint8_t *data, std::size_t len) = 0;

        typename rclcpp::Publisher<MsgT>::SharedPtr GetPublisher() const { return publisher_; }
        std::shared_ptr<LittleChassisNode> GetNode() const { return node_.lock(); }

    private:
        std::weak_ptr<LittleChassisNode> node_;
        typename rclcpp::Publisher<MsgT>::SharedPtr publisher_;
    };
} // namespace little_chassis

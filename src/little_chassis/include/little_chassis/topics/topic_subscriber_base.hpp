#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <utility>

namespace little_chassis
{
    /**
     * @brief Base template for subscribing to ROS topics.
     *
     * Init creates the subscription; derived classes implement HandleMessage
     * to process incoming messages.
     */
    template <typename MsgT>
    class TopicSubscriberBase
    {
    public:
        TopicSubscriberBase() = default;
        virtual ~TopicSubscriberBase() = default;

        /**
         * @brief Initialize subscription.
         *
         * @param node Shared pointer to the node that owns the subscription.
         * @param topicName Topic name to subscribe to.
         * @param qos QoS profile (default depth 10).
         */
        void Init(const std::shared_ptr<rclcpp::Node> &node,
                  const std::string &topicName,
                  const rclcpp::QoS &qos = rclcpp::QoS(10))
        {
            node_ = node;
            subscription_ = node->template create_subscription<MsgT>(
                topicName,
                qos,
                [this](const typename MsgT::SharedPtr msg)
                {
                    this->HandleMessage(msg);
                });
        }

    protected:
        virtual void HandleMessage(const typename MsgT::SharedPtr msg) = 0;
        std::shared_ptr<rclcpp::Node> GetNode() const { return node_.lock(); }

    private:
        std::weak_ptr<rclcpp::Node> node_;
        typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
    };
} // namespace little_chassis

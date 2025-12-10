#include "little_chassis/topics/odom_publisher.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

namespace little_chassis
{
void OdomPublisher::Init(const std::shared_ptr<LittleChassisNode> &node,
                         const std::string &topicName,
                         const std::string &frameId,
                         const std::string &childFrameId,
                         const rclcpp::QoS &qos)
{
    frameId_ = frameId;
    childFrameId_ = childFrameId;
    TopicPublisherBase<nav_msgs::msg::Odometry>::Init(node, ROS_FEEDBACK_ODOMETRY, topicName, qos);
}

void OdomPublisher::HandleMcuMessage(const uint8_t *data, std::size_t len)
{
    if (!data || len < sizeof(OdometryMessage_t))
        return;

    const auto *msg = reinterpret_cast<const OdometryMessage_t *>(data);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Clock().now();
    odom.header.frame_id = frameId_;
    odom.child_frame_id = childFrameId_;
    odom.pose.pose.position.x = msg->posX;
    odom.pose.pose.position.y = msg->posY;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg->theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = msg->velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = msg->omega;

    // Basic covariance: small confidence on planar xy/yaw, large on unused axes
    odom.pose.covariance = {
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-2};

    odom.twist.covariance = {
        1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-2};

    auto pub = GetPublisher();
    if (pub) pub->publish(odom);
}
} // namespace little_chassis

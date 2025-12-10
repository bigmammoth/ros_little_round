#include "little_chassis/params_transfer/parameter_transfer.hpp"
#include "little_chassis/little_chassis_node.hpp"

using little_chassis::ParameterTransfer;
using little_chassis::LittleChassisNode;

void ParameterTransfer::DeclareParameters(rclcpp::Node &node)
{
    node.declare_parameter("mcu_param_timeout_ms", 1000);
    node.declare_parameter("mcu_param_max_retries", 3);

    node.declare_parameter("state_feedback_hz", 20);
    node.declare_parameter("odometry_feedback_hz", 50);
    node.declare_parameter("battery_feedback_hz", 1);
    node.declare_parameter("wheel_diameter", 0.15);
    node.declare_parameter("track_width", 0.30);
    node.declare_parameter("max_linear_accel", 1.0);
    node.declare_parameter("max_angular_accel", 1.0);
    node.declare_parameter("max_linear_vel", 1.0);
    node.declare_parameter("max_angular_vel", 2.0);
}

void ParameterTransfer::Init(const std::shared_ptr<LittleChassisNode> &node)
{
    if (!node)
        return;

    node_ = node;
    LoadParameters(node);

    node->RegisterMessageCallback(ROS_FEEDBACK_PARAMETERS,
                                  [this](const uint8_t *data, std::size_t len)
                                  { this->HandleParametersResponse(data, len); });

    initialized_ = true;
}

void ParameterTransfer::OnMcuOnline()
{
    if (!initialized_)
        return;

    parametersSynced_.store(false, std::memory_order_relaxed);
    parameterRetryCount_.store(0, std::memory_order_relaxed);
    SendParameters();

    auto node = node_.lock();
    if (!node)
        return;

    parameterTimeoutTimer_ = node->create_wall_timer(
        parameterTimeout_, [this]
        { this->EvaluateParameterResponse(); });
}

void ParameterTransfer::OnMcuOffline()
{
    parametersSynced_.store(false, std::memory_order_relaxed);
    if (parameterTimeoutTimer_)
        parameterTimeoutTimer_->cancel();
}

void ParameterTransfer::SendParameters()
{
    auto node = node_.lock();
    if (!node)
        return;

    auto msg = parameterMessage_;
    msg.messageID = parameterMessageId_.fetch_add(1, std::memory_order_relaxed);
    msg.success = 0;
    node->SendToMcu(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg));
}

void ParameterTransfer::HandleParametersResponse(const uint8_t *data, std::size_t len)
{
    if (!data || len < sizeof(FeedbackParametersMessage_t))
        return;

    const auto *resp = reinterpret_cast<const FeedbackParametersMessage_t *>(data);
    if (resp->success)
    {
        parametersSynced_.store(true, std::memory_order_relaxed);
        if (parameterTimeoutTimer_)
            parameterTimeoutTimer_->cancel();

        auto node = node_.lock();
        if (node)
        {
            RCLCPP_INFO(node->get_logger(), "MCU parameters acknowledged");
        }
    }
}

void ParameterTransfer::EvaluateParameterResponse()
{
    if (parametersSynced_.load(std::memory_order_relaxed))
        return;

    auto node = node_.lock();
    if (!node)
        return;

    const int attempts = parameterRetryCount_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (attempts > parameterMaxRetries_)
    {
        if (parameterTimeoutTimer_)
            parameterTimeoutTimer_->cancel();

        node->ForceMcuOffline("MCU parameter sync failed after retries - marking offline");
        return;
    }

    RCLCPP_WARN(node->get_logger(), "MCU parameter ack timeout, retry %d/%d", attempts, parameterMaxRetries_);
    SendParameters();
}

void ParameterTransfer::LoadParameters(const std::shared_ptr<LittleChassisNode> &node)
{
    int paramTimeoutMs{1000};
    int paramMaxRetries{3};
    node->get_parameter("mcu_param_timeout_ms", paramTimeoutMs);
    node->get_parameter("mcu_param_max_retries", paramMaxRetries);

    parameterTimeout_ = std::chrono::milliseconds(std::max(10, paramTimeoutMs));
    parameterMaxRetries_ = std::max(1, paramMaxRetries);

    ParametersMessage_t params{};
    params.messageType = ROS_CMD_PARAMETERS;
    params.messageID = 0;
    params.success = 0;
    node->get_parameter("state_feedback_hz", params.stateFeedbackFrequency);
    node->get_parameter("odometry_feedback_hz", params.odometryFeedbackFrequency);
    node->get_parameter("battery_feedback_hz", params.batteryFeedbackFrequency);
    node->get_parameter("wheel_diameter", params.wheelDiameter);
    node->get_parameter("track_width", params.trackWidth);
    node->get_parameter("max_linear_accel", params.maxLinearAcceleration);
    node->get_parameter("max_angular_accel", params.maxAngularAcceleration);
    node->get_parameter("max_linear_vel", params.maxLinearVelocity);
    node->get_parameter("max_angular_vel", params.maxAngularVelocity);

    parameterMessage_ = params;
}

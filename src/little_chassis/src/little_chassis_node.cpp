#include "little_chassis/little_chassis_node.hpp"
#include "little_chassis/ros_messages.h"
#include "little_chassis/topics/chassis_state_publisher.hpp"
#include "little_chassis/topics/cmd_vel_subscriber.hpp"
#include "little_chassis/topics/odom_publisher.hpp"
#include "little_chassis/services/io_control_service.hpp"
#include "little_chassis/services/light_control_service.hpp"
#include "little_chassis/services/motion_control_service.hpp"
#include "little_chassis/params_transfer/parameter_transfer.hpp"
using little_chassis::LittleChassisNode;

/**
 * @brief Constructor for LittleChassisNode.
 * 
 * This function initializes the node, sets up parameters, subscriptions,
 * publishers, and Boost.Asio sockets for communication with the MCU.
 */
LittleChassisNode::LittleChassisNode()
    : Node("little_chassis"),
      cmdSock_(ioCtx_)
{
    /* Declare & get parameters */
    declare_parameter("mcu_ip", "192.168.55.100");
    declare_parameter("mcu_cmd_port", 12000);
    declare_parameter("heartbeat_period_ms", 1000);
    declare_parameter("heartbeat_timeout_ms", 1500);
    ParameterTransfer::DeclareParameters(*this);

    std::string mcuIp;
    int cmdPort;
    int hbPeriodMs;
    int hbTimeoutMs;
    get_parameter("mcu_ip", mcuIp);
    get_parameter("mcu_cmd_port", cmdPort);
    get_parameter("heartbeat_period_ms", hbPeriodMs);
    get_parameter("heartbeat_timeout_ms", hbTimeoutMs);

    heartbeatPeriod_ = std::chrono::milliseconds(std::max(10, hbPeriodMs));
    heartbeatTimeout_ = std::chrono::milliseconds(std::max(hbPeriodMs, hbTimeoutMs));

    /* Boost.Asio sockets */
    RCLCPP_INFO(get_logger(), "Connecting to MCU at %s:%d", mcuIp.c_str(), cmdPort);

    mcuEndpoint_ = udp::endpoint(boost::asio::ip::make_address(mcuIp), cmdPort);
    cmdSock_.open(udp::v4());
    cmdSock_.set_option(udp::socket::reuse_address(true));
    cmdSock_.bind(udp::endpoint(udp::v4(), cmdPort));

    StartReceive();

    RegisterMessageCallback(ROS_HEART_BEAT, [this](const uint8_t *data, std::size_t len)
                               { this->HandleHeartbeat(data, len); });

    heartbeatTimer_ = this->create_wall_timer(
        heartbeatPeriod_, [this]
        { this->SendHeartbeat(); });

    heartbeatWatchdogTimer_ = this->create_wall_timer(
        heartbeatTimeout_ / 2, [this]
        { this->EvaluateHeartbeat(); });

    offlineLogTimer_ = this->create_wall_timer(
        std::chrono::seconds(2), [this]
        {
            if (!mcuOnline_.load(std::memory_order_relaxed))
            {
                RCLCPP_WARN(get_logger(), "MCU offline");
            }
        });

    /* Spin io_context in a background thread */
    running_ = true;
    ioThread_ = std::thread([this]
                             { ioCtx_.run(); });
}

void LittleChassisNode::InitializeInterfaces()
{
    if (interfacesInitialized_)
        return;

    auto self = std::dynamic_pointer_cast<LittleChassisNode>(shared_from_this());
    if (!self)
        return;

    if (!parameterTransfer_)
        parameterTransfer_ = std::make_unique<ParameterTransfer>();

    // Use fixed defaults: 1000ms timeout, 3 retries. Adjust here if needed.
    parameterTransfer_->Init(self, std::chrono::milliseconds(1000), 3);

    /* Topic subscribers/publishers */
    cmdVelSub_ = std::make_unique<CmdVelSubscriber>();
    cmdVelSub_->Init(self, "cmd_vel");

    odomPub_ = std::make_unique<OdomPublisher>();
    odomPub_->Init(self, "odom", "odom", "base_link");

    chassisStatePub_ = std::make_unique<ChassisStatePublisher>();
    chassisStatePub_->Init(self, "chassis_state");

    /* Service servers */
    ioControlService_ = std::make_unique<IoControlService>();
    ioControlService_->Init(self, "io_control", ROS_CMD_READ_IO);

    lightControlService_ = std::make_unique<LightControlService>();
    lightControlService_->Init(self, "light_control", ROS_CMD_LIGHT);

    motionControlService_ = std::make_unique<MotionControlService>();
    motionControlService_->Init(self, "motion_control", ROS_CMD_MOTION);

    interfacesInitialized_ = true;
}

/**
 * @brief Destructor for LittleChassisNode.
 * 
 * This function stops the io_context and joins the io_thread to ensure
 * proper cleanup of resources.
 */
LittleChassisNode::~LittleChassisNode()
{
    running_ = false;
    ioCtx_.stop();
    if (ioThread_.joinable())
        ioThread_.join();
}

void LittleChassisNode::RegisterMessageCallback(MessageType_t type,
                                                std::function<void(const uint8_t *, std::size_t)> callback)
{
    std::lock_guard<std::mutex> lock(callbacksMutex_);
    callbacks_[type] = std::move(callback);
}

void LittleChassisNode::SendToMcu(const uint8_t *data, std::size_t len)
{
    if (!data || len == 0)
        return;

    boost::system::error_code ec;
    cmdSock_.send_to(boost::asio::buffer(data, len), mcuEndpoint_, 0, ec);
    if (ec)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "send_to error: %s", ec.message().c_str());
    }
}

void LittleChassisNode::ForceMcuOffline(const char *reason)
{
    if (mcuOnline_.exchange(false, std::memory_order_relaxed))
    {
        const char *msg = reason ? reason : "MCU marked offline";
        RCLCPP_WARN(get_logger(), "%s", msg);
    }
    if (parameterTransfer_)
        parameterTransfer_->OnMcuOffline();
}

/**
 * @brief Start asynchronous receive operation on the UDP socket.
 * 
 * This function sets up the socket to listen for incoming packets and
 * binds the handler to process received data.
 */
void LittleChassisNode::StartReceive()
{
    cmdSock_.async_receive_from(
        boost::asio::buffer(recvBuf_), senderEndpoint_,
        std::bind(&LittleChassisNode::HandleReceive, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Handle received UDP packets.
 * 
 * This function processes the received data based on its type and publishes
 * relevant information such as motor speed and system status.
 * 
 * @param ec The error code indicating the result of the receive operation.
 * @param bytes_recvd The number of bytes received in the packet.
 */
void LittleChassisNode::HandleReceive(const boost::system::error_code &ec,
                                      std::size_t bytesRecvd)
{
    if (!ec && bytesRecvd > 0)
    {
        const uint32_t rawType = *reinterpret_cast<const uint32_t *>(recvBuf_.data());
        const auto msgType = static_cast<MessageType_t>(rawType);

        if (DispatchCallback(msgType, recvBuf_.data(), bytesRecvd))
        {
            if (running_)
                StartReceive();
            return;
        }
    }
    else if (ec != boost::asio::error::operation_aborted)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "recv error: %s", ec.message().c_str());
    }

    if (running_)
        StartReceive(); // re‑arm
}

bool LittleChassisNode::DispatchCallback(MessageType_t type, const uint8_t *data, std::size_t len)
{
    std::function<void(const uint8_t *, std::size_t)> cb;
    {
        std::lock_guard<std::mutex> lock(callbacksMutex_);
        auto it = callbacks_.find(type);
        if (it == callbacks_.end())
            return false;
        cb = it->second;
    }

    if (cb)
    {
        cb(data, len);
        return true;
    }
    return false;
}

void LittleChassisNode::SendHeartbeat()
{
    HeartBeatMessage_t hb{};
    hb.messageType = ROS_HEART_BEAT;
    hb.messageID = heartbeatMessageId_.fetch_add(1, std::memory_order_relaxed);
    hb.success = 0;
    hb.reset = 0;

    SendToMcu(reinterpret_cast<const uint8_t *>(&hb), sizeof(hb));
}

void LittleChassisNode::HandleHeartbeat(const uint8_t *data, std::size_t len)
{
    if (!data || len < sizeof(HeartBeatMessage_t))
        return;

    const auto nowNs = std::chrono::steady_clock::now().time_since_epoch().count();
    lastHeartbeatNs_.store(nowNs, std::memory_order_relaxed);

    if (!mcuOnline_.exchange(true, std::memory_order_relaxed))
    {
        RCLCPP_INFO(get_logger(), "MCU heartbeat online");
        if (parameterTransfer_)
            parameterTransfer_->OnMcuOnline();
    }
}

void LittleChassisNode::EvaluateHeartbeat()
{
    const auto now = std::chrono::steady_clock::now();
    const auto lastNs = lastHeartbeatNs_.load(std::memory_order_relaxed);
    if (lastNs == 0)
        return; // never received yet

    const auto last = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(lastNs));
    if (now - last > heartbeatTimeout_)
    {
        if (mcuOnline_.exchange(false, std::memory_order_relaxed))
        {
            RCLCPP_WARN(get_logger(), "MCU heartbeat timeout - marking offline");
            if (parameterTransfer_)
                parameterTransfer_->OnMcuOffline();
        }
    }
}

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "little_chassis/ros_messages.h"
#include <memory>
#include <chrono>

#include "little_chassis/params_transfer/parameter_transfer.hpp"

#include <boost/asio.hpp>
#include <array>
#include <thread>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <mutex>

namespace little_chassis
{
    class CmdVelSubscriber;
    class OdomPublisher;
    class ChassisStatePublisher;
    class IoControlService;
    class LightControlService;
    class MotionControlService;

    class LittleChassisNode : public rclcpp::Node
    {
    public:
        LittleChassisNode();
        ~LittleChassisNode();

        /**
         * @brief Initialize topic publishers/subscribers and services.
         * Call once after the node is constructed (requires shared_ptr).
         */
        void InitializeInterfaces();

        void RegisterMessageCallback(MessageType_t type,
                                      std::function<void(const uint8_t *, std::size_t)> callback);
        void SendToMcu(const uint8_t *data, std::size_t len);
        bool IsMcuOnline() const { return mcuOnline_.load(std::memory_order_relaxed); }
        void ForceMcuOffline(const char *reason);

    private:
        /* Boost.Asio ------------------------------------------*/
        using udp = boost::asio::ip::udp;
        boost::asio::io_context ioCtx_;
        udp::socket cmdSock_;
        udp::endpoint mcuEndpoint_;
        std::array<uint8_t, 128> recvBuf_{};
        udp::endpoint senderEndpoint_;
        std::thread ioThread_;
        std::atomic<bool> running_{false};

        /* Callbacks -------------------------------------------*/
        std::unordered_map<MessageType_t, std::function<void(const uint8_t *, std::size_t)>> callbacks_;
        std::mutex callbacksMutex_;

        void StartReceive();
        void HandleReceive(const boost::system::error_code &ec, std::size_t bytesRecvd);
        bool DispatchCallback(MessageType_t type, const uint8_t *data, std::size_t len);
        void SendHeartbeat();
        void HandleHeartbeat(const uint8_t *data, std::size_t len);
        void EvaluateHeartbeat();

        /* Heartbeat ----------------------------------------*/
        rclcpp::TimerBase::SharedPtr heartbeatTimer_;
        rclcpp::TimerBase::SharedPtr heartbeatWatchdogTimer_;
        std::atomic<uint32_t> heartbeatMessageId_{0};
        std::atomic<int64_t> lastHeartbeatNs_{0};
        std::atomic<bool> mcuOnline_{false};
        std::chrono::milliseconds heartbeatPeriod_{1000};
        std::chrono::milliseconds heartbeatTimeout_{1500};

        /* Parameter sync -----------------------------------*/
        std::unique_ptr<ParameterTransfer> parameterTransfer_;

        /* Topic interfaces ----------------------------------*/
        std::unique_ptr<CmdVelSubscriber> cmdVelSub_;
        std::unique_ptr<OdomPublisher> odomPub_;
        std::unique_ptr<ChassisStatePublisher> chassisStatePub_;

        /* Service interfaces --------------------------------*/
        std::unique_ptr<IoControlService> ioControlService_;
        std::unique_ptr<LightControlService> lightControlService_;
        std::unique_ptr<MotionControlService> motionControlService_;

        bool interfacesInitialized_{false};
    };
} // namespace little_chassis

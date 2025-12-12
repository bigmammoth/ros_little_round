#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <atomic>
#include <chrono>

#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    class LittleChassisNode;

    class ParameterTransfer
    {
    public:
        static void DeclareParameters(rclcpp::Node &node);

        void Init(const std::shared_ptr<LittleChassisNode> &node,
              std::chrono::milliseconds timeout,
              int maxRetries);
        void OnMcuOnline();
        void OnMcuOffline();

    private:
        void SendParameters();
        void HandleParametersResponse(const uint8_t *data, std::size_t len);
        void EvaluateParameterResponse();
        void LoadParameters(const std::shared_ptr<LittleChassisNode> &node);

        std::weak_ptr<LittleChassisNode> node_;
        rclcpp::TimerBase::SharedPtr parameterTimeoutTimer_;
        std::atomic<uint32_t> parameterMessageId_{0};
        ParametersMessage_t parameterMessage_{};
        std::atomic<int> parameterRetryCount_{0};
        int parameterMaxRetries_{3};
        std::chrono::milliseconds parameterTimeout_{1000};
        std::atomic<bool> parametersSynced_{false};
        bool initialized_{false};
    };
} // namespace little_chassis

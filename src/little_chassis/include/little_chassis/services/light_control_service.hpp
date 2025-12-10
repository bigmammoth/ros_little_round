#pragma once

#include <atomic>
#include <vector>
#include "little_chassis/services/service_server_base.hpp"
#include "little_chassis/ros_messages.h"
#include "little_chassis/srv/light_control.hpp"

namespace little_chassis
{
    class LightControlService : public ServiceServerBase<little_chassis::srv::LightControl>
    {
    protected:
        bool BuildMcuRequest(const std::shared_ptr<Request> &request,
                             std::vector<uint8_t> &outBuffer) override;

        bool HandleMcuResponse(const std::vector<uint8_t> &data,
                               Response &response) override;

    private:
        std::atomic<uint32_t> messageIdCounter_{0};
    };
} // namespace little_chassis

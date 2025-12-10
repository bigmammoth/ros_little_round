#include "little_chassis/services/light_control_service.hpp"

namespace little_chassis
{
bool LightControlService::BuildMcuRequest(const std::shared_ptr<Request> &request,
                                          std::vector<uint8_t> &outBuffer)
{
    LightMessage_t msg{};
    msg.messageType = ROS_CMD_LIGHT;
    msg.messageID = messageIdCounter_.fetch_add(1, std::memory_order_relaxed);
    msg.success = 0;
    msg.color = request->light.color;
    msg.mode = request->light.mode;
    msg.brightness = request->light.brightness;
    msg.frequency = request->light.frequency;

    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&msg);
    outBuffer.assign(ptr, ptr + sizeof(msg));
    return true;
}

bool LightControlService::HandleMcuResponse(const std::vector<uint8_t> &data,
                                            Response &response)
{
    if (data.size() < sizeof(LightMessage_t))
        return false;

    const auto *resp = reinterpret_cast<const LightMessage_t *>(data.data());
    response.success = (resp->success != 0);
    return response.success;
}
} // namespace little_chassis

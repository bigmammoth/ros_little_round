#include "little_chassis/services/motion_control_service.hpp"

namespace little_chassis
{
bool MotionControlService::BuildMcuRequest(const std::shared_ptr<Request> &request,
                                           std::vector<uint8_t> &outBuffer)
{
    MotionMessage_t msg{};
    msg.messageType = ROS_CMD_MOTION;
    msg.messageID = messageIdCounter_.fetch_add(1, std::memory_order_relaxed);
    msg.success = 0;
    msg.gearMode = static_cast<GearMode_t>(request->motion.gear_mode);
    msg.autoMode = request->motion.auto_mode;

    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&msg);
    outBuffer.assign(ptr, ptr + sizeof(msg));
    return true;
}

bool MotionControlService::HandleMcuResponse(const std::vector<uint8_t> &data,
                                             Response &response)
{
    if (data.size() < sizeof(MotionMessage_t))
        return false;

    const auto *resp = reinterpret_cast<const MotionMessage_t *>(data.data());
    response.success = (resp->success != 0);
    return response.success;
}
} // namespace little_chassis

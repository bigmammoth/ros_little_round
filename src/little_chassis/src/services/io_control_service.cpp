#include "little_chassis/services/io_control_service.hpp"

namespace little_chassis
{
bool IoControlService::BuildMcuRequest(const std::shared_ptr<Request> &request,
                                       std::vector<uint8_t> &outBuffer)
{
    if (request->io_state.pin_count > MAX_IO_PINS)
        return false;

    SetIoMessage_t msg{};
    msg.messageType = ROS_CMD_SET_IO;
    msg.messageID = messageIdCounter_.fetch_add(1, std::memory_order_relaxed);
    msg.success = 0;
    msg.pinCount = request->io_state.pin_count;
    for (uint32_t i = 0; i < msg.pinCount; ++i)
    {
        msg.pins[i] = request->io_state.pins[i];
    }

    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&msg);
    outBuffer.assign(ptr, ptr + sizeof(msg));
    return true;
}

bool IoControlService::HandleMcuResponse(const std::vector<uint8_t> &data,
                                         Response &response)
{
    if (data.size() < sizeof(ReadIoMessage_t))
        return false;

    const auto *resp = reinterpret_cast<const ReadIoMessage_t *>(data.data());
    response.success = (resp->success != 0);
    return response.success;
}
} // namespace little_chassis

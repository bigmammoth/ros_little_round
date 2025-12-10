#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "little_chassis/little_chassis_node.hpp"
#include "little_chassis/ros_messages.h"

namespace little_chassis
{
    /**
     * @brief Generic service server base with MCU request/response handling and retry.
     *
     * Derived classes implement BuildMcuRequest to serialize the outbound MCU message
     * and HandleMcuResponse to parse the MCU reply into the ROS service response.
     * The base handles service registration, MCU response routing, timeout, and retry.
     */
    template <typename ServiceT>
    class ServiceServerBase
    {
    public:
        using Request = typename ServiceT::Request;
        using Response = typename ServiceT::Response;

        ServiceServerBase() = default;
        virtual ~ServiceServerBase() = default;

        /**
         * @brief Initialize the service server.
         * @param node Owning chassis node (provides SendToMcu and message callbacks).
         * @param serviceName ROS service name to advertise.
         * @param responseType MCU response MessageType_t to listen for.
         * @param timeout Timeout per try when waiting for MCU reply.
         * @param maxRetries Number of send attempts before failing.
         */
        void Init(const std::shared_ptr<LittleChassisNode> &node,
                  const std::string &serviceName,
                  MessageType_t responseType,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(500),
                  int maxRetries = 3)
        {
            node_ = node;
            responseType_ = responseType;
            timeout_ = timeout;
            maxRetries_ = maxRetries;

            if (auto n = node_.lock())
            {
                n->RegisterMessageCallback(responseType_,
                                           [this](const uint8_t *data, std::size_t len)
                                           {
                                               this->OnMcuResponse(data, len);
                                           });

                service_ = n->template create_service<ServiceT>(
                    serviceName,
                    std::bind(&ServiceServerBase::OnServiceRequest, this,
                              std::placeholders::_1, std::placeholders::_2));
            }
        }

    protected:
        /**
         * @brief Build outbound MCU request message from ROS service request.
         * @return true if message should be sent; false to fail immediately.
         */
        virtual bool BuildMcuRequest(const std::shared_ptr<Request> &request,
                                     std::vector<uint8_t> &outBuffer) = 0;

        /**
         * @brief Parse MCU response into ROS service response. Return true on success.
         */
        virtual bool HandleMcuResponse(const std::vector<uint8_t> &data,
                                       Response &response) = 0;

        std::shared_ptr<LittleChassisNode> GetNode() const { return node_.lock(); }

    private:
        void OnServiceRequest(const std::shared_ptr<Request> request,
                              std::shared_ptr<Response> response)
        {
            response->success = false;
            auto node = node_.lock();
            if (!node)
                return;

            for (int attempt = 0; attempt < maxRetries_; ++attempt)
            {
                std::vector<uint8_t> buffer;
                if (!BuildMcuRequest(request, buffer) || buffer.empty())
                {
                    response->success = false;
                    return;
                }

                {
                    std::lock_guard<std::mutex> lk(respMutex_);
                    hasResponse_ = false;
                }

                node->SendToMcu(buffer.data(), buffer.size());

                std::vector<uint8_t> mcuReply;
                if (WaitForResponse(mcuReply))
                {
                    response->success = HandleMcuResponse(mcuReply, *response);
                    return;
                }
            }

            response->success = false; // timed out after retries
        }

        bool WaitForResponse(std::vector<uint8_t> &out)
        {
            std::unique_lock<std::mutex> lk(respMutex_);
            const bool got = respCv_.wait_for(lk, timeout_, [this]
                                              { return hasResponse_; });
            if (!got)
                return false;

            out = lastResponse_;
            hasResponse_ = false;
            return true;
        }

        void OnMcuResponse(const uint8_t *data, std::size_t len)
        {
            if (!data || len == 0)
                return;

            std::lock_guard<std::mutex> lk(respMutex_);
            lastResponse_.assign(data, data + len);
            hasResponse_ = true;
            respCv_.notify_one();
        }

        std::weak_ptr<LittleChassisNode> node_;
        MessageType_t responseType_{};
        std::chrono::milliseconds timeout_{500};
        int maxRetries_{3};

        typename rclcpp::Service<ServiceT>::SharedPtr service_;

        std::mutex respMutex_;
        std::condition_variable respCv_;
        std::vector<uint8_t> lastResponse_;
        bool hasResponse_{false};
    };
} // namespace little_chassis

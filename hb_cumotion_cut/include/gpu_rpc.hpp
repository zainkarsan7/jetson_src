#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <hb_cumotion_cut/srv/solve_process_path.hpp>
#include <hb_cumotion_cut/srv/plan_transit.hpp>

namespace hb_robot_skills {
inline bool sameJoints(const std::vector<std::string>& got, const std::vector<std::string>& wanted) {
    return got.size() == wanted.size() && std::set<std::string>(got.begin(), got.end()) ==
        std::set<std::string>(wanted.begin(), wanted.end());
}

inline bool finitePositions(const std::vector<double>& v, size_t n) {
    return v.size() == n && std::all_of(v.begin(), v.end(), [](double x){ return std::isfinite(x); });
}

template<class Service>
typename Service::Response::SharedPtr gpuRequest(
    const typename rclcpp::Client<Service>::SharedPtr& client,
    const typename Service::Request::SharedPtr& request) {
    using namespace std::chrono_literals;
    if (!client->wait_for_service(2s))
        throw std::runtime_error(std::string("Missing GPU service ") + client->get_service_name());
    auto future = client->async_send_request(request);
    if (future.wait_for(60s) != std::future_status::ready) {
        client->remove_pending_request(future);
        throw std::runtime_error(std::string("GPU service timeout: ") + client->get_service_name());
    }
    auto response = future.get();
    if (response->backend_error) throw std::runtime_error(response->message);
    return response;
}
}  // namespace hb_robot_skills

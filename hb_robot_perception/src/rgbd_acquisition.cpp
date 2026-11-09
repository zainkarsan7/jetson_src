#include "hb_robot_perception/rgbd_acquisition.hpp"
#include <cmath>
#include <functional>
#include <tf2/exceptions.h>

namespace hb_perception {
using namespace std::chrono_literals;

RGBDAcquisition::RGBDAcquisition(rclcpp::Node* node, tf2_ros::Buffer* tf_buffer,
    const std::string& target_frame, const std::string& rgb_topic,
    const std::string& depth_topic, const std::string& camera_info_topic)
    : node_(node), tf_buffer_(tf_buffer), target_frame_(target_frame)
{
    rgb_sub_.subscribe(node_, rgb_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(node_, depth_topic, rmw_qos_profile_sensor_data);
    synchronizer_ =
        std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(15), rgb_sub_, depth_sub_);
        //     synchronizer_->setInterMessageLowerBound(
        //     0, rclcpp::Duration(30ms));

        // synchronizer_->setInterMessageLowerBound(
        //     1, rclcpp::Duration(30ms));
    synchronizer_->setMaxIntervalDuration(
        rclcpp::Duration(40ms));

    synchronizer_->registerCallback(
        std::bind(
            &RGBDAcquisition::synchronizedCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    retry_timer_ = node_->create_wall_timer(50ms, [this] { resolvePending(); });
    retry_timer_->cancel();
    camera_info_sub_ = node_->create_subscription<CameraInfo>(camera_info_topic,
        rclcpp::SensorDataQoS().keep_last(1), [this](CameraInfo::ConstSharedPtr info) {
            camera_info_ = std::move(info);
            camera_info_sub_.reset();
            resolvePending();
        });
}

void RGBDAcquisition::synchronizedCallback(const Image::ConstSharedPtr& rgb,
    const Image::ConstSharedPtr& depth)
{ 
        // RCLCPP_WARN_THROTTLE(
        // node_->get_logger(),
        // *node_->get_clock(),
        // 2000,
        // "RGBD synchronized: RGB=%.3f depth=%.3f "
        // "delta=%.1f ms now=%.3f",
        // rclcpp::Time(rgb->header.stamp).seconds(),
        // rclcpp::Time(depth->header.stamp).seconds(),
        // std::abs(
        //     (rclcpp::Time(rgb->header.stamp) -
        //     rclcpp::Time(depth->header.stamp)).seconds()) * 1000.0,
        // node_->now().seconds());
    // TF treats stamp zero as a request for the latest transform.
    if (rclcpp::Time(rgb->header.stamp).nanoseconds() == 0 ||
        rclcpp::Time(depth->header.stamp).nanoseconds() == 0) return;

    if (pending_.size() == 20) pending_.pop_front();
    pending_.push_back({rgb, depth, std::chrono::steady_clock::now()});
    resolvePending();
    ++synchronized_count_;
}

void RGBDAcquisition::resolvePending()
{
    while (!pending_.empty() &&
           std::chrono::steady_clock::now() - pending_.front().arrival > 2000ms)
           {
           ++expired_count_;
            pending_.pop_front();
           }
    // Prefer the newest resolvable capture. TF may already cover an earlier
    // pair while the newest still needs a future bracketing transform sample.
    for (size_t i = pending_.size(); camera_info_ && i > 0; --i) {
        const auto& pair = pending_[i - 1];
        const rclcpp::Time stamp(pair.rgb->header.stamp);
        geometry_msgs::msg::TransformStamped pose;
        try {
            pose = tf_buffer_->lookupTransform(target_frame_, pair.rgb->header.frame_id,
                tf2_ros::fromRclcpp(stamp));
        } catch (const tf2::TransformException& ex) {
           
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "RGBD TF unavailable: "
        "camera=%s target=%s "
        "image_stamp=%.3f now=%.3f "
        "age=%.3f s | %s",
        pair.rgb->header.frame_id.c_str(),
        target_frame_.c_str(),
        stamp.seconds(),
        node_->now().seconds(),
        (node_->now() - stamp).seconds(),
        ex.what());

    continue;

        }
        {
            std::lock_guard<std::mutex> lock(observation_mutex_);
            latest_observation_ = Observation{pair.rgb, pair.depth, camera_info_, std::move(pose), stamp};
        }
        ++resolved_count_;
        observation_cv_.notify_all();
        RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    2000,
    "RGBD observation READY: stamp=%.3f age=%.3f s",
    stamp.seconds(),
    (node_->now() - stamp).seconds());
        pending_.erase(pending_.begin(), pending_.begin() + i);
        break;
    }
    if (pending_.empty()) retry_timer_->cancel();
    else if (retry_timer_->is_canceled()) retry_timer_->reset();

    // RCLCPP_INFO_THROTTLE(
    // node_->get_logger(),
    // *node_->get_clock(),
    // 5000,
    // "RGBD pipeline: synchronized=%zu "
    // "resolved=%zu expired=%zu pending=%zu",
    // synchronized_count_,
    // resolved_count_,
    // expired_count_,
    // pending_.size());
}

std::optional<Observation> RGBDAcquisition::acquireAfter(const rclcpp::Time& min_stamp,
    std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(observation_mutex_);
    if (!observation_cv_.wait_for(lock, timeout, [&] {
        return latest_observation_ && latest_observation_->stamp > min_stamp &&
            rclcpp::Time(latest_observation_->depth_to_rgb->header.stamp) > min_stamp;
    })) {
        RCLCPP_WARN(node_->get_logger(), "No calibrated RGB-D pair with a pose after %.6f within %ld ms",
            min_stamp.seconds(), static_cast<long>(timeout.count()));
        return std::nullopt;
    }
    return latest_observation_;
}

std::optional<Observation> RGBDAcquisition::latest() const
{
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return latest_observation_;
}

}  // namespace hb_perception

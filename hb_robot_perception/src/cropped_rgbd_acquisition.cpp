
#include "hb_robot_perception/cropped_rgbd_acquisition.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.h>

namespace hb_perception {

using namespace std::chrono_literals;

CroppedRGBDAcquisition::CroppedRGBDAcquisition(
    rclcpp::Node* node,
    tf2_ros::Buffer* tf_buffer,
    const std::string& target_frame,
    const std::string& rgbd_topic)
    : CroppedRGBDAcquisition(
          node,
          tf_buffer,
          target_frame,
          Mode::Continuous,
          rgbd_topic)
{
}

CroppedRGBDAcquisition::CroppedRGBDAcquisition(
    rclcpp::Node* node,
    tf2_ros::Buffer* tf_buffer,
    const std::string& target_frame,
    Mode mode,
    const std::string& rgbd_topic)
    : node_(node),
      tf_buffer_(tf_buffer),
      target_frame_(target_frame),
      mode_(mode)
{
    if (!node_ || !tf_buffer_ ||
        target_frame_.empty() || rgbd_topic.empty())
    {   
        RCLCPP_INFO(
        node_->get_logger(),
        "Cropped RGBD acquisition initialized: "
        "mode=%s topic=%s target=%s",
        mode_ == Mode::CaptureWindow
            ? "capture_window"
            : "continuous",
        rgbd_topic.c_str(),
        target_frame_.c_str());

        throw std::invalid_argument(
            "Invalid cropped RGBD acquisition configuration");
    }

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        mode_ == Mode::Continuous);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    if (mode_ == Mode::CaptureWindow)
    {
        rclcpp::ExecutorOptions executor_options;

        executor_options.context =
            node_->get_node_base_interface()
                ->get_context();

        window_executor_ = std::make_unique<
            rclcpp::executors::SingleThreadedExecutor>(
                executor_options);

        window_executor_->add_callback_group(
            callback_group_,
            node_->get_node_base_interface());
    }

    // Exactly one image subscription.
    // Matches the driver's BEST_EFFORT sensor QoS.
    rgbd_sub_ = node_->create_subscription<RGBDFrame>(
        rgbd_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](RGBDFrame::ConstSharedPtr frame)
        {
            frameCallback(std::move(frame));
        },
        options);

    retry_timer_ = node_->create_wall_timer(
        50ms,
        [this]()
        {
            resolvePending();
        },
        callback_group_);

    retry_timer_->cancel();

    diagnostics_timer_ = node_->create_wall_timer(
        5000ms,
        [this]()
        {
            logDiagnostics();
        },
        callback_group_);

    // CaptureWindow has its own executor so acquireAfter()
    // can block on an action worker without starving images.
    if (window_executor_)
    {
        window_thread_ = std::thread(
            [this]()
            {
                window_executor_->spin();
            });
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Cropped RGBD acquisition initialized: "
        "mode=%s topic=%s target=%s",
        mode_ == Mode::CaptureWindow
            ? "capture_window"
            : "continuous",
        rgbd_topic.c_str(),
        target_frame_.c_str());
}

CroppedRGBDAcquisition::~CroppedRGBDAcquisition()
{
    if (window_executor_)
    {
        window_executor_->cancel();

        if (window_thread_.joinable())
            window_thread_.join();

        window_executor_->remove_callback_group(
            callback_group_);
    }

    // As with the original RGBDAcquisition,
    // Continuous mode requires the owning executor
    // to be stopped before this object is destroyed.
    if (retry_timer_)
        retry_timer_->cancel();

    if (diagnostics_timer_)
        diagnostics_timer_->cancel();

    rgbd_sub_.reset();
    retry_timer_.reset();
    diagnostics_timer_.reset();

    pending_.reset();

    RCLCPP_INFO(
        node_->get_logger(),
        "Cropped RGBD stopped: "
        "received=%zu invalid=%zu resolved=%zu "
        "tf_failures=%zu expired=%zu superseded=%zu",
        received_count_,
        invalid_count_,
        resolved_count_,
        tf_failure_count_,
        expired_count_,
        superseded_count_);
}

bool CroppedRGBDAcquisition::validFrame(
    const RGBDFrame& frame) const
{
    const auto& rgb = frame.rgb;
    const auto& depth = frame.depth;
    const auto& info = frame.camera_info;

    auto reject = [&](const char* reason) -> bool
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Cropped RGBD validation failed: %s",
            reason);

        return false;
    };

    if (rgb.width == 0 || rgb.height == 0)
        return reject("RGB dimensions are zero");

    if (rgb.width != depth.width ||
        rgb.height != depth.height)
        return reject("RGB/depth dimensions differ");

    if (info.width != rgb.width ||
        info.height != rgb.height)
        return reject("CameraInfo dimensions differ");

    if (rgb.encoding != "bgra8")
        return reject("Unexpected RGB encoding");

    if (depth.encoding != "32FC1")
        return reject("Unexpected depth encoding");

    if (rgb.is_bigendian != 0 ||
        depth.is_bigendian != 0)
        return reject("Unexpected image endianness");

    if (rgb.step < rgb.width * 4 ||
        depth.step < depth.width * 4)
        return reject("Image row stride is too small");

    if (rgb.data.size() <
            static_cast<std::size_t>(rgb.step) *
                rgb.height ||
        depth.data.size() <
            static_cast<std::size_t>(depth.step) *
                depth.height)
        return reject("Image data buffer is too small");

    const rclcpp::Time rgb_stamp(rgb.header.stamp);
    const rclcpp::Time depth_stamp(depth.header.stamp);

    if (rgb_stamp.nanoseconds() == 0 ||
        depth_stamp.nanoseconds() == 0)
        return reject("Image timestamp is zero");

    const auto skew_ns =
    (rgb_stamp - depth_stamp).nanoseconds();

    const double skew_ms =
        std::abs(static_cast<double>(skew_ns)) / 1e6;

    // Allow a small difference between the RGB and depth
    // timestamps within the driver's paired capture.
    constexpr double max_skew_ms = 100.0;

    if (skew_ms > max_skew_ms)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "RGBD timestamp skew too large: %.3f ms",
            skew_ms);

        return false;
    }

    if (rgb.header.frame_id.empty())
        return reject("RGB frame_id is empty");

    if (depth.header.frame_id != rgb.header.frame_id)
        return reject("RGB/depth frame_ids differ");

    if (info.header.frame_id != rgb.header.frame_id)
        return reject("CameraInfo frame_id differs");

    if (!std::isfinite(info.k[0]) ||
        !std::isfinite(info.k[4]) ||
        !std::isfinite(info.k[2]) ||
        !std::isfinite(info.k[5]) ||
        info.k[0] <= 0.0 ||
        info.k[4] <= 0.0)
        return reject("Invalid camera intrinsics");

    return true;
}

void CroppedRGBDAcquisition::frameCallback(
    RGBDFrame::ConstSharedPtr frame)
{
    ++received_count_;

    if (!frame || !validFrame(*frame))
    {
        ++invalid_count_;

        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Invalid cropped RGBD frame received");

        return;
    }

    // Keep only the newest frame awaiting TF.
    // No image copying or synchronization.
    if (pending_)
        ++superseded_count_;

    pending_ = PendingFrame{
        std::move(frame),
        std::chrono::steady_clock::now()
    };

    resolvePending();
}

void CroppedRGBDAcquisition::resolvePending()
{
    if (!pending_)
    {
        retry_timer_->cancel();
        return;
    }

    if (std::chrono::steady_clock::now() -
            pending_->arrival > 1000ms)
    {
        ++expired_count_;

        pending_.reset();
        retry_timer_->cancel();
        return;
    }

    const auto& frame = pending_->frame;

    const rclcpp::Time stamp(
        frame->rgb.header.stamp);

    geometry_msgs::msg::TransformStamped camera_pose;

    try
    {
        camera_pose = tf_buffer_->lookupTransform(
            target_frame_,
            frame->rgb.header.frame_id,
            tf2_ros::fromRclcpp(stamp));
    }
    catch (const tf2::TransformException& ex)
    {
        ++tf_failure_count_;

        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Cropped RGBD TF unavailable: %s",
            ex.what());

        if (retry_timer_->is_canceled())
            retry_timer_->reset();

        return;
    }

    // Aliasing shared_ptrs:
    // The RGBDFrame owns all three embedded messages.
    // These pointers keep the entire RGBDFrame alive,
    // without copying either image or CameraInfo.
    Image::ConstSharedPtr rgb(
        frame,
        &frame->rgb);

    Image::ConstSharedPtr depth(
        frame,
        &frame->depth);

    CameraInfo::ConstSharedPtr info(
        frame,
        &frame->camera_info);

    Observation observation{
        std::move(rgb),
        std::move(depth),
        std::move(info),
        std::move(camera_pose),
        stamp
    };

    bool updated = false;

    {
        std::lock_guard<std::mutex> lock(
            observation_mutex_);

        // Preserve monotonic observation timestamps.
        if (!latest_observation_ ||
            observation.stamp >
                latest_observation_->stamp)
        {
            latest_observation_ =
                std::move(observation);

            updated = true;
        }
    }

    ++resolved_count_;

    pending_.reset();
    retry_timer_->cancel();

    if (updated)
        observation_cv_.notify_all();
}

std::optional<Observation>
CroppedRGBDAcquisition::acquireAfter(
    const rclcpp::Time& min_stamp,
    std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(
        observation_mutex_);

    const bool received = observation_cv_.wait_for(
        lock,
        timeout,
        [&]()
        {
            return latest_observation_ &&
                latest_observation_->stamp > min_stamp &&
                rclcpp::Time(
                    latest_observation_
                        ->depth_to_rgb->header.stamp)
                    > min_stamp;
        });

    if (!received)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "No fresh cropped RGBD observation after "
            "%.6f within %ld ms",
            min_stamp.seconds(),
            static_cast<long>(timeout.count()));

        return std::nullopt;
    }

    return latest_observation_;
}

std::optional<Observation>
CroppedRGBDAcquisition::latest() const
{
    std::lock_guard<std::mutex> lock(
        observation_mutex_);

    return latest_observation_;
}

void CroppedRGBDAcquisition::logCaptureFailure(
    const rclcpp::Time& boundary)
{
    const auto observation = latest();

    if (!observation)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Cropped RGBD capture failure: "
            "no resolved observation; boundary=%.6f",
            boundary.seconds());

        return;
    }

    const rclcpp::Time rgb_stamp =
        observation->stamp;

    const rclcpp::Time depth_stamp(
        observation->depth_to_rgb->header.stamp);

    const rclcpp::Time current_time =
        node_->now();

    RCLCPP_WARN(
        node_->get_logger(),
        "Cropped RGBD capture failure: "
        "boundary=%.6f "
        "latest_rgb=%.6f latest_depth=%.6f "
        "rgb_age=%.3f s depth_age=%.3f s "
        "rgb_before_boundary=%.3f s",
        boundary.seconds(),
        rgb_stamp.seconds(),
        depth_stamp.seconds(),
        (current_time - rgb_stamp).seconds(),
        (current_time - depth_stamp).seconds(),
        (boundary - rgb_stamp).seconds());
}

void CroppedRGBDAcquisition::logDiagnostics()
{
    RCLCPP_DEBUG(
        node_->get_logger(),
        "Cropped RGBD: "
        "received=%zu invalid=%zu resolved=%zu "
        "tf_failures=%zu expired=%zu superseded=%zu",
        received_count_,
        invalid_count_,
        resolved_count_,
        tf_failure_count_,
        expired_count_,
        superseded_count_);
}

}  // namespace hb_perception

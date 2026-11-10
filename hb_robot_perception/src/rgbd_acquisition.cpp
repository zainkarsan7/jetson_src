
#include "hb_robot_perception/rgbd_acquisition.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include <rmw/qos_profiles.h>
#include <tf2/exceptions.h>

namespace hb_perception {

using namespace std::chrono_literals;

RGBDAcquisition::RGBDAcquisition(
    rclcpp::Node* node,
    tf2_ros::Buffer* tf_buffer,
    const std::string& target_frame,
    const std::string& rgb_topic,
    const std::string& depth_topic,
    const std::string& camera_info_topic)
    : RGBDAcquisition(
          node,
          tf_buffer,
          target_frame,
          Mode::Continuous,
          rgb_topic,
          depth_topic,
          camera_info_topic)
{
}

RGBDAcquisition::RGBDAcquisition(
    rclcpp::Node* node,
    tf2_ros::Buffer* tf_buffer,
    const std::string& target_frame,
    Mode mode,
    const std::string& rgb_topic,
    const std::string& depth_topic,
    const std::string& camera_info_topic)
    : node_(node),
      tf_buffer_(tf_buffer),
      target_frame_(target_frame),
      mode_(mode)
{
    // Dedicated mutually exclusive group for acquisition.
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        mode_ == Mode::Continuous);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    if (mode_ == Mode::CaptureWindow)
    {
        rclcpp::ExecutorOptions executor_options;
        executor_options.context =
            node_->get_node_base_interface()->get_context();

        window_executor_ = std::make_unique<
            rclcpp::executors::SingleThreadedExecutor>(
                executor_options);

        window_executor_->add_callback_group(
            callback_group_,
            node_->get_node_base_interface());
    }

    // Restore the original DDS sensor-data QoS.
    rgb_sub_.subscribe(
        node_,
        rgb_topic,
        rmw_qos_profile_sensor_data,
        options);

    depth_sub_.subscribe(
        node_,
        depth_topic,
        rmw_qos_profile_sensor_data,
        options);

    // Diagnostic taps. These do not replace the synchronizer.
    rgb_sub_.registerCallback(
        [this](const Image::ConstSharedPtr&)
        {
            ++rgb_received_;
        });

    depth_sub_.registerCallback(
        [this](const Image::ConstSharedPtr&)
        {
            ++depth_received_;
        });

    // Restore the original synchronization behavior.
    synchronizer_ = std::make_shared<
        message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(5),
            rgb_sub_,
            depth_sub_);

    // Intentionally no setMaxIntervalDuration().
    // Intentionally no setInterMessageLowerBound().
    synchronizer_->registerCallback(
        std::bind(
            &RGBDAcquisition::synchronizedCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    camera_info_sub_ =
        node_->create_subscription<CameraInfo>(
            camera_info_topic,
            rclcpp::SensorDataQoS().keep_last(1),
            [this](CameraInfo::ConstSharedPtr info)
            {
                if (!camera_info_)
                {
                    camera_info_ = std::move(info);

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "RGBD camera calibration received");
                }

                resolvePending();
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

    // CaptureWindow has its own executor.
    // Continuous is serviced by the owning node's executor.
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
        "RGBD acquisition initialized: "
        "mode=%s rgb=%s depth=%s target=%s",
        mode_ == Mode::CaptureWindow
            ? "capture_window"
            : "continuous",
        rgb_topic.c_str(),
        depth_topic.c_str(),
        target_frame_.c_str());
}

RGBDAcquisition::~RGBDAcquisition()
{
    if (window_executor_)
    {
        window_executor_->cancel();

        if (window_thread_.joinable())
            window_thread_.join();

        window_executor_->remove_callback_group(
            callback_group_);
    }

    // For Continuous mode, the owning node/executor must
    // be stopped before destruction of this object.
    if (retry_timer_)
        retry_timer_->cancel();

    if (diagnostics_timer_)
        diagnostics_timer_->cancel();

    synchronizer_.reset();
    rgb_sub_.unsubscribe();
    depth_sub_.unsubscribe();

    camera_info_sub_.reset();
    retry_timer_.reset();
    diagnostics_timer_.reset();

    RCLCPP_INFO(
        node_->get_logger(),
        "RGBD stopped: rgb=%zu depth=%zu "
        "synchronized=%zu resolved=%zu "
        "tf_failures=%zu expired=%zu",
        rgb_received_,
        depth_received_,
        synchronized_count_,
        resolved_count_,
        tf_failure_count_,
        expired_count_);
}

void RGBDAcquisition::synchronizedCallback(
    const Image::ConstSharedPtr& rgb,
    const Image::ConstSharedPtr& depth)
{
    ++synchronized_count_;

    const rclcpp::Time rgb_stamp(rgb->header.stamp);
    const rclcpp::Time depth_stamp(depth->header.stamp);

    if (rgb_stamp.nanoseconds() == 0 ||
        depth_stamp.nanoseconds() == 0)
    {
        return;
    }

    // Keep only the newest pair awaiting TF.
    pending_ = PendingPair{
        rgb,
        depth,
        std::chrono::steady_clock::now()
    };

    resolvePending();
}

void RGBDAcquisition::resolvePending()
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

    if (!camera_info_)
    {
        if (retry_timer_->is_canceled())
            retry_timer_->reset();

        return;
    }

    const auto stamp =
        rclcpp::Time(pending_->rgb->header.stamp);

    geometry_msgs::msg::TransformStamped camera_pose;

    try
    {
        camera_pose = tf_buffer_->lookupTransform(
            target_frame_,
            pending_->rgb->header.frame_id,
            tf2_ros::fromRclcpp(stamp));
    }
    catch (const tf2::TransformException& ex)
    {
        ++tf_failure_count_;

        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "RGBD TF unavailable: %s",
            ex.what());

        if (retry_timer_->is_canceled())
            retry_timer_->reset();

        return;
    }

    Observation observation{
        pending_->rgb,
        pending_->depth,
        camera_info_,
        std::move(camera_pose),
        stamp
    };

    {
        std::lock_guard<std::mutex> lock(
            observation_mutex_);

        // Preserve monotonic observation timestamps.
        if (!latest_observation_ ||
            observation.stamp > latest_observation_->stamp)
        {
            latest_observation_ = std::move(observation);
        }
    }

    ++resolved_count_;

    pending_.reset();
    retry_timer_->cancel();

    observation_cv_.notify_all();
}

std::optional<Observation>
RGBDAcquisition::acquireAfter(
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
            "No fresh RGBD observation after %.6f "
            "within %ld ms",
            min_stamp.seconds(),
            static_cast<long>(timeout.count()));

        return std::nullopt;
    }

    return latest_observation_;
}

std::optional<Observation>
RGBDAcquisition::latest() const
{
    std::lock_guard<std::mutex> lock(
        observation_mutex_);

    return latest_observation_;
}


void RGBDAcquisition::logCaptureFailure(
    const rclcpp::Time& boundary)
{
    const auto observation = latest();

    if (!observation)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "RGBD capture failure: no resolved observation; "
            "boundary=%.6f",
            boundary.seconds());
        return;
    }

    const rclcpp::Time rgb_stamp = observation->stamp;

    const rclcpp::Time depth_stamp(
        observation->depth_to_rgb->header.stamp);

    const rclcpp::Time current_time = node_->now();

    RCLCPP_WARN(
        node_->get_logger(),
        "RGBD capture failure: "
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


void RGBDAcquisition::logDiagnostics()
{

   


    RCLCPP_DEBUG(
        node_->get_logger(),
        "RGBD: rgb=%zu depth=%zu "
        "synchronized=%zu resolved=%zu "
        "tf_failures=%zu expired=%zu "
        "camera_info=%s",
        rgb_received_,
        depth_received_,
        synchronized_count_,
        resolved_count_,
        tf_failure_count_,
        expired_count_,
        camera_info_ ? "yes" : "no");
}

}  // namespace hb_perception

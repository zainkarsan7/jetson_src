
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include "hb_robot_perception/rgbd_acquisition.hpp"
#include "hb_robot_perception/perception_types.hpp"

using namespace std::chrono_literals;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using Observation = hb_perception::Observation;

static Cloud::Ptr toXYZ(
    const hb_perception::PointCloud& input)
{
    auto output = std::make_shared<Cloud>();
    output->reserve(input.size());

    for (const auto& p : input.points)
    {
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
            continue;

        output->push_back(
            pcl::PointXYZ(p.x, p.y, p.z));
    }

    return output;
}

static Cloud::Ptr downsample(
    const Cloud::ConstPtr& input,
    float leaf)
{
    auto output = std::make_shared<Cloud>();

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);

    return output;
}


static Cloud::Ptr cropAngular(
    const Cloud::ConstPtr& input,
    const Eigen::Vector3f& origin,
    const Eigen::Vector2f& direction,
    double half_angle_deg,
    double min_range = 1.0,
    double max_range = 3.0)
{
    auto output = std::make_shared<Cloud>();
    output->reserve(input->size());

    constexpr double PI = 3.14159265358979323846;

    const float cos_limit = static_cast<float>(
        std::cos(half_angle_deg * PI / 180.0));

    const float max_range_sq =
        static_cast<float>(max_range * max_range);

    const float min_range_sq =
        static_cast<float>(min_range * min_range);
    const Eigen::Vector2f axis = direction.normalized();

    for (const auto& p : input->points)
    {
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
            continue;

        const Eigen::Vector3f v =
            Eigen::Vector3f(p.x, p.y, p.z) - origin;

        // Full 3D radial distance from the LiDAR.
        if (v.squaredNorm() > max_range_sq || v.squaredNorm()<min_range_sq)
            continue;

        // Horizontal bearing around world Z.
        const Eigen::Vector2f horizontal = v.head<2>();

        const float horizontal_norm = horizontal.norm();

        if (horizontal_norm < 1e-6f)
            continue;

        const float cosine =
            horizontal.dot(axis) / horizontal_norm;

        if (cosine >= cos_limit)
            output->push_back(p);
    }

    return output;
}


static Cloud::Ptr cropCloud(
    const Cloud::ConstPtr& input,
    const Eigen::Vector3f& minimum,
    const Eigen::Vector3f& maximum)
{
    auto output = std::make_shared<Cloud>();

    pcl::CropBox<pcl::PointXYZ> filter;
    filter.setInputCloud(input);

    filter.setMin(Eigen::Vector4f(
        minimum.x(),
        minimum.y(),
        minimum.z(),
        1.0f));

    filter.setMax(Eigen::Vector4f(
        maximum.x(),
        maximum.y(),
        maximum.z(),
        1.0f));

    filter.filter(*output);

    return output;
}

static void cloudBounds(
    const Cloud& cloud,
    Eigen::Vector3f& minimum,
    Eigen::Vector3f& maximum)
{
    minimum.setConstant(
        std::numeric_limits<float>::infinity());

    maximum.setConstant(
        -std::numeric_limits<float>::infinity());

    for (const auto& p : cloud.points)
    {
        Eigen::Vector3f v(p.x, p.y, p.z);

        minimum = minimum.cwiseMin(v);
        maximum = maximum.cwiseMax(v);
    }
}

class LidarExtrinsicCalibrator : public rclcpp::Node
{
public:
    LidarExtrinsicCalibrator()
        : Node("lidar_extrinsic_calibrator"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        base_frame_ = declare_parameter<std::string>(
            "base_frame", "HB_carriage");

        lidar_frame_ = declare_parameter<std::string>(
            "lidar_frame", "unilidar_lidar");

        const auto lidar_topic =
            declare_parameter<std::string>(
                "lidar_topic", "/hb_robot_lidar/cloud");

        const auto rgb_topic =
            declare_parameter<std::string>(
                "rgb_topic", "/k4a/rgb/image_raw");

        const auto depth_topic =
            declare_parameter<std::string>(
                "depth_topic",
                "/k4a/depth_to_rgb/image_raw");

        const auto camera_info_topic =
            declare_parameter<std::string>(
                "camera_info_topic",
                "/k4a/depth_to_rgb/camera_info");

        depth_range_ = declare_parameter<double>(
            "depth_range", 3.0);

        voxel_size_ = declare_parameter<double>(
            "voxel_size", 0.025);

        roi_padding_ = declare_parameter<double>(
            "roi_padding", 0.25);

        accumulation_seconds_ =
            declare_parameter<double>(
                "lidar_accumulation_seconds", 10.0);

        max_lidar_samples_ =
            declare_parameter<int>(
                "max_lidar_samples", 100);

        min_lidar_samples_ =
            declare_parameter<int>(
                "min_lidar_samples", 5);

        acquisition_timeout_ =
            declare_parameter<double>(
                "acquisition_timeout", 2.0);

        max_acquisition_attempts_ =
            declare_parameter<int>(
                "max_acquisition_attempts", 10);

        max_correspondence_ =
            declare_parameter<double>(
                "max_correspondence", 0.20);

        max_translation_correction_ =
            declare_parameter<double>(
                "max_translation_correction", 0.25);

        max_rotation_correction_deg_ =
            declare_parameter<double>(
                "max_rotation_correction_deg", 15.0);

        max_fitness_ =
            declare_parameter<double>(
                "max_fitness", 0.003);
        
        lidar_max_range_ =
            declare_parameter<double>("lidar_max_range", 3.0);
        lidar_min_range_ =
            declare_parameter<double>("lidar_min_range", 1.0);

        angular_axis_ = declare_parameter<std::string>(
            "angular_axis", "+y");

        angular_half_angle_deg_ = declare_parameter<double>(
            "angular_half_angle_deg", 35.0);

        angular_origin_x_ = declare_parameter<double>(
            "angular_origin_x", 0.0);

        angular_origin_y_ = declare_parameter<double>(
            "angular_origin_y", 0.0);

        acquisition_ =
            std::make_unique<
                hb_perception::RGBDAcquisition>(
                this,
                &tf_buffer_,
                base_frame_,
                rgb_topic,
                depth_topic,
                camera_info_topic);

        lidar_sub_ = create_subscription<CloudMsg>(
            lidar_topic,
            rclcpp::SensorDataQoS().keep_last(5),
            [this](CloudMsg::ConstSharedPtr msg)
            {
                lidarCallback(msg);
            });

        const auto qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        rgbd_pub_ =
            create_publisher<CloudMsg>(
                "~/rgbd_base", qos);

        lidar_before_pub_ =
            create_publisher<CloudMsg>(
                "~/lidar_before", qos);

        lidar_after_pub_ =
            create_publisher<CloudMsg>(
                "~/lidar_after", qos);

        calibration_service_ =
            create_service<std_srvs::srv::Trigger>(
                "~/calibrate",
                [this](
                    const std::shared_ptr<
                        std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<
                        std_srvs::srv::Trigger::Response> response)
                {
                    if (state_ != State::IDLE)
                    {
                        response->success = false;
                        response->message =
                            "Calibration already in progress";
                        return;
                    }

                    {
                        std::lock_guard<std::mutex> lock(
                            lidar_mutex_);

                        lidar_samples_.clear();
                    }

                    capture_start_ = now();
                    attempt_start_ = capture_start_;
                    acquisition_attempts_ = 1;

                    observation_.reset();

                    state_ = State::CAPTURING;

                    response->success = true;
                    response->message =
                        "Calibration capture started. "
                        "Watch node logs for result.";
                });

        timer_ = create_wall_timer(
            100ms,
            [this]() { tick(); });

        RCLCPP_INFO(
            get_logger(),
            "LiDAR extrinsic calibrator ready");

        RCLCPP_INFO(
            get_logger(),
            "Base: %s | LiDAR: %s",
            base_frame_.c_str(),
            lidar_frame_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "LiDAR accumulation: %.2f s",
            accumulation_seconds_);
    }

private:
    enum class State
    {
        IDLE,
        CAPTURING,
        PROCESSING
    };

    void lidarCallback(
        const CloudMsg::ConstSharedPtr& msg)
    {
        if (state_ != State::CAPTURING)
            return;

        if (msg->header.frame_id != lidar_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring LiDAR cloud in frame '%s'; "
                "expected '%s'",
                msg->header.frame_id.c_str(),
                lidar_frame_.c_str());

            return;
        }

        std::lock_guard<std::mutex> lock(
            lidar_mutex_);

        // if (lidar_samples_.size() >=
        //     static_cast<std::size_t>(
        //         max_lidar_samples_))
        // {
        //     return;
        // }

        lidar_samples_.push_back(msg);
    }

    void tick()
    {
        if (state_ != State::CAPTURING)
            return;

        const auto current_time = now();

        // ---------------------------------------------
        // Acquire Kinect observation with retries.
        // ---------------------------------------------

        if (!observation_)
        {
            const auto latest = acquisition_->latest();

            if (latest &&
                latest->stamp > capture_start_)
            {
                observation_ = latest;

                RCLCPP_INFO(
                    get_logger(),
                    "RGBD acquired at %.3f",
                    latest->stamp.seconds());
            }
            else
            {
                const double elapsed =
                    (current_time - attempt_start_)
                        .seconds();

                if (elapsed >= acquisition_timeout_)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "RGBD attempt %d/%d failed",
                        acquisition_attempts_,
                        max_acquisition_attempts_);

                    if (acquisition_attempts_ >=
                        max_acquisition_attempts_)
                    {
                        RCLCPP_ERROR(
                            get_logger(),
                            "RGBD acquisition failed "
                            "after %d attempts",
                            max_acquisition_attempts_);

                        state_ = State::IDLE;
                        return;
                    }

                    ++acquisition_attempts_;
                    attempt_start_ = current_time;
                }
            }
        }

        // ---------------------------------------------
        // Wait for LiDAR accumulation window.
        // ---------------------------------------------

        const double capture_elapsed =
            (current_time - capture_start_).seconds();

        if (capture_elapsed < accumulation_seconds_)
            return;

        // Kinect can take longer than LiDAR.
        if (!observation_)
            return;

        std::vector<CloudMsg::ConstSharedPtr> samples;

        {
            std::lock_guard<std::mutex> lock(
                lidar_mutex_);

            samples = lidar_samples_;
        }

        if (samples.size() <
            static_cast<std::size_t>(
                min_lidar_samples_))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Insufficient LiDAR messages: %zu",
                samples.size());

            state_ = State::IDLE;
            return;
        }

        state_ = State::PROCESSING;

        RCLCPP_INFO(
            get_logger(),
            "Captured %zu LiDAR messages "
            "and one RGBD observation",
            samples.size());

        try
        {
            calibrate(*observation_, samples);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Calibration exception: %s",
                e.what());
        }

        observation_.reset();
        state_ = State::IDLE;
    }

    void publishCloud(
        const Cloud& cloud,
        const rclcpp::Publisher<CloudMsg>::SharedPtr& pub)
    {
        CloudMsg msg;

        pcl::toROSMsg(cloud, msg);

        msg.header.frame_id = base_frame_;
        msg.header.stamp = now();

        pub->publish(msg);
    }

    void calibrate(
        const Observation& observation,
        const std::vector<
            CloudMsg::ConstSharedPtr>& lidar_samples)
    {
        // =============================================
        // 1. Kinect depth cloud in robot base
        // =============================================

        const Eigen::Matrix4f T_BC =
            tf2::transformToEigen(
                observation.camera_pose)
                .matrix()
                .cast<float>();

        auto rgbd_raw =
            hb_perception::depthToCloud(
                observation.depth_to_rgb,
                observation.rgb_camera_info,
                depth_range_,
                3);

        if (!rgbd_raw || rgbd_raw->empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "RGBD depth cloud is empty");
            return;
        }

        Cloud rgbd_base;

        pcl::transformPointCloud(
            *toXYZ(*rgbd_raw),
            rgbd_base,
            T_BC);

        auto target = downsample(
            std::make_shared<Cloud>(rgbd_base),
            static_cast<float>(voxel_size_));

        if (target->size() < 100)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Insufficient RGBD target points: %zu",
                target->size());
            return;
        }

        // =============================================
        // 2. Find approximate common region
        // =============================================

        Eigen::Vector2f angular_direction;

        if (angular_axis_ == "+x")
            angular_direction = Eigen::Vector2f::UnitX();
        else if (angular_axis_ == "-x")
            angular_direction = -Eigen::Vector2f::UnitX();
        else if (angular_axis_ == "+y")
            angular_direction = Eigen::Vector2f::UnitY();
        else if (angular_axis_ == "-y")
            angular_direction = -Eigen::Vector2f::UnitY();
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid angular_axis '%s'. "
                "Use +x, -x, +y, or -y.",
                angular_axis_.c_str());
            return;
        }

        if (angular_half_angle_deg_ <= 0.0 ||
            angular_half_angle_deg_ > 180.0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "angular_half_angle_deg must be in (0, 180]");
            return;
        }

const Eigen::Vector2f angular_origin(
    static_cast<float>(angular_origin_x_),
    static_cast<float>(angular_origin_y_));

        // =============================================
        // 3. Current base -> LiDAR transform
        // =============================================

        // This is the current physical mounting estimate.
        // It must not contain IMU attitude drift.
        const auto tf_msg =
            tf_buffer_.lookupTransform(
                base_frame_,
                lidar_frame_,
                tf2::TimePointZero);

        const Eigen::Matrix4f T_BL =
            tf2::transformToEigen(tf_msg)
                .matrix()
                .cast<float>();

        // =============================================
        // 4. Accumulate LiDAR scans
        // =============================================

        Cloud accumulated;

        std::size_t raw_points = 0;
        std::size_t accepted_scans = 0;

        for (const auto& msg : lidar_samples)
        {
            if (msg->header.frame_id != lidar_frame_)
                continue;

            Cloud raw;
            pcl::fromROSMsg(*msg, raw);

            Cloud clean;
            std::vector<int> indices;

            pcl::removeNaNFromPointCloud(
                raw,
                clean,
                indices);

            if (clean.empty())
                continue;

            Cloud transformed;

            pcl::transformPointCloud(
                clean,
                transformed,
                T_BL);

            accumulated += transformed;

            raw_points += clean.size();
            ++accepted_scans;
        }

        if (accumulated.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Accumulated LiDAR cloud is empty");
            return;
        }

        // Downsample once after accumulation.
        auto lidar_downsampled = downsample(
            std::make_shared<Cloud>(accumulated),
            static_cast<float>(voxel_size_));

        const Eigen::Vector3f lidar_origin =
            T_BL.block<3, 1>(0, 3);

        auto source = cropAngular(
            lidar_downsampled,
            lidar_origin,
            angular_direction,
            angular_half_angle_deg_,
            lidar_min_range_,
            lidar_max_range_);

        RCLCPP_INFO(
            get_logger(),
            "Angular crop: axis=%s half-angle=%.1f deg "
            "origin=(%.3f, %.3f) "
            "points=%zu/%zu",
            angular_axis_.c_str(),
            angular_half_angle_deg_,
            angular_origin_x_,
            angular_origin_y_,
            source->size(),
            lidar_downsampled->size());

        RCLCPP_INFO(
            get_logger(),
            "LiDAR scans=%zu raw_points=%zu "
            "voxelized=%zu overlap=%zu "
            "RGBD=%zu",
            accepted_scans,
            raw_points,
            lidar_downsampled->size(),
            source->size(),
            target->size());

        if (source->size() < 60)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Insufficient overlapping LiDAR points");
            return;
        }

        // =============================================
        // 5. Generalized ICP
        // =============================================

        pcl::GeneralizedIterativeClosestPoint<
            pcl::PointXYZ,
            pcl::PointXYZ> gicp;

        gicp.setInputSource(source);
        gicp.setInputTarget(target);

        gicp.setMaxCorrespondenceDistance(
            max_correspondence_);

        gicp.setMaximumIterations(45);
        gicp.setTransformationEpsilon(1e-7);

        Cloud aligned;

        // Both source and target are already expressed
        // in base_frame_, so estimate only the correction.
        gicp.align(aligned);

        if (!gicp.hasConverged())
        {
            RCLCPP_ERROR(
                get_logger(),
                "GICP failed to converge");

            publishCloud(*target, rgbd_pub_);
            publishCloud(*source, lidar_before_pub_);
            return;
        }

        // =============================================
        // 6. Correct mounting transform
        // =============================================

        const Eigen::Matrix4f delta =
            gicp.getFinalTransformation();

        // Correction is expressed in base coordinates.
        const Eigen::Matrix4f T_BL_new =
            delta * T_BL;

        const Eigen::Vector3f correction_translation =
            delta.block<3, 1>(0, 3);

        const Eigen::AngleAxisf correction_rotation(
            delta.block<3, 3>(0, 0));

        constexpr double RAD_TO_DEG =
            180.0 / 3.14159265358979323846;

        const double correction_angle_deg =
            std::abs(correction_rotation.angle()) *
            RAD_TO_DEG;

        const double correction_distance =
            correction_translation.norm();

        const double fitness =
            gicp.getFitnessScore(max_correspondence_);

        // =============================================
        // 7. RViz diagnostics
        // =============================================

        publishCloud(*target, rgbd_pub_);
        publishCloud(*source, lidar_before_pub_);
        publishCloud(aligned, lidar_after_pub_);

        RCLCPP_INFO(
            get_logger(),
            "GICP converged: "
            "fitness=%.7f m^2 "
            "translation=%.4f m "
            "rotation=%.3f deg",
            fitness,
            correction_distance,
            correction_angle_deg);

        // =============================================
        // 8. Sanity checks
        // =============================================

        if (!std::isfinite(fitness) ||
            fitness > max_fitness_ ||
            correction_distance >
                max_translation_correction_ ||
            correction_angle_deg >
                max_rotation_correction_deg_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Calibration REJECTED by thresholds. "
                "Inspect RViz before changing TF.");
            return;
        }

        // =============================================
        // 9. Report candidate extrinsic
        // =============================================

        const Eigen::Matrix3f R =
            T_BL_new.block<3, 3>(0, 0);

        Eigen::Quaternionf q(R);
        q.normalize();

        const Eigen::Vector3f p =
            T_BL_new.block<3, 1>(0, 3);

        const Eigen::Vector3f rpy =
            R.eulerAngles(2, 1, 0).reverse();

        RCLCPP_INFO(
            get_logger(),
            "\n"
            "====================================\n"
            "CANDIDATE LIDAR EXTRINSIC\n"
            "====================================\n"
            "Parent: %s\n"
            "Child:  %s\n"
            "xyz:  %.6f %.6f %.6f\n"
            "rpy:  %.6f %.6f %.6f\n"
            "xyzw: %.7f %.7f %.7f %.7f\n"
            "====================================",
            base_frame_.c_str(),
            lidar_frame_.c_str(),
            p.x(), p.y(), p.z(),
            rpy.x(), rpy.y(), rpy.z(),
            q.x(), q.y(), q.z(), q.w());

        RCLCPP_WARN(
            get_logger(),
            "Candidate only. Do not update Xacro "
            "until repeated captures agree.");
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<hb_perception::RGBDAcquisition>
        acquisition_;

    rclcpp::Subscription<CloudMsg>::SharedPtr
        lidar_sub_;

    rclcpp::Publisher<CloudMsg>::SharedPtr
        rgbd_pub_;

    rclcpp::Publisher<CloudMsg>::SharedPtr
        lidar_before_pub_;

    rclcpp::Publisher<CloudMsg>::SharedPtr
        lidar_after_pub_;

    rclcpp::Service<
        std_srvs::srv::Trigger>::SharedPtr
        calibration_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex lidar_mutex_;

    std::vector<CloudMsg::ConstSharedPtr>
        lidar_samples_;

    std::optional<Observation> observation_;

    State state_ = State::IDLE;

    rclcpp::Time capture_start_{
        0, 0, RCL_ROS_TIME};

    rclcpp::Time attempt_start_{
        0, 0, RCL_ROS_TIME};

    int acquisition_attempts_ = 0;

    std::string base_frame_;
    std::string lidar_frame_;

    double depth_range_ = 3.0;
    double voxel_size_ = 0.025;
    double roi_padding_ = 0.25;

    double accumulation_seconds_ = 3.0;

    int max_lidar_samples_ = 100;
    int min_lidar_samples_ = 5;

    double acquisition_timeout_ = 2.0;
    int max_acquisition_attempts_ = 10;
    double lidar_min_range_ = 1.0;
    double lidar_max_range_ = 3.0;
    double max_correspondence_ = 0.20;
    double max_translation_correction_ = 2.25;
    double max_rotation_correction_deg_ = 15.0;
    double max_fitness_ = 0.003;
    std::string angular_axis_ = "+y";
    double angular_half_angle_deg_ = 35.0;
    double angular_origin_x_ = 0.0;
    double angular_origin_y_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<LidarExtrinsicCalibrator>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

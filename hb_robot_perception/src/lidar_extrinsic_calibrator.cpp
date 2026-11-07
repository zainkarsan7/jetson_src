
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
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

#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <hb_robot_perception/perception_types.hpp>

using namespace std::chrono_literals;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// Convert the existing perception package's XYZRGB cloud
// to a geometry-only XYZ cloud.
static Cloud::Ptr xyz(const hb_perception::PointCloud& in)
{
    auto out = std::make_shared<Cloud>();
    out->reserve(in.size());

    for (const auto& p : in)
    {
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            continue;
        }

        out->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    return out;
}

static Cloud::Ptr voxel(
    const Cloud::ConstPtr& in,
    float leaf)
{
    auto out = std::make_shared<Cloud>();

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*out);

    return out;
}

static Cloud::Ptr crop(
    const Cloud::ConstPtr& in,
    const Eigen::Vector3f& lo,
    const Eigen::Vector3f& hi)
{
    auto out = std::make_shared<Cloud>();

    pcl::CropBox<pcl::PointXYZ> filter;
    filter.setInputCloud(in);

    filter.setMin(
        Eigen::Vector4f(lo.x(), lo.y(), lo.z(), 1.0f));

    filter.setMax(
        Eigen::Vector4f(hi.x(), hi.y(), hi.z(), 1.0f));

    filter.filter(*out);

    return out;
}

static void bounds(
    const Cloud& cloud,
    Eigen::Vector3f& lo,
    Eigen::Vector3f& hi)
{
    lo.setConstant(
        std::numeric_limits<float>::infinity());

    hi.setConstant(
        -std::numeric_limits<float>::infinity());

    for (const auto& p : cloud)
    {
        const Eigen::Vector3f v(p.x, p.y, p.z);

        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }
}

class Calibrator : public rclcpp::Node
{
public:
    Calibrator()
        : Node("lidar_extrinsic_calibrator"),
          tf_(get_clock()),
          listener_(tf_)
    {
        base_ = declare_parameter<std::string>(
            "base_frame", "world");

        lidar_ = declare_parameter<std::string>(
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

        depth_range_ =
            declare_parameter<double>("depth_range", 3.0);

        leaf_ =
            declare_parameter<double>("voxel_size", 0.025);

        roi_pad_ =
            declare_parameter<double>("roi_padding", 0.25);

        max_pair_dt_ =
            declare_parameter<double>("max_pair_dt", 0.15);

        max_corr_ =
            declare_parameter<double>(
                "max_correspondence", 0.20);

        max_translation_ =
            declare_parameter<double>(
                "max_translation_correction", 0.25);

        max_rotation_deg_ =
            declare_parameter<double>(
                "max_rotation_correction_deg", 15.0);

        max_fitness_ =
            declare_parameter<double>(
                "max_fitness", 0.003);

        max_acquisition_attempts_ =
                declare_parameter<int>(
                    "max_acquisition_attempts", 10);

        acquisition_timeout_ =
            declare_parameter<double>(
                "acquisition_timeout", 2.0);

        // Reuse the existing synchronized RGBD pipeline.
        // Its Observation includes a timestamped camera pose
        // expressed in base_.
        acquisition_ =
            std::make_unique<hb_perception::RGBDAcquisition>(
                this,
                &tf_,
                base_,
                rgb_topic,
                depth_topic,
                camera_info_topic);

        sub_ =
            create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic,
                rclcpp::SensorDataQoS().keep_last(5),
                [this](
                    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(mu_);

                    lidar_samples_.push_back(msg);

                    while (lidar_samples_.size() > 40)
                    {
                        lidar_samples_.pop_front();
                    }
                });

        const auto qos =
            rclcpp::QoS(1).reliable().transient_local();

        rgb_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "~/rgbd_base", qos);

        before_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "~/lidar_before", qos);

        after_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "~/lidar_after", qos);

        service_ =
            create_service<std_srvs::srv::Trigger>(
                "~/calibrate",
                [this](
                    const std::shared_ptr<
                        std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<
                        std_srvs::srv::Trigger::Response> response)
                {
                    if (busy_)
                    {
                        response->success = false;
                        response->message =
                            "Calibration already running";
                        return;
                    }

                    busy_ = true;
                    request_stamp_ = now();
                    attempt_started_ = request_stamp_;
                    acquisition_attempts_ = 1;

                    response->success = true;
                    response->message =
                        "Capture requested. "
                        "Watch node logs for result; "
                        "no TF is changed.";
                });

        // Processing occurs outside the service callback.
        // The executor can continue receiving sensor data
        // while waiting for a suitable pair.
        timer_ = create_wall_timer(
            100ms,
            [this]() { tick(); });

        RCLCPP_INFO(
            get_logger(),
            "Ready. Call ~/calibrate with "
            "std_srvs/srv/Trigger");
    }

private:
    void publish(
        const Cloud& cloud,
        const rclcpp::Publisher<
            sensor_msgs::msg::PointCloud2>::SharedPtr& pub)
    {
        sensor_msgs::msg::PointCloud2 msg;

        pcl::toROSMsg(cloud, msg);

        msg.header.frame_id = base_;
        msg.header.stamp = now();

        pub->publish(msg);
    }

    void tick()
    {
       
    if (!busy_)
        return;

    const auto ob = acquisition_->latest();

    if (!ob || ob->stamp <= request_stamp_)
    {

        RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for RGBD: now=%.3f request=%.3f "
                "latest=%.3f",
                now().seconds(),
                request_stamp_.seconds(),
                ob ? ob->stamp.seconds() : -1.0);
        const double elapsed =
            (now() - attempt_started_).seconds();

        if (elapsed >= acquisition_timeout_)
        {
            RCLCPP_WARN(
                get_logger(),
                "RGBD acquisition attempt %d/%d failed",
                acquisition_attempts_,
                max_acquisition_attempts_);

            if (acquisition_attempts_ >=
                max_acquisition_attempts_)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "RGBD acquisition failed after %d attempts",
                    max_acquisition_attempts_);

                busy_ = false;
                return;
            }

            ++acquisition_attempts_;
            attempt_started_ = now();
        }

        return;
    }

    // Existing LiDAR sample selection and
    // calibration logic continues here.

        sensor_msgs::msg::PointCloud2::ConstSharedPtr
            lidar_msg;

        double best_dt = max_pair_dt_;

        {
            std::lock_guard<std::mutex> lock(mu_);

            for (const auto& msg : lidar_samples_)
            {
                const double dt = std::abs(
                    (rclcpp::Time(msg->header.stamp) -
                     ob->stamp).seconds());

                if (dt < best_dt)
                {
                    best_dt = dt;
                    lidar_msg = msg;
                }
            }
        }

        if (!lidar_msg)
        {
            if ((now() - request_stamp_).seconds() > 8.0)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "No LiDAR sample within %.3f s "
                    "of RGBD capture",
                    max_pair_dt_);

                busy_ = false;
            }

            return;
        }

        busy_ = false;

        try
        {
            calibrate(*ob, *lidar_msg, best_dt);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Calibration failed: %s",
                e.what());
        }
    }

    void calibrate(
        const hb_perception::Observation& ob,
        const sensor_msgs::msg::PointCloud2& lidar_msg,
        double pair_dt)
    {
        // The cloud must be expressed in the physical
        // LiDAR frame, not the IMU initial/world frame.
        if (lidar_msg.header.frame_id != lidar_)
        {
            RCLCPP_ERROR(
                get_logger(),
                "LiDAR message frame '%s' "
                "!= configured frame '%s'",
                lidar_msg.header.frame_id.c_str(),
                lidar_.c_str());

            return;
        }

        // ------------------------------------------------
        // 1. Kinect depth -> cloud -> robot base
        // ------------------------------------------------

        const Eigen::Matrix4f T_BC =
            tf2::transformToEigen(ob.camera_pose)
                .matrix()
                .cast<float>();

        auto cam_raw =
            hb_perception::depthToCloud(
                ob.depth_to_rgb,
                ob.rgb_camera_info,
                depth_range_,
                3);

        if (!cam_raw || cam_raw->empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No valid RGBD points");

            return;
        }

        Cloud cam_base;

        pcl::transformPointCloud(
            *xyz(*cam_raw),
            cam_base,
            T_BC);

        auto target = voxel(
            std::make_shared<Cloud>(cam_base),
            static_cast<float>(leaf_));

        if (target->size() < 100)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Insufficient RGBD points");

            return;
        }

        // ------------------------------------------------
        // 2. Approximate overlap ROI in base coordinates
        // ------------------------------------------------

        Eigen::Vector3f lo;
        Eigen::Vector3f hi;

        bounds(*target, lo, hi);

        lo.array() -= static_cast<float>(roi_pad_);
        hi.array() += static_cast<float>(roi_pad_);

        // ------------------------------------------------
        // 3. LiDAR cloud -> robot base using current TF
        // ------------------------------------------------

        Cloud lidar_raw;

        pcl::fromROSMsg(lidar_msg, lidar_raw);

        std::vector<int> kept;

        auto lidar_clean = std::make_shared<Cloud>();

        pcl::removeNaNFromPointCloud(
            lidar_raw,
            *lidar_clean,
            kept);

        const auto tf_msg = tf_.lookupTransform(
            base_,
            lidar_,
            lidar_msg.header.stamp,
            rclcpp::Duration::from_seconds(0.2));

        const Eigen::Matrix4f T_BL =
            tf2::transformToEigen(tf_msg)
                .matrix()
                .cast<float>();

        Cloud lidar_base;

        pcl::transformPointCloud(
            *lidar_clean,
            lidar_base,
            T_BL);

        auto source = crop(
            voxel(
                std::make_shared<Cloud>(lidar_base),
                static_cast<float>(leaf_)),
            lo,
            hi);

        if (source->size() < 60)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Only %zu LiDAR points overlap RGBD ROI",
                source->size());

            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Registering %zu LiDAR points "
            "to %zu RGBD points (dt=%.3f s)",
            source->size(),
            target->size(),
            pair_dt);

        // ------------------------------------------------
        // 4. Register LiDAR source to Kinect target
        // ------------------------------------------------

        pcl::GeneralizedIterativeClosestPoint<
            pcl::PointXYZ,
            pcl::PointXYZ> reg;

        reg.setInputSource(source);
        reg.setInputTarget(target);

        reg.setMaxCorrespondenceDistance(max_corr_);
        reg.setMaximumIterations(45);
        reg.setTransformationEpsilon(1e-7);

        Cloud aligned;

        // Both clouds are already in base coordinates.
        // Thus the initial registration correction is I.
        reg.align(aligned);

        if (!reg.hasConverged())
        {
            RCLCPP_ERROR(
                get_logger(),
                "GICP did not converge");

            return;
        }

        // ------------------------------------------------
        // 5. Recover corrected base -> LiDAR transform
        // ------------------------------------------------

        const Eigen::Matrix4f delta =
            reg.getFinalTransformation();

        // delta is expressed in base coordinates.
        // Therefore left-multiply the existing extrinsic.
        const Eigen::Matrix4f T_new = delta * T_BL;

        const Eigen::AngleAxisf aa(
            delta.block<3, 3>(0, 0));

        constexpr double RAD_TO_DEG =
            180.0 / 3.14159265358979323846;

        const double rot_deg =
            std::abs(aa.angle()) * RAD_TO_DEG;

        const double translation =
            delta.block<3, 1>(0, 3).norm();

        const double fitness =
            reg.getFitnessScore(max_corr_);

        // ------------------------------------------------
        // 6. Publish diagnostic clouds for RViz
        // ------------------------------------------------

        publish(*target, rgb_pub_);
        publish(*source, before_pub_);
        publish(aligned, after_pub_);

        RCLCPP_INFO(
            get_logger(),
            "GICP fitness %.6f m^2, "
            "correction %.3f m / %.2f deg",
            fitness,
            translation,
            rot_deg);

        if (!std::isfinite(fitness) ||
            fitness > max_fitness_ ||
            translation > max_translation_ ||
            rot_deg > max_rotation_deg_)
        {
            RCLCPP_WARN(
                get_logger(),
                "REJECTED: fitness/correction exceeded "
                "configured limits. Inspect RViz.");

            return;
        }

        // ------------------------------------------------
        // 7. Report candidate Xacro extrinsic
        // ------------------------------------------------

        Eigen::Quaternionf q(
            T_new.block<3, 3>(0, 0));

        q.normalize();

        const Eigen::Vector3f p =
            T_new.block<3, 1>(0, 3);

        RCLCPP_INFO(
            get_logger(),
            "CANDIDATE %s -> %s:\n"
            "  xyz  [%.6f %.6f %.6f]\n"
            "  xyzw [%.7f %.7f %.7f %.7f]",
            base_.c_str(),
            lidar_.c_str(),
            p.x(), p.y(), p.z(),
            q.x(), q.y(), q.z(), q.w());

        RCLCPP_WARN(
            get_logger(),
            "Do not apply automatically. "
            "Validate with independent captures "
            "and check for planar degeneracy.");
    }

    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener listener_;

    std::unique_ptr<hb_perception::RGBDAcquisition>
        acquisition_;

    rclcpp::Subscription<
        sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
            rgb_pub_, before_pub_, after_pub_;

    rclcpp::Service<
        std_srvs::srv::Trigger>::SharedPtr service_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mu_;

    std::deque<
        sensor_msgs::msg::PointCloud2::ConstSharedPtr>
            lidar_samples_;

    bool busy_ = false;

    rclcpp::Time request_stamp_{0, 0, RCL_ROS_TIME};

    std::string base_;
    std::string lidar_;

    double depth_range_;
    double leaf_;
    double roi_pad_;
    double max_pair_dt_;
    double max_corr_;
    double max_translation_;
    double max_rotation_deg_;
    double max_fitness_;
    int max_acquisition_attempts_ = 10;
    double acquisition_timeout_ = 2.0;

    int acquisition_attempts_ = 0;
    rclcpp::Time attempt_started_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<Calibrator>());

    rclcpp::shutdown();

    return 0;
}

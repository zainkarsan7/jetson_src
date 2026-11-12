
#include "hb_robot_skills/inspect_profile_server.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <moveit/robot_state/conversions.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "hb_robot_skills/motion/transit_planner.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hb_robot_skills {

using namespace std::chrono_literals;

namespace {

using CollisionObject =
    moveit_msgs::msg::CollisionObject;

using Cloud =
    pcl::PointCloud<pcl::PointXYZRGB>;

bool finiteRigid(const Eigen::Isometry3d& pose)
{
    if (!pose.matrix().allFinite())
        return false;

    const auto& r = pose.linear();

    return std::abs(r.determinant() - 1.0) < 1e-4 &&
           (r.transpose() * r).isApprox(
               Eigen::Matrix3d::Identity(), 1e-4);
}

bool matchingPose(
    const Eigen::Isometry3d& a,
    const Eigen::Isometry3d& b)
{
    return finiteRigid(a) &&
           finiteRigid(b) &&
           (a.translation() -
            b.translation()).norm() <= 1e-4 &&
           (a.linear() - b.linear()).norm() <= 1e-4;
}

}  // namespace

InspectProfileServer::InspectProfileServer(
    const rclcpp::NodeOptions& options)
    : Node("inspect_profile_server", options),
      scene_model_(
          std::make_shared<hb_perception::SceneModel>())
{
    planning_group_ =
        declare_parameter<std::string>(
            "planning_group", "manipulator");

    camera_link_ =
        declare_parameter<std::string>(
            "camera_link", "camera_visor");

    planning_time_ =
        declare_parameter<double>(
            "planning_time", 5.0);

    stability_timeout_ =
        declare_parameter<double>(
            "stability_timeout", 5.0);

    skip_motion_ =
        declare_parameter<bool>(
            "skip_inspection_motion", true);

    require_plan_approval_ =
        declare_parameter<bool>(
            "require_plan_approval", true);

    rgb_topic_ =
        declare_parameter<std::string>(
            "perception_rgb_topic",
            "/k4a/rgb/image_raw");

    depth_topic_ =
        declare_parameter<std::string>(
            "perception_depth_topic",
            "/k4a/depth_to_rgb/image_raw");

    rgbd_topic_ =
    declare_parameter<std::string>(
        "perception_rgbd_topic",
        "/k4a/rgbd");

    camera_info_topic_ =
        declare_parameter<std::string>(
            "perception_camera_info_topic",
            "/k4a/depth_to_rgb/camera_info");

    scene_frame_ =
        declare_parameter<std::string>(
            "scene_base_frame", "world");

    acquisition_timeout_ =
        declare_parameter<double>(
            "acquisition_timeout", 5.0);

    commit_timeout_ =
        declare_parameter<double>(
            "workpiece_commit_timeout", 10.0);

    rms_target_ =
        declare_parameter<double>(
            "rms_target", 0.01);

    inlier_fraction_target_ =
        declare_parameter<double>(
            "inlier_fraction_target", 0.4);

    coverage_target_ =
        declare_parameter<double>(
            "coverage_target", 0.4);

    exploration_weight_ =
        declare_parameter<double>(
            "exploration_weight_", 100.0);

    motion_weight_ =
        declare_parameter<double>(
            "motion_weight_", 0.001);

    inspection_pos_tol_ =
        declare_parameter<double>(
            "inspection_pos_tol", 0.03);
    inspection_ang_tol_ =
        declare_parameter<double>(
            "inspection_anf_tol_", 0.1);


    if (planning_time_ <= 0.0 ||
        stability_timeout_ <= 0.0 ||
        acquisition_timeout_ <= 0.0 ||
        commit_timeout_ <= 0.0)
    {
        throw std::invalid_argument(
            "Inspection timeouts must be positive");
    }
}

InspectProfileServer::~InspectProfileServer()
{
    if (move_group_)
        move_group_->stop();

    approval_cv_.notify_all();

    if (worker_.joinable())
        worker_.join();
}

void InspectProfileServer::initialize()
{
    move_group_ =
        std::make_unique<
            moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),
            planning_group_);

    move_group_->setPlanningTime(planning_time_);

    nbv_planner_ =
        std::make_unique<motion::NBVPlanner>(
            move_group_->getRobotModel(),
            planning_group_,
            camera_link_);

    planning_scene_monitor_ =
        std::make_shared<
            planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(),
            "robot_description");

    if (!planning_scene_monitor_->getPlanningScene())
        throw std::runtime_error(
            "Planning scene monitor unavailable");

    planning_scene_monitor_->startSceneMonitor(
        "/monitored_planning_scene");

    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(
            get_clock());

    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(
            *tf_buffer_);

    rgbd_acquisition_ =
    std::make_unique<
        hb_perception::CroppedRGBDAcquisition>(
        this,
        tf_buffer_.get(),
        scene_frame_,
        hb_perception::CroppedRGBDAcquisition::
            Mode::CaptureWindow,
        "/k4a/rgbd");



        // std::make_unique<
        //     hb_perception::RGBDAcquisition>(
        //     this,
        //     tf_buffer_.get(),
        //     scene_frame_,
        //     hb_perception::RGBDAcquisition::
        //         Mode::CaptureWindow,
        //     rgb_topic_,
        //     depth_topic_,
        //     camera_info_topic_);

    platform_supervision_ =
        std::make_unique<PlatformSupervisionClient>(
            *this);

    commit_client_ =
        create_client<CommitWorkpiece>(
            "/task_state/commit_workpiece");

    const auto qos =
        rclcpp::QoS(1).reliable().transient_local();

    display_traj_pub_ =
        create_publisher<
            moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10);

    scene_cloud_pub_ =
        create_publisher<
            sensor_msgs::msg::PointCloud2>(
            "/profile_inspection/scene_cloud", qos);

    profile_marker_pub_ =
        create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "/profile_inspection/matches", qos);

    coverage_marker_pub_ =
        create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "/profile_inspection/coverage", qos);

    camera_pose_pub_ =
        create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "/profile_inspection/camera_poses", qos);

    // Provisional diagnostic estimates only.
    profile_estimate_pub_ =
        create_publisher<
            hb_robot_interfaces::msg::ProfileEstimate>(
            "/profile_inspection/estimate", qos);

    rms_pub_ =
        create_publisher<std_msgs::msg::Float64>(
            "/profile_inspection/metrics/rms_mm", 10);

    inlier_pub_ =
        create_publisher<std_msgs::msg::Float64>(
            "/profile_inspection/metrics/inlier_fraction",
            10);

    coverage_pub_ =
        create_publisher<std_msgs::msg::Float64>(
            "/profile_inspection/metrics/coverage",
            10);

    action_server_ =
        rclcpp_action::create_server<InspectProfile>(
            shared_from_this(),
            "inspect_profile",
            std::bind(
                &InspectProfileServer::handleGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &InspectProfileServer::handleCancel,
                this,
                std::placeholders::_1),
            std::bind(
                &InspectProfileServer::handleAccepted,
                this,
                std::placeholders::_1));

    approval_service_ =
        create_service<
            hb_robot_interfaces::srv::ApproveMotion>(
            "approve_profile_motion",
            std::bind(
                &InspectProfileServer::handleApproval,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    

    RCLCPP_INFO(
        get_logger(),
        "InspectProfile ready; commit service: "
        "/task_state/commit_workpiece");
}

rclcpp_action::GoalResponse
InspectProfileServer::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const InspectProfile::Goal> goal)
{
    if (goal->max_views == 0 ||
        goal->samples_per_view == 0 ||
        !std::isfinite(goal->depth_range) ||
        !std::isfinite(goal->camera_standoff) ||
        goal->depth_range <= 0.0 ||
        goal->camera_standoff <= 0.0)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;

    if (!busy_.compare_exchange_strong(
            expected, true))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::
        ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
InspectProfileServer::handleCancel(
    const std::shared_ptr<GoalHandle>)
{
    if (move_group_)
        move_group_->stop();

    approval_cv_.notify_all();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void InspectProfileServer::handleAccepted(
    const std::shared_ptr<GoalHandle> handle)
{
    if (worker_.joinable())
        worker_.join();

    worker_ = std::thread(
        [this, handle]()
        {
            try
            {
                execute(handle);
            }
            catch (const std::exception& e)
            {
                auto result =
                    std::make_shared<
                        InspectProfile::Result>();

                result->success = false;
                result->result_code =
                    InspectProfile::Result::
                        PLANNING_FAILED;

                result->message =
                    std::string("Inspection exception: ") +
                    e.what();

                if (handle->is_canceling())
                    handle->canceled(result);
                else
                    handle->abort(result);
            }

            busy_.store(false);
        });
}

void InspectProfileServer::handleApproval(
    const std::shared_ptr<
        hb_robot_interfaces::srv::ApproveMotion::Request>
        request,
    std::shared_ptr<
        hb_robot_interfaces::srv::ApproveMotion::Response>
        response)
{
    {
        std::lock_guard<std::mutex> lock(
            approval_mutex_);

        motion_approved_ = request->approve;
        motion_rejected_ = !request->approve;
    }

    response->accepted = true;

    response->message =
        request->approve
            ? "Motion approved"
            : "Motion rejected";

    approval_cv_.notify_all();
}

InspectProfileServer::MotionOutcome
InspectProfileServer::executeApprovedPlan(
    const moveit::core::RobotState& start,
    const moveit::planning_interface::
        MoveGroupInterface::Plan& plan,
    const std::shared_ptr<GoalHandle>& handle,
    const std::string& description)
{
    if (handle->is_canceling())
        return MotionOutcome::CANCELED;

    moveit_msgs::msg::DisplayTrajectory display;

    display.model_id =
        move_group_->getRobotModel()->getName();

    moveit::core::robotStateToRobotStateMsg(
        start,
        display.trajectory_start);

    display.trajectory.push_back(
        plan.trajectory_);

    display_traj_pub_->publish(display);

    if (skip_motion_)
    {
        RCLCPP_INFO(
            get_logger(),
            "Preview only: %s",
            description.c_str());

        return MotionOutcome::REJECTED;
    }

    if (require_plan_approval_)
    {
        {
            std::lock_guard<std::mutex> lock(
                approval_mutex_);

            motion_approved_ = false;
            motion_rejected_ = false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Awaiting inspection motion approval: %s",
            description.c_str());

        std::unique_lock<std::mutex> lock(
            approval_mutex_);

        approval_cv_.wait(
            lock,
            [&]()
            {
                return motion_approved_ ||
                       motion_rejected_ ||
                       handle->is_canceling() ||
                       !rclcpp::ok();
            });

        if (handle->is_canceling())
            return MotionOutcome::CANCELED;

        if (!rclcpp::ok())
            return MotionOutcome::FAILED;

        if (motion_rejected_)
            return MotionOutcome::REJECTED;
    }

    if (handle->is_canceling())
        return MotionOutcome::CANCELED;

    const auto outcome =
        move_group_->execute(plan);

    if (handle->is_canceling())
        return MotionOutcome::CANCELED;

    return static_cast<bool>(outcome)
        ? MotionOutcome::SUCCESS
        : MotionOutcome::FAILED;
}

std::optional<
    moveit::planning_interface::MoveGroupInterface::Plan>
InspectProfileServer::planToState(
    const moveit::core::RobotState& start,
    const moveit::core::RobotState& target,
    const bool home_return)
{
    using Plan =
        moveit::planning_interface::
            MoveGroupInterface::Plan;

    // Conservative inspection-transit limits.
    
     double max_joint_travel_rad = 1.5;
     const double max_total_travel_rad = 4.0;
     double max_step_rad = 0.25;

    const motion::TransitValidator validator =
        [&](const Plan& plan) -> bool
    {
        const auto& jt =
            plan.trajectory_.joint_trajectory;

        if (jt.joint_names.empty() ||
            jt.points.empty())
        {
            return false;
        }

        double total_travel = 0.0;
        double maximum_joint_travel = 0.0;
        double maximum_step = 0.0;

        for (std::size_t j = 0;
             j < jt.joint_names.size();
             ++j)
        {
            double joint_travel = 0.0;

            for (std::size_t i = 1;
                 i < jt.points.size();
                 ++i)
            {
                const auto& previous =
                    jt.points[i - 1].positions;

                const auto& current =
                    jt.points[i].positions;

                if (previous.size() !=
                        jt.joint_names.size() ||
                    current.size() !=
                        jt.joint_names.size())
                {
                    return false;
                }

                const double step =
                    std::abs(
                        current[j] -
                        previous[j]);

                if (!std::isfinite(step))
                    return false;

                joint_travel += step;

                maximum_step =
                    std::max(
                        maximum_step,
                        step);
            }

            total_travel += joint_travel;

            maximum_joint_travel =
                std::max(
                    maximum_joint_travel,
                    joint_travel);
        }

        const bool accepted =
    home_return ||
    (maximum_joint_travel <= max_joint_travel_rad &&
     total_travel <= max_total_travel_rad &&
     maximum_step <= max_step_rad);

        if (!accepted)
        {
            RCLCPP_WARN(
                get_logger(),
                "Inspection transit rejected: "
                "total=%.3f max_joint=%.3f "
                "max_step=%.3f",
                total_travel,
                maximum_joint_travel,
                maximum_step);
        }

        return accepted;
    };

    // Use the existing portfolio, trajectory scoring,
    // and best-plan selection.
    //
    // false: stop exploring fallbacks once a
    // sufficiently good candidate is found.
    return motion::planTransit(
        *move_group_,
        get_logger(),
        start,
        target,
        validator,
        false);
}



    // if (!move_group_->setJointValueTarget(target))
    //     return std::nullopt;

    // moveit::planning_interface::
    //     MoveGroupInterface::Plan plan;

    // if (!static_cast<bool>(move_group_->plan(plan)))
    //     return std::nullopt;

    // return plan;


bool InspectProfileServer::waitForStability()
{
    const auto timeout =
        std::chrono::milliseconds(
            static_cast<int64_t>(
                stability_timeout_ * 1000.0));

    const bool stable =
        platform_supervision_->waitForStable(
            timeout);

    if (!stable)
    {
        RCLCPP_WARN(
            get_logger(),
            "Platform not stable: %s",
            platform_supervision_->reason().c_str());
    }

    return stable;
}

bool InspectProfileServer::acquireSamples(
    uint32_t count,
    double depth_range,
    const std::function<bool()>& canceled)
{
    if (!rgbd_acquisition_ || count == 0)
        return false;

    const auto deadline =
        std::chrono::steady_clock::now() + 20s;

    uint32_t integrated = 0;
    uint32_t received = 0;
    uint32_t rejected = 0;
    uint32_t timeouts = 0;

    // Boundary is established after stability.
    rclcpp::Time boundary = now();

    while (rclcpp::ok() &&
            integrated == 0 &&
           !canceled())
    {
        
             
        // retain the collection strategy here, to compensate for irregular kinect behabiour
        auto observation =
            rgbd_acquisition_->acquireAfter(
                boundary, std::chrono::milliseconds(
                    static_cast<int64_t>(
                        acquisition_timeout_ * 1000.0)));

        if (!observation)
        {
            ++timeouts;
            continue;
        }

        ++received;

        boundary = observation->stamp;

        // A viewpoint is accepted only when the
        // platform is stable at acquisition time.
        if (!platform_supervision_->stable())
        {
            ++rejected;
            continue;
        }

        const auto stamp =
            observation->stamp;
        
        const auto& info = *observation->rgb_camera_info;
        const auto& rgb = *observation->rgb;
        const auto& depth = *observation->depth_to_rgb;

        // RCLCPP_INFO(
        //     get_logger(),
        //     "RGBD geometry: "
        //     "rgb=%ux%u depth=%ux%u info=%ux%u "
        //     "fx=%.3f fy=%.3f cx=%.3f cy=%.3f "
        //     "depth_encoding=%s",
        //     rgb.width, rgb.height,
        //     depth.width, depth.height,
        //     info.width, info.height,
        //     info.k[0], info.k[4],
        //     info.k[2], info.k[5],
        //     depth.encoding.c_str());
        
        const auto& t =
            observation->camera_pose.transform.translation;

        // RCLCPP_INFO(
        //     get_logger(),
        //     "Camera TF translation: x=%.3f y=%.3f z=%.3f",
        //     t.x, t.y, t.z);
                

        if (scene_model_->addObservation(
                std::move(*observation),
                depth_range))
        {
            if (!first_observation_)
                first_observation_ = stamp;

            ++integrated;
        }
        else
        {
            ++rejected;
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "Acquisition: integrated=%u/%u "
        "received=%u rejected=%u timeouts=%u",
        integrated,
        count,
        received,
        rejected,
        timeouts);

    if (integrated == 0)
        rgbd_acquisition_->logCaptureFailure(
            boundary);

    return integrated>0;
}

void InspectProfileServer::publishSceneCloud()
{
    const auto cloud = scene_model_->cloud();

    if (!cloud || cloud->empty())
        return;

    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(*cloud, msg);

    msg.header.frame_id = scene_frame_;
    msg.header.stamp = now();

    scene_cloud_pub_->publish(msg);
}

std::optional<InspectProfileServer::ProfileEstimate>
InspectProfileServer::estimateProfile()
{
    const auto cloud = scene_model_->cloud();

    if (!cloud || cloud->empty())
        return std::nullopt;

    auto workpiece =
        extractor_.extract(cloud);

    if (!workpiece)
        return std::nullopt;

    const auto axis =
        extractor_.estimateAxisFromNormals(
            workpiece->cloud, 0.025f);

    if (!axis.allFinite() ||
        axis.norm() < 1e-6f)
    {
        return std::nullopt;
    }

    *workpiece =
        extractor_.withAxis(
            *workpiece, axis);

    auto section =
        extractor_.extractSection(
            *workpiece, 0.01f, 0.015f);

    if (!section ||
        section->points_2d.size() < 30)
    {
        return std::nullopt;
    }

    const auto profiles =
        hb_perception::ProfileLibrary::ipnProfiles();

    const auto matches =
        matcher_.match(*section, profiles);

    if (matches.empty())
        return std::nullopt;

    const auto& best = matches.front();

    if (!std::isfinite(best.rms_dist))
        return std::nullopt;

    observation_model_.reset();
    observation_model_.setMatch(best);

    if (!observation_model_.integrate(*section))
        return std::nullopt;

    ProfileEstimate estimate;

    estimate.workpiece =
        std::move(*workpiece);

    estimate.section =
        std::move(*section);

    estimate.match = best;

    estimate.coverage =
        observation_model_.coverageFraction();

    estimate.world_from_member =
        matcher_.makeProfilePose(
            estimate.section,
            estimate.match).cast<double>();

    if (!finiteRigid(
            estimate.world_from_member))
    {
        return std::nullopt;
    }

    estimate.sufficient =
        best.rms_dist < rms_target_ &&
        best.inlier_fraction >
            inlier_fraction_target_ &&
        estimate.coverage > coverage_target_;

    return estimate;
}

void InspectProfileServer::publishProfile(
    const ProfileEstimate& estimate)
{
    const std::vector<
        hb_perception::ProfileMatch> matches{
            estimate.match
        };

    auto markers =
        matcher_.getVisualization(
            estimate.section,
            matches,
            scene_frame_,
            1);

    const auto stamp = now();

    for (auto& marker : markers.markers)
        marker.header.stamp = stamp;

    profile_marker_pub_->publish(markers);

    auto msg =
        matcher_.getProfileEstimateMsg(
            scene_frame_,
            estimate.section,
            estimate.match);

    msg.header.stamp = stamp;

    // This remains a diagnostic topic.
    profile_estimate_pub_->publish(msg);
}

void InspectProfileServer::publishCoverage(
    const hb_perception::SectionModel& section)
{
    using Marker =
        visualization_msgs::msg::Marker;

    visualization_msgs::msg::MarkerArray array;

    Marker clear;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    Marker points;

    points.header.frame_id = scene_frame_;
    points.header.stamp = now();
    points.ns = "profile_support";
    points.id = 0;
    points.type = Marker::POINTS;
    points.action = Marker::ADD;

    points.pose.orientation.w = 1.0;
    points.scale.x = 0.003;
    points.scale.y = 0.003;

    for (const auto& sample :
         observation_model_.coverage())
    {
        const Eigen::Vector2f section_point =
            observation_model_.match().transform *
            sample.position;

        const Eigen::Vector3f world_point =
            section.frame *
            Eigen::Vector3f(
                section_point.x(),
                section_point.y(),
                0.0f);

        geometry_msgs::msg::Point point;

        point.x = world_point.x();
        point.y = world_point.y();
        point.z = world_point.z();

        points.points.push_back(point);

        const float support =
            std::clamp(
                sample.support, 0.0f, 1.0f);

        std_msgs::msg::ColorRGBA color;

        color.r = 1.0f - support;
        color.g = support;
        color.a = 1.0f;

        points.colors.push_back(color);
    }

    array.markers.push_back(
        std::move(points));

    coverage_marker_pub_->publish(array);
}

motion::ProfileInspectionRequest
InspectProfileServer::makeProfileInspectionRequest(
    const InspectProfile::Goal& goal) const
{
    motion::ProfileInspectionRequest request;

    request.cam_dist = goal.camera_standoff;
    request.pos_tol = inspection_pos_tol_;
    request.ang_tol = inspection_ang_tol_;
    request.num_candidates = 8;
    request.exploration_weight = exploration_weight_;
    request.motion_weight = motion_weight_;

    return request;
}

bool InspectProfileServer::updateWorkpieceCollision(
    const hb_perception::WorkpieceModel& workpiece)
{
    if (!workpiece.cloud ||
        workpiece.cloud->empty())
    {
        return false;
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;

    voxel.setInputCloud(workpiece.cloud);
    voxel.setLeafSize(0.015f, 0.015f, 0.015f);

    Cloud coarse;
    voxel.filter(coarse);

    if (coarse.empty())
        return false;

    CollisionObject object;

    object.header.frame_id = scene_frame_;
    object.header.stamp = now();

    object.id = "inspection_workpiece";
    object.operation = CollisionObject::ADD;
    object.pose.orientation.w = 1.0;

    constexpr double radius = 0.012;

    for (const auto& p : coarse.points)
    {
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            continue;
        }

        shape_msgs::msg::SolidPrimitive sphere;

        sphere.type =
            shape_msgs::msg::SolidPrimitive::SPHERE;

        sphere.dimensions = {radius};

        geometry_msgs::msg::Pose pose;

        pose.orientation.w = 1.0;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;

        object.primitives.push_back(sphere);
        object.primitive_poses.push_back(pose);
    }

    if (object.primitives.empty())
        return false;

    return planning_scene_interface_
        .applyCollisionObject(object);
}

std::optional<CollisionObject>
InspectProfileServer::buildCuttingCollision(
    const ProfileEstimate& estimate)
{
    if (!estimate.workpiece.cloud ||
        estimate.workpiece.cloud->empty() ||
        !estimate.section.cloud ||
        estimate.section.cloud->empty() ||
        !finiteRigid(estimate.world_from_member))
    {
        return std::nullopt;
    }

    const Eigen::Isometry3d member_from_world =
        estimate.world_from_member.inverse();

    double z_min =
        std::numeric_limits<double>::infinity();

    double z_max =
        -std::numeric_limits<double>::infinity();

    for (const auto& p :
         estimate.workpiece.cloud->points)
    {
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            continue;
        }

        const Eigen::Vector3d world(p.x, p.y, p.z);

        const Eigen::Vector3d local =
            member_from_world * world;

        z_min = std::min(z_min, local.z());
        z_max = std::max(z_max, local.z());
    }

    if (!std::isfinite(z_min) ||
        !std::isfinite(z_max) ||
        z_max <= z_min)
    {
        return std::nullopt;
    }

    constexpr double padding = 0.02;

    auto mesh =
        mesh_builder_.extrude(
            estimate.match.profile,
            z_min - padding,
            z_max + padding);

    if (!mesh ||
        mesh->vertices.empty() ||
        mesh->triangles.empty())
    {
        return std::nullopt;
    }

    CollisionObject object;

    object.header.frame_id = scene_frame_;
    object.header.stamp = now();

    object.id = "wk_workpiece_collision";
    object.operation = CollisionObject::ADD;
    object.pose.orientation.w = 1.0;

    object.meshes.push_back(std::move(*mesh));

    const auto serialized =
        tf2::toMsg(estimate.world_from_member);

    Eigen::Isometry3d roundtrip =
        Eigen::Isometry3d::Identity();

    tf2::fromMsg(serialized, roundtrip);

    if (!matchingPose(
            roundtrip,
            estimate.world_from_member))
    {
        return std::nullopt;
    }

    object.mesh_poses.push_back(serialized);

    return object;
}

bool InspectProfileServer::commitInitialWorkpiece(
    const ProfileEstimate& estimate,
    const std::shared_ptr<GoalHandle>& handle,
    hb_robot_interfaces::msg::WorkpieceReference&
        committed,
    std::string& error)
{
    if (handle->is_canceling())
    {
        error = "Canceled before commit";
        return false;
    }

    if (!first_observation_)
    {
        error = "No observation timestamp";
        return false;
    }

    if (!platform_supervision_->stable())
    {
        error = "Platform not stable for commit";
        return false;
    }

    auto collision =
        buildCuttingCollision(estimate);

    if (!collision)
    {
        error = "Cannot build final extrusion";
        return false;
    }

    hb_robot_interfaces::msg::WorkpieceReference
        reference;

    reference.estimate =
        matcher_.getProfileEstimateMsg(
            scene_frame_,
            estimate.section,
            estimate.match);

    // Exactly the same pose used by the mesh.
    reference.estimate.pose =
        tf2::toMsg(estimate.world_from_member);

    reference.estimate.header.frame_id =
        scene_frame_;

    reference.estimate.header.stamp = now();

    pcl::toROSMsg(
        *estimate.workpiece.cloud,
        reference.cloud);

    reference.cloud.header =
        reference.estimate.header;

    reference.cloud.header.stamp =
        *first_observation_;

    reference.relocalized = false;
    reference.expected_pose_version = 0;

    // Validate locally before contacting TaskState.
    if (collision->meshes.size() != 1 ||
        collision->mesh_poses.size() != 1 ||
        collision->header.frame_id !=
            reference.estimate.header.frame_id)
    {
        error = "Malformed final collision object";
        return false;
    }

    Eigen::Isometry3d mesh_pose =
        Eigen::Isometry3d::Identity();

    tf2::fromMsg(
        collision->mesh_poses.front(),
        mesh_pose);

    if (!matchingPose(
            mesh_pose,
            estimate.world_from_member))
    {
        error = "Extrusion and reference poses differ";
        return false;
    }

    if (!commit_client_->wait_for_service(3s))
    {
        error = "TaskState commit service unavailable";
        return false;
    }

    auto request =
        std::make_shared<CommitWorkpiece::Request>();

    request->mode =
        CommitWorkpiece::Request::INITIAL;

    request->reference = reference;
    request->collision_object = *collision;

    // INITIAL allocates new authoritative versions.
    request->expected_geometry_version = 0;
    request->expected_pose_version = 0;
    request->expected_scene_version = 0;

    auto future =
        commit_client_->async_send_request(request);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                commit_timeout_));

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() <
               deadline)
    {
        if (future.wait_for(50ms) !=
            std::future_status::ready)
        {
            continue;
        }

        const auto response = future.get();

        if (!response->accepted)
        {
            error =
                "TaskState rejected commit: " +
                response->message;

            return false;
        }

        committed = reference;

        RCLCPP_INFO(
            get_logger(),
            "Workpiece committed: "
            "id=%s geometry=%llu pose=%llu scene=%llu",
            response->reference_id.c_str(),
            static_cast<unsigned long long>(
                response->geometry_version),
            static_cast<unsigned long long>(
                response->pose_version),
            static_cast<unsigned long long>(
                response->scene_version));

        return true;
    }

    error =
        "Commit timed out; outcome unknown. "
        "Do not retry until TaskState and MoveIt "
        "have been reconciled";

    return false;
}

void InspectProfileServer::execute(
    const std::shared_ptr<GoalHandle> handle)
{
    std::lock_guard<std::mutex> execution_lock(
        execution_mutex_);

    const auto goal = handle->get_goal();

    auto result =
        std::make_shared<InspectProfile::Result>();

    result->success = false;
    result->viewpoints_captured = 0;

    first_observation_.reset();
    scene_model_->clear();
    observation_model_.reset();

    auto fail =
        [&](uint8_t code, const std::string& message)
    {
        result->success = false;
        result->result_code = code;
        result->message = message;

        RCLCPP_ERROR(
            get_logger(),
            "Inspection failed: %s",
            message.c_str());

        if (handle->is_canceling())
            handle->canceled(result);
        else
            handle->abort(result);
    };

    auto canceled = [&]() -> bool
    {
        if (!handle->is_canceling())
            return false;

        move_group_->stop();

        result->success = false;
        result->result_code =
            InspectProfile::Result::CANCELLED;

        result->message = "Inspection canceled";

        handle->canceled(result);
        return true;
    };

    std::optional<ProfileEstimate> estimate;

    const auto initial =
        move_group_->getCurrentState(2.0);

    if (!initial)
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            "Cannot obtain initial robot state");
        return;
    }

    initial->update();

    const Eigen::Isometry3d reference_camera =
        initial->getGlobalLinkTransform(camera_link_);

    for (uint32_t view = 0;
         view < goal->max_views;
         ++view)
    {
        if (canceled())
            return;

        if (!waitForStability())
        {
            if (canceled())
                return;

            fail(
                InspectProfile::Result::
                    ACQUISITION_FAILED,
                "Platform not stable before acquisition");
            return;
        }

        const bool acquired =
            acquireSamples(
                goal->samples_per_view,
                goal->depth_range,
                [&]()
                {
                    return handle->is_canceling();
                });

        if (canceled())
            return;

        if (!acquired)
        {
            fail(
                InspectProfile::Result::
                    ACQUISITION_FAILED,
                "RGB-D acquisition failed");
            return;
        }

        ++result->viewpoints_captured;

        scene_model_->downsample(0.005f);
        publishSceneCloud();

        estimate = estimateProfile();

        if (estimate)
        {
            publishProfile(*estimate);
            publishCoverage(estimate->section);

            if (!updateWorkpieceCollision(
                    estimate->workpiece))
            {
                fail(
                    InspectProfile::Result::
                        PLANNING_FAILED,
                    "Temporary inspection collision "
                    "update failed");
                return;
            }

            std_msgs::msg::Float64 rms;
            rms.data =
                estimate->match.rms_dist * 1000.0;

            rms_pub_->publish(rms);

            std_msgs::msg::Float64 inliers;
            inliers.data =
                estimate->match.inlier_fraction;

            inlier_pub_->publish(inliers);

            std_msgs::msg::Float64 coverage;
            coverage.data = estimate->coverage;

            coverage_pub_->publish(coverage);

            RCLCPP_INFO(
                get_logger(),
                "View %u: %s RMS=%.2f mm "
                "inliers=%.3f coverage=%.3f",
                view + 1,
                estimate->match.profile.name.c_str(),
                rms.data,
                inliers.data,
                coverage.data);

            if (estimate->sufficient)
                break;
        }

        if (view + 1 >= goal->max_views)
            break;

        if (!estimate)
        {
            fail(
                InspectProfile::Result::
                    INSUFFICIENT_GEOMETRY,
                "No provisional profile for NBV");
            return;
        }

        auto request =
            makeProfileInspectionRequest(*goal);

        request.world_from_member =
            estimate->world_from_member;

        request.target_world =
            estimate->world_from_member.translation();

        const auto start =
            move_group_->getCurrentState(2.0);

        if (!start)
        {
            fail(
                InspectProfile::Result::
                    PLANNING_FAILED,
                "Cannot obtain robot state for NBV");
            return;
        }

        start->update();

        request.current_camera_pose =
            start->getGlobalLinkTransform(
                camera_link_);

        request.reference_camera_pose =
            reference_camera;

        const auto candidates =
            nbv_planner_->debugCandidates(
                request, estimate->match);

        for (std::size_t i = 0;
             i < candidates.size(); ++i)
        {
            const double gain =
                nbv_planner_->predictedCoverageGain(
                    request,
                    candidates[i].nominal_cam_pose,
                    observation_model_);

            publishCameraPose(
                candidates[i].nominal_cam_pose,
                "nbv_candidates",
                static_cast<int>(100 + 4 * i),
                gain);
        }

        std::optional<motion::ProfileViewSolution>
            next_view;

        {
            planning_scene_monitor::
                LockedPlanningSceneRO locked(
                    planning_scene_monitor_);

            if (!locked)
            {
                fail(
                    InspectProfile::Result::
                        PLANNING_FAILED,
                    "Planning scene unavailable");
                return;
            }

            auto snapshot = locked->diff();

            next_view =
                nbv_planner_->plan(
                    *start,
                    request,
                    observation_model_,
                    snapshot);
        }

        if (!next_view)
        {
            fail(
                InspectProfile::Result::
                    NO_REACHABLE_VIEW,
                "No reachable next viewpoint");
            return;
        }

        publishCameraPose(
            next_view->candidate.actual_cam_pose,
            "nbv_ik_solution",
            10,
            next_view->candidate.predicted_gain);

        auto plan =
            planToState(
                *start,
                next_view->robot_state, false);

        if (!plan)
        {
            fail(
                InspectProfile::Result::
                    PLANNING_FAILED,
                "Cannot plan to next viewpoint");
            return;
        }

        const auto outcome =
            executeApprovedPlan(
                *start,
                *plan,
                handle,
                "inspect next viewpoint");

        if (outcome == MotionOutcome::CANCELED)
        {
            canceled();
            return;
        }

        if (outcome != MotionOutcome::SUCCESS)
        {
            fail(
                InspectProfile::Result::
                    PLANNING_FAILED,
                "Viewpoint motion not completed");
            return;
        }
    }

    if (canceled())
        return;

    if (!estimate)
    {
        fail(
            InspectProfile::Result::
                INSUFFICIENT_GEOMETRY,
            "No final profile estimate");
        return;
    }

    // Preserve the original inspection policy:
    // quality is reported, but a fitted estimate
    // may be returned at the view budget limit.
    //
    // The commit coordinator decides whether the
    // reference can become authoritative.

    const auto home_start =
        move_group_->getCurrentState(2.0);

    if (!home_start)
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            "Cannot obtain state for inspection home");
        return;
    }

    home_start->update();

    moveit::core::RobotState home(
        move_group_->getRobotModel());

    if (!home.setToDefaultValues(
            planning_group_,
            "inspection_home"))
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            "Missing inspection_home state");
        return;
    }

    const auto home_plan =
        planToState(*home_start, home, true);

    if (!home_plan)
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            "Cannot plan inspection home");
        return;
    }

    const auto home_outcome =
        executeApprovedPlan(
            *home_start,
            *home_plan,
            handle,
            "return inspection home");

    if (home_outcome == MotionOutcome::CANCELED)
    {
        canceled();
        return;
    }

    if (home_outcome != MotionOutcome::SUCCESS)
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            "Inspection home not completed");
        return;
    }

    if (canceled())
        return;

    if (!waitForStability())
    {
        fail(
            InspectProfile::Result::MOTION_FAILED,
            "Platform not stable for final commit");
        return;
    }

    hb_robot_interfaces::msg::WorkpieceReference
        committed;

    std::string error;

    if (!commitInitialWorkpiece(
            *estimate,
            handle,
            committed,
            error))
    {
        fail(
            InspectProfile::Result::PLANNING_FAILED,
            error);
        return;
    }

    result->profile_name =
        estimate->match.profile.name;

    result->rms_error =
        estimate->match.rms_dist;

    result->coverage =
        estimate->coverage;

    result->inlier_fraction =
        estimate->match.inlier_fraction;

    result->profile_pose.header =
        committed.estimate.header;

    result->profile_pose.pose =
        committed.estimate.pose;

    result->success = true;
    result->result_code =
        InspectProfile::Result::SUCCESS;

    result->message =
        "Profile inspection completed; "
        "workpiece committed by TaskState";

    handle->succeed(result);
}

void InspectProfileServer::publishCameraPose(
    const Eigen::Isometry3d& pose,
    const std::string& ns,
    int id,
    std::optional<double> gain)
{
    visualization_msgs::msg::MarkerArray array;

    constexpr double length = 0.15;
    constexpr double shaft = 0.006;
    constexpr double head = 0.014;
    constexpr double head_length = 0.02;

    const std::array<std::array<float, 3>, 3>
        colors{{
            {{1.0f, 0.0f, 0.0f}},
            {{0.0f, 1.0f, 0.0f}},
            {{0.0f, 0.0f, 1.0f}}
        }};

    for (int axis = 0; axis < 3; ++axis)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id =
            move_group_->getRobotModel()
                ->getModelFrame();

        marker.header.stamp = now();

        marker.ns = ns;
        marker.id = id + axis;

        marker.type =
            visualization_msgs::msg::Marker::ARROW;

        marker.action =
            visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        const Eigen::Vector3d origin =
            pose.translation();

        const Eigen::Vector3d end =
            origin +
            length * pose.linear().col(axis);

        geometry_msgs::msg::Point p0, p1;

        p0.x = origin.x();
        p0.y = origin.y();
        p0.z = origin.z();

        p1.x = end.x();
        p1.y = end.y();
        p1.z = end.z();

        marker.points = {p0, p1};

        marker.scale.x = shaft;
        marker.scale.y = head;
        marker.scale.z = head_length;

        marker.color.r = colors[axis][0];
        marker.color.g = colors[axis][1];
        marker.color.b = colors[axis][2];
        marker.color.a = 1.0f;

        array.markers.push_back(
            std::move(marker));
    }

    if (gain)
    {
        visualization_msgs::msg::Marker label;

        label.header.frame_id =
            move_group_->getRobotModel()
                ->getModelFrame();

        label.header.stamp = now();

        label.ns = ns;
        label.id = id + 3;

        label.type =
            visualization_msgs::msg::Marker::
                TEXT_VIEW_FACING;

        label.action =
            visualization_msgs::msg::Marker::ADD;

        label.pose.orientation.w = 1.0;

        label.pose.position.x =
            pose.translation().x();

        label.pose.position.y =
            pose.translation().y();

        label.pose.position.z =
            pose.translation().z() + 0.1;

        label.scale.z = 0.045;

        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;

        std::ostringstream text;

        text << std::fixed
             << std::setprecision(3)
             << "Gain: " << *gain;

        label.text = text.str();

        array.markers.push_back(
            std::move(label));
    }

    camera_pose_pub_->publish(array);
}

}  // namespace hb_robot_skills

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<
            hb_robot_skills::InspectProfileServer>();

    try
    {
        node->initialize();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Inspection initialization failed: %s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

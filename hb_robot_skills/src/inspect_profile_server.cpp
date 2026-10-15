#include "hb_robot_skills/inspect_profile_server.hpp"
#include <algorithm>
#include <cmath>
#include <array>
#include <functional>
#include <limits>
#include <optional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <chrono>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit/robot_state/conversions.h>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


namespace hb_robot_skills{ 
    using namespace std::chrono_literals;


    InspectProfileServer::InspectProfileServer(const rclcpp::NodeOptions & options)
    :rclcpp::Node("inspect_profile_server",options),
    scene_model_(std::make_shared<hb_perception::SceneModel>())
{

    // pass stuff to the private variables using some parameters
    planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
    camera_link_ = declare_parameter<std::string>("camera_link","camera_visor");
    planning_time_ = declare_parameter<double>("planning_time",5.0);
    stability_timeout_ = declare_parameter<double>("stability_timeout",5.0);
    skip_motion_ = declare_parameter<bool>("skip_inspection_motion", true);
    require_plan_approval_ = declare_parameter<bool>("require_plan_approval",true);

    rgb_topic_ = declare_parameter<std::string>("perception_rgb_topic","/k4a/rgb/image_raw");
    depth_topic_=declare_parameter<std::string>("perception_depth_topic","/k4a/depth_to_rgb/image_raw");
    camera_info_topic_=declare_parameter<std::string>("perception_camera_info_topic","/k4a/depth_to_rgb/camera_info");
    scene_frame_= declare_parameter<std::string>("scene_base_frame","world");
    acquisition_timeout_ = declare_parameter<double>("acquisition_timeout",2.0);
    
}
    void InspectProfileServer::initialize(){
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
        move_group_->setPlanningTime(planning_time_);
        
        const auto robot_model =
            move_group_->getRobotModel();

        nbv_planner_ = std::make_unique<motion::NBVPlanner>(robot_model,planning_group_,camera_link_);
        
        planning_scene_monitor_ = std::make_shared<
        planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description"
        );
        if(!planning_scene_monitor_->getPlanningScene()) throw std::runtime_error("Planning Scene Montiro failed");
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        rgbd_acquisition_ = std::make_unique<hb_perception::RGBDAcquisition>(this,
        tf_buffer_.get(),scene_frame_,rgb_topic_,depth_topic_,camera_info_topic_);
    RCLCPP_INFO(get_logger(),"Acquisition Initialized");
    
        const auto qos = rclcpp::QoS(1).transient_local().reliable();

        display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
        10);
        scene_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>
        ("/profile_inspection/scene_cloud",qos);
        
        profile_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>
        ("/profile_inspection/matches",qos);

        coverage_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>
        ("/profile_inspection/coverage",qos);
        profile_estimate_pub_ = create_publisher<hb_robot_interfaces::msg::ProfileEstimate>
        ("/profile_inspection/estimate",qos);

        camera_pose_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/profile_inspection/camera_poses",
        qos);
    


        
        action_server_ = rclcpp_action::create_server<InspectProfile>(
            shared_from_this(),
        "inspect_profile",
        std::bind(
            &InspectProfileServer::handleGoal,
            this,
            std::placeholders::_1,std::placeholders::_2
        ),
        std::bind(
            &InspectProfileServer::handleCancel,
            this,
            std::placeholders::_1
        ),
        std::bind(
            &InspectProfileServer::handleAccepted,
            this,
            std::placeholders::_1
        ));

        approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
                "approve_profile_motion",
                std::bind(&InspectProfileServer::handleApproval,this,
                    std::placeholders::_1,std::placeholders::_2)
            );
        RCLCPP_INFO(get_logger(),"Inspect Profile Server Initialized");  

    }


rclcpp_action::GoalResponse
InspectProfileServer::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const InspectProfile::Goal> goal)
{
    if (goal->max_views == 0 ||
        goal->samples_per_view == 0 ||
        goal->depth_range <= 0.0 ||
        goal->camera_standoff <= 0.0)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
InspectProfileServer::handleCancel(
    const std::shared_ptr<GoalHandleInspectProfile>)
{
    if (move_group_)
        move_group_->stop();

    approval_cv_.notify_all();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void InspectProfileServer::handleAccepted(
    const std::shared_ptr<GoalHandleInspectProfile> goal_handle)
{
    std::thread(
        [this, goal_handle]()
        {
            execute(goal_handle);
        }).detach();
}

void InspectProfileServer::handleApproval(
    const std::shared_ptr<
        hb_robot_interfaces::srv::ApproveMotion::Request> request,
    std::shared_ptr<
        hb_robot_interfaces::srv::ApproveMotion::Response> response)
{
    {
        std::lock_guard<std::mutex> lock(approval_mutex_);
        motion_approved_ = request->approve;
    }

    response->accepted = true;
    response->message =
        request->approve ? "Motion approved" : "Motion not approved";

    approval_cv_.notify_all();
}

std::optional<
    moveit::planning_interface::MoveGroupInterface::Plan>
InspectProfileServer::planToState(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& target_state)
{
    move_group_->setStartState(start_state);

    if (!move_group_->setJointValueTarget(target_state))
        return std::nullopt;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (!static_cast<bool>(move_group_->plan(plan)))
        return std::nullopt;

    return plan;
}
bool InspectProfileServer::updateWorkpieceCollision(
    const hb_perception::WorkpieceModel& workpiece)
{
    if (!workpiece.cloud || workpiece.cloud->empty())
        return false;

    // 1. Longitudinal direction from the smallest eigenvector
    //    of the surface-normal covariance.
    Eigen::Vector3f z =
        extractor_.estimateAxisFromNormals(
            workpiece.cloud, 0.025f);

    if (!z.allFinite() || z.norm() < 1e-6f)
    {
        RCLCPP_WARN(
            get_logger(),
            "Invalid normal-derived workpiece axis");
        return false;
    }

    z.normalize();

    // Keep the axis sign consistent with the extracted workpiece.
    if (z.dot(workpiece.l_axes) < 0.0f)
        z = -z;

    // 2. Build an orthonormal frame around the normal-derived axis.
    // Prefer the secondary PCA direction for cross-section orientation.
    Eigen::Vector3f x =
        workpiece.p_axes.col(1);

    x -= x.dot(z) * z;

    // If the projected PCA direction is degenerate, use a
    // world-axis fallback that is not parallel to z.
    if (!x.allFinite() || x.norm() < 1e-5f)
    {
        const Eigen::Vector3f reference =
            std::abs(z.dot(Eigen::Vector3f::UnitX())) < 0.9f
                ? Eigen::Vector3f::UnitX()
                : Eigen::Vector3f::UnitY();

        x = reference - reference.dot(z) * z;
    }

    x.normalize();

    Eigen::Vector3f y = z.cross(x).normalized();

    // Recompute x to remove numerical orthogonality error.
    x = y.cross(z).normalized();

    Eigen::Matrix3f R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    // 3. Project every finite cloud point into the new box frame.
    Eigen::Vector3f min_local =
        Eigen::Vector3f::Constant(
            std::numeric_limits<float>::infinity());

    Eigen::Vector3f max_local =
        Eigen::Vector3f::Constant(
            -std::numeric_limits<float>::infinity());

    std::size_t valid_points = 0;

    for (const auto& p : workpiece.cloud->points)
    {
        const Eigen::Vector3f world(p.x, p.y, p.z);

        if (!world.allFinite())
            continue;

        const Eigen::Vector3f local =
            R.transpose() * (world - workpiece.centroid);

        min_local = min_local.cwiseMin(local);
        max_local = max_local.cwiseMax(local);

        ++valid_points;
    }

    if (valid_points < 30 ||
        !min_local.allFinite() ||
        !max_local.allFinite())
    {
        RCLCPP_WARN(
            get_logger(),
            "Insufficient points for workpiece collision box");
        return false;
    }

    // 4. Dimensions and center from the actual point bounds.
    const Eigen::Vector3f center_local =
        0.5f * (min_local + max_local);

    const Eigen::Vector3f center_world =
        workpiece.centroid + R * center_local;

    const Eigen::Vector3f dimensions =
        max_local - min_local;

    constexpr double padding = 0.02;  // 20 mm on each side

    Eigen::Quaternionf q(R);
    q.normalize();

    // 5. Construct the MoveIt collision object.
    moveit_msgs::msg::CollisionObject object;

    object.header.frame_id = scene_frame_;
    object.header.stamp = now();

    object.id = "inspection_workpiece";
    object.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;

    box.dimensions.resize(3);

    box.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_X] =
        static_cast<double>(dimensions.x()) + 2.0 * padding;

    box.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Y] =
        static_cast<double>(dimensions.y()) + 2.0 * padding;

    box.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Z] =
        static_cast<double>(dimensions.z()) + 2.0 * padding;

    geometry_msgs::msg::Pose pose;

    pose.position.x = center_world.x();
    pose.position.y = center_world.y();
    pose.position.z = center_world.z();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);

    // 6. Publish to the MoveIt planning scene.
    const bool applied =
        planning_scene_interface_.applyCollisionObject(object);

    if (!applied)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to apply workpiece collision object");
        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Workpiece collision box: "
        "%.3f x %.3f x %.3f m, "
        "%zu points",
        box.dimensions[
            shape_msgs::msg::SolidPrimitive::BOX_X],
        box.dimensions[
            shape_msgs::msg::SolidPrimitive::BOX_Y],
        box.dimensions[
            shape_msgs::msg::SolidPrimitive::BOX_Z],
        valid_points);

    return true;
}



bool InspectProfileServer::waitForStability()
{
    if (stability_timeout_ < 0.0)
        return false;

    // Initial implementation: fixed settling delay.
    // Replace with actual robot/camera stability checks later.
    rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(stability_timeout_)));

    return rclcpp::ok();
}

bool InspectProfileServer::acquireSamples(
    uint32_t sample_count,
    double depth_range)
{
    if (!rgbd_acquisition_ || sample_count == 0)
        return false;

    uint32_t integrated = 0;

    // Every acquisition must be newer than this boundary.
    rclcpp::Time boundary = now();

    for (uint32_t i = 0; i < sample_count; ++i)
    {
        auto observation =
            rgbd_acquisition_->acquireAfter(
                boundary,
                std::chrono::milliseconds(
                    static_cast<int64_t>(
                        acquisition_timeout_ * 1000.0)));

        if (!observation)
            continue;

        const auto stamp = observation->stamp;
        boundary = stamp;

        if (scene_model_->addObservation(
                std::move(*observation), depth_range))
        {
            ++integrated;
        }
    }

    return integrated > 0;
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

    auto workpiece = extractor_.extract(cloud);

    if (!workpiece)
        return std::nullopt;

    
    const Eigen::Vector3f normal_axis =
        extractor_.estimateAxisFromNormals(
            workpiece->cloud, 0.025f);

    if (!normal_axis.allFinite() ||
        normal_axis.norm() < 1e-6f)
    {
        return std::nullopt;
    }

    *workpiece =
        extractor_.withAxis(*workpiece, normal_axis);

    auto section =
        extractor_.extractSection(
            *workpiece, 0.01f, 0.015f);

    if (!section || section->points_2d.size() < 30)
        return std::nullopt;

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
    estimate.workpiece = std::move(*workpiece);
    estimate.section = std::move(*section);
    estimate.match = best;
    estimate.coverage = observation_model_.coverageFraction();

    Eigen::Isometry3f section_from_profile =
        Eigen::Isometry3f::Identity();

    section_from_profile.linear().block<2, 2>(0, 0) =
        best.transform.linear();

    section_from_profile.translation().head<2>() =
        best.transform.translation();

    estimate.world_from_member =
        (estimate.section.frame * section_from_profile)
            .cast<double>();
    estimate.sufficient =
        best.rms_dist < 0.005f &&
        best.inlier_fraction > 0.85f &&
        estimate.coverage > 0.65f;

    return estimate;
}
  void InspectProfileServer::publishProfile(
    const ProfileEstimate& estimate)
{
    const std::vector<hb_perception::ProfileMatch> matches{
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
    profile_estimate_pub_->publish(msg);
}

void InspectProfileServer::publishCoverage(
    const hb_perception::SectionModel& section)
{
    using Marker = visualization_msgs::msg::Marker;

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

    for (const auto& sample : observation_model_.coverage())
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

        geometry_msgs::msg::Point p;
        p.x = world_point.x();
        p.y = world_point.y();
        p.z = world_point.z();

        points.points.push_back(p);

        const float support =
            std::clamp(sample.support, 0.0f, 1.0f);

        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f - support;
        color.g = support;
        color.b = 0.0f;
        color.a = 1.0f;

        points.colors.push_back(color);
    }

    array.markers.push_back(std::move(points));
    coverage_marker_pub_->publish(array);
} 
motion::ProfileInspectionRequest
InspectProfileServer::makeProfileInspectionRequest(
    const InspectProfile::Goal& goal) const
{
    motion::ProfileInspectionRequest request;
    request.cam_dist = goal.camera_standoff;
    request.pos_tol = 0.075;
    request.ang_tol = 0.25;
    request.num_candidates = 8;

    return request;
}

void InspectProfileServer::execute(
    const std::shared_ptr<GoalHandleInspectProfile> goal_handle)
{
    std::lock_guard<std::mutex> execution_lock(
        execution_mutex_);

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<InspectProfile::Result>();

    result->success = false;
    result->viewpoints_captured = 0;

    scene_model_->clear();
    observation_model_.reset();

    {
        std::lock_guard<std::mutex> approval_lock(
            approval_mutex_);
        motion_approved_ = false;
    }

    auto fail = [&](uint8_t code, const std::string& message)
    {
        result->success = false;
        result->result_code = code;
        result->message = message;
        goal_handle->abort(result);
    };

    auto canceled = [&]()
    {
        if (!goal_handle->is_canceling())
            return false;

        move_group_->stop();
        result->result_code =
            InspectProfile::Result::CANCELLED;
        result->message = "Inspection cancelled";
        goal_handle->canceled(result);
        return true;
    };

    // An NBV target requires at least a provisional member axis.
    std::optional<ProfileEstimate> estimate;

    for (uint32_t view = 0; view < goal->max_views; ++view)
    {
        if (canceled())
            return;

        // First iteration observes the current camera position.
        // Subsequent iterations arrive here after a completed move.
        if (!acquireSamples(
                goal->samples_per_view,
                goal->depth_range))
        {
            fail(
                InspectProfile::Result::ACQUISITION_FAILED,
                "No fresh RGB-D observation integrated");
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
            if (!updateWorkpieceCollision(estimate->workpiece))
            {
                fail(
                    InspectProfile::Result::PLANNING_FAILED,
                    "Could not update workpiece collision geometry");
                return;
            }
            RCLCPP_INFO(
                get_logger(),
                "View %u: %s RMS %.2f mm, inliers %.2f, "
                "coverage %.2f",
                view + 1,
                estimate->match.profile.name.c_str(),
                estimate->match.rms_dist * 1000.0f,
                estimate->match.inlier_fraction,
                estimate->coverage);

            if (estimate->sufficient)
            {
                result->success = true;
                result->result_code =
                    InspectProfile::Result::SUCCESS;

                result->profile_name =
                    estimate->match.profile.name;

                result->rms_error =
                    estimate->match.rms_dist;

                result->inlier_fraction =
                    estimate->match.inlier_fraction;

                result->profile_pose.header.frame_id =
                    scene_frame_;

                result->profile_pose.header.stamp = now();

                result->profile_pose.pose =
                    tf2::toMsg(estimate->world_from_member);

                result->message =
                    "Profile inspection sufficient";

                goal_handle->succeed(result);
                return;
            }
        }

        if (view + 1 >= goal->max_views)
            break;

        if (!estimate)
        {
            fail(
                InspectProfile::Result::INSUFFICIENT_GEOMETRY,
                "Cannot estimate a provisional member axis "
                "for the next viewpoint");
            return;
        }

        if (canceled())
            return;

        auto request = makeProfileInspectionRequest(*goal);

        request.world_from_member =
            estimate->world_from_member;

        request.target_world =
            estimate->world_from_member.translation();

         const auto current_state =
            move_group_->getCurrentState(2.0);

        if (!current_state)
        {
            fail(
                InspectProfile::Result::PLANNING_FAILED,
                "Cannot obtain current robot state");
            return;
        }

        current_state->update();

        request.current_camera_pose = current_state->getGlobalLinkTransform(camera_link_);



        const auto candidates =
    nbv_planner_->debugCandidates(
        request,
        estimate->match);

        for (std::size_t i = 0; i < candidates.size(); ++i)
        {


            const double gain = nbv_planner_->predictedCoverageGain( request, candidates[i].nominal_cam_pose,observation_model_);

            publishCameraPose(
                candidates[i].nominal_cam_pose,
                "nbv_candidates",
                static_cast<int>(100 + 4 * i),
                gain
                );
        }

        

        std::optional<motion::ProfileViewSolution> next_view;

        {
            // Take a snapshot of the monitored planning scene.
            planning_scene_monitor::LockedPlanningSceneRO locked(
                planning_scene_monitor_);

            if (!locked)
            {
                fail(
                    InspectProfile::Result::PLANNING_FAILED,
                    "Planning scene unavailable");
                return;
            }

            // diff() creates a planning scene derived from
            // the monitored scene for candidate checking.
            auto scene_snapshot = locked->diff();

            
            
            next_view = nbv_planner_->plan(
                *current_state,
                request,
               observation_model_,
               scene_snapshot
            );
        }

        if (!next_view)
        {
            fail(
                InspectProfile::Result::NO_REACHABLE_VIEW,
                "No valid camera viewpoint found");
            return;
        }

        

        publishCameraPose(
            next_view->candidate.actual_cam_pose,
            "nbv_ik_solution",
            10,
            next_view->candidate.predicted_gain
        
        );

        auto trajectory = planToState(
            *current_state,
            next_view->robot_state);

        if (!trajectory)
        {
            fail(
                InspectProfile::Result::PLANNING_FAILED,
                "MoveIt could not plan to selected viewpoint");
            return;
        }

        moveit_msgs::msg::DisplayTrajectory display_msg;
        display_msg.model_id = move_group_->getRobotModel()->getName();
        moveit::core::robotStateToRobotStateMsg(
            *current_state,
            display_msg.trajectory_start
        );
        display_msg.trajectory.push_back(
            trajectory->trajectory_
        );
        display_traj_pub_->publish(display_msg);

        if (skip_motion_)
        {
            // Do not acquire again and call it a new viewpoint.
            // Dry-run ends after showing the first proposed move.
            fail(
                InspectProfile::Result::INSUFFICIENT_GEOMETRY,
                "Dry-run: viewpoint planned but not executed");
            return;
        }

        if (require_plan_approval_)
        {
            {
                std::lock_guard<std::mutex> approval_lock(
                    approval_mutex_);
                motion_approved_ = false;
            }

            RCLCPP_INFO(
                get_logger(),
                "Awaiting approval for inspection move %u",
                view + 1);

            while (rclcpp::ok())
            {
                if (canceled())
                    return;

                std::unique_lock<std::mutex> approval_lock(
                    approval_mutex_);

                if (approval_cv_.wait_for(
                        approval_lock,
                        100ms,
                        [this]()
                        {
                            return motion_approved_;
                        }))
                {
                    break;
                }
            }

            if (!rclcpp::ok())
                return;
        }

        if (canceled())
            return;

        const auto execution_result =
            move_group_->execute(*trajectory);

        if (!static_cast<bool>(execution_result))
        {
            fail(
                InspectProfile::Result::MOTION_FAILED,
                "Inspection camera motion failed");
            return;
        }

        if (canceled())
            return;

        if (!waitForStability())
        {
            fail(
                InspectProfile::Result::MOTION_FAILED,
                "Camera did not settle");
            return;
        }

        // Next loop iteration acquires a fresh observation
        // and refits against the accumulated scene.
    }

    fail(
        InspectProfile::Result::INSUFFICIENT_GEOMETRY,
        "Inspection budget exhausted without sufficient geometry");
}

void InspectProfileServer::publishCameraPose(
    const Eigen::Isometry3d& pose,
    const std::string& ns,
    int id_base,
    std::optional<double> predicted_gain
)
{
    visualization_msgs::msg::MarkerArray array;

    constexpr double axis_length = 0.15;
    constexpr double shaft_diameter = 0.006;
    constexpr double head_diameter = 0.014;
    constexpr double head_length = 0.02;

    const std::array<std::array<float, 3>, 3> colors{{
        {{1.0f, 0.0f, 0.0f}}, // +X forward: red
        {{0.0f, 1.0f, 0.0f}}, // +Y left: green
        {{0.0f, 0.0f, 1.0f}}  // +Z up: blue
    }};

    for (int i = 0; i < 3; ++i)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = move_group_->getRobotModel()->getModelFrame();
        marker.header.stamp = now();
        marker.ns = ns;
        marker.pose.orientation.w=1.0;
        marker.id = id_base + i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        const Eigen::Vector3d origin = pose.translation();
        const Eigen::Vector3d endpoint =
            origin + axis_length * pose.linear().col(i);

        geometry_msgs::msg::Point start;
        start.x = origin.x();
        start.y = origin.y();
        start.z = origin.z();

        geometry_msgs::msg::Point end;
        end.x = endpoint.x();
        end.y = endpoint.y();
        end.z = endpoint.z();

        marker.points = {start, end};

        marker.scale.x = shaft_diameter;
        marker.scale.y = head_diameter;
        marker.scale.z = head_length;

        marker.color.r = colors[i][0];
        marker.color.g = colors[i][1];
        marker.color.b = colors[i][2];
        marker.color.a = 1.0f;

        array.markers.push_back(std::move(marker));
    }

    if(predicted_gain){
        visualization_msgs::msg::Marker label;

        label.header.frame_id = move_group_->getRobotModel()->getModelFrame();
        label.header.stamp = now();
        label.ns = ns;
        label.pose.orientation.w=1.0;
        label.id = id_base + 3;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x= pose.translation().x();
        label.pose.position.y= pose.translation().y();
        label.pose.position.z= pose.translation().z()+0.1;
        
        label.scale.z = 0.045;

        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;
        std::ostringstream ss;
        ss <<std::fixed<<std::setprecision(3)<<"Gain: "<<*predicted_gain;
        label.text =ss.str();
        
        array.markers.push_back(std::move(label));
    }

    RCLCPP_INFO(
    get_logger(),
    "Publishing camera pose '%s' in frame '%s' "
    "at [%.3f, %.3f, %.3f]",
    ns.c_str(),
    move_group_->getRobotModel()->getModelFrame().c_str(),
    pose.translation().x(),
    pose.translation().y(),
    pose.translation().z());
    camera_pose_pub_->publish(array);
}



}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<hb_robot_skills::InspectProfileServer>();

    try
    {
        node->initialize();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Initialization failed: %s",
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
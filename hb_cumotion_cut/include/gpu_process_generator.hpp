#pragma once
#include "hb_robot_skills/motion/cut_process_trajectory_generator.hpp"
#include "gpu_rpc.hpp"
namespace hb_robot_skills::motion {
class GpuProcessGenerator
{
public:
    GpuProcessGenerator(
        moveit::core::RobotModelConstPtr robot_model,
        std::string planning_group,
        std::string plasma_link,
        rclcpp::Node& node, ProcessGeneratorOptions options);

    ProcessGenerationResult generate(
        const CutSegment& segment,
        const moveit::core::RobotState& reference_state,
        const planning_scene::PlanningSceneConstPtr& scene) const;

private:
    using StatePtr = moveit::core::RobotStatePtr;
    using StatePath = std::vector<StatePtr>;

    enum class Failure
    {
        None,
        IK,
        Collision,
        JointLimits,
        JointJump,
        TCPError
    };

    struct SolveResult
    {
        StatePath states;
        Failure failure = Failure::None;
    };

    struct CartesianPath
    {
        std::vector<Eigen::Isometry3d> poses;

        // Indices of phase boundaries in poses.
        std::size_t approach_end = 0;
        std::size_t cut_end = 0;
    };

    CartesianPath buildCartesianPath(
        const CutSegment& segment,
        const Eigen::Matrix3d& orientation_offset) const;

    void appendInterpolated(
        std::vector<Eigen::Isometry3d>& output,
        const Eigen::Isometry3d& start,
        const Eigen::Isometry3d& end) const;

    SolveResult solvePath(
        const CartesianPath& path,
        const moveit::core::RobotState& seed,
        const planning_scene::PlanningSceneConstPtr& scene,
        bool backward,
        bool constrain_initial_ik, const CutSegment& segment) const;

    bool validState(
        const moveit::core::RobotState& state,
        const planning_scene::PlanningSceneConstPtr& scene,
        Failure& failure) const;

    bool closeToTarget(
        const moveit::core::RobotState& state,
        const Eigen::Isometry3d& target) const;

    double jointDistance(
        const moveit::core::RobotState& a,
        const moveit::core::RobotState& b) const;

    bool jointStepValid(
        const moveit::core::RobotState& a,
        const moveit::core::RobotState& b) const;

    double pathCost(const StatePath& states) const;

    bool isDuplicate(
        const ProcessCandidate& candidate,
        const std::vector<ProcessCandidate>& existing) const;

    ProcessCandidate makeCandidate(
        const CutSegment& segment,
        const CartesianPath& path,
        StatePath states,
        double roll,
        std::size_t seed_index,
        bool backward) const;

    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_ = nullptr;

    std::string planning_group_;
    std::string plasma_link_;
    ProcessGeneratorOptions options_;
    rclcpp::Client<hb_cumotion_cut::srv::SolveProcessPath>::SharedPtr client_;
};

}  // namespace hb_robot_skills::motion

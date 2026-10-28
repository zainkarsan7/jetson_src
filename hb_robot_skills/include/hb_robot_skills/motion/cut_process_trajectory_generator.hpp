
#pragma once

#include "cut_planner.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace hb_robot_skills::motion
{
using CandidateId = std::size_t;
using EdgeId = std::size_t;

enum class EdgeStatus{
    Unknown,
    Valid,
    FailedWithBudget,
    Invalid
};

struct TransitEdge{

    EdgeId id;
    CandidateId from;
    CandidateId to;

    EdgeStatus status = EdgeStatus::Unknown;

    double estimated_cost = 0.0;
    double actual_cost = 0.0;
    std::vector<moveit::core::RobotStatePtr> trajectory;
    std::size_t attempts = 0;



};


struct ProcessGeneratorOptions
{
    // Uniform samples over the allowed local-Z torch rotation.
    std::size_t roll_samples = 12;
    double min_roll_rad = -3.141592653589793;
    double max_roll_rad =  3.141592653589793;

    // IK attempts per roll. The first uses the supplied reference.
    std::size_t seeds_per_roll = 12;

    // Maximum Cartesian spacing between consecutive samples.
    double max_translation_step_m = 0.005;
    double max_rotation_step_rad = 0.05;

    // Per-waypoint IK timeout.
    double ik_timeout_s = 0.05;

    // Local continuation guard, per joint variable.
    // This is a sampling/branch-continuity limit, not a
    // physical velocity or acceleration limit.
    double max_joint_step_rad = 0.25;

    // TCP accuracy checks after IK.
    double position_tolerance_m = 0.001;
    double orientation_tolerance_rad = 0.02;

    // Reject solutions that are essentially the same.
    double duplicate_joint_distance = 0.15;

    std::size_t max_candidates = 24;

    // Diagnostic option: try solving the same path backward.
    // The returned candidate is still ordered approach->cut->retract.
    bool enable_reverse_ik_search = true;
};


struct ProcessCandidate
{

    CandidateId id;
    std::string segment_name;

    double torch_roll_rad = 0.0;
    std::size_t seed_index = 0;
    bool solved_backward = false;

    // Each vector includes its start and end states.
    // Boundaries are deliberately duplicated.
    std::vector<moveit::core::RobotStatePtr> approach;
    std::vector<moveit::core::RobotStatePtr> cut;
    std::vector<moveit::core::RobotStatePtr> retract;

    moveit::core::RobotStatePtr entry;
    moveit::core::RobotStatePtr exit;

    // Lower is better. This is a ranking heuristic,
    // not a time or energy estimate.
    double score = 0.0;
};

struct ProcessGenerationStats
{
    std::size_t attempts = 0;
    std::size_t ik_failures = 0;
    std::size_t collision_failures = 0;
    std::size_t joint_limit_failures = 0;
    std::size_t joint_jump_failures = 0;
    std::size_t tcp_error_failures = 0;
    std::size_t duplicate_candidates = 0;
    std::size_t accepted = 0;
};

struct ProcessGenerationResult
{
    std::vector<ProcessCandidate> candidates;
    ProcessGenerationStats stats;
};

class CutProcessTrajectoryGenerator
{
public:
    CutProcessTrajectoryGenerator(
        moveit::core::RobotModelConstPtr robot_model,
        std::string planning_group,
        std::string plasma_link,
        ProcessGeneratorOptions options = {});

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
        double roll_rad) const;

    void appendInterpolated(
        std::vector<Eigen::Isometry3d>& output,
        const Eigen::Isometry3d& start,
        const Eigen::Isometry3d& end) const;

    SolveResult solvePath(
        const CartesianPath& path,
        const moveit::core::RobotState& seed,
        const planning_scene::PlanningSceneConstPtr& scene,
        bool backward) const;

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
};

}  // namespace hb_robot_skills::motion

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_state/conversions.h>
#include "hb_robot_skills/motion/cut_graph_planner.hpp"

using namespace hb_robot_skills::motion;

class CutGraphPlannerTest : public ::testing::Test {
protected:
    moveit::core::RobotModelPtr model;
    void SetUp() override {
        auto urdf = urdf::parseURDF(R"(
          <robot name="test"><link name="base"/><link name="tip"/>
          <joint name="joint" type="revolute"><parent link="base"/>
          <child link="tip"/><axis xyz="0 0 1"/>
          <limit lower="-6.28" upper="6.28" effort="10" velocity="1"/>
          </joint></robot>)");
        ASSERT_TRUE(urdf);
        auto srdf = std::make_shared<srdf::Model>();
        ASSERT_TRUE(srdf->initString(*urdf, R"(
          <robot name="test"><group name="arm"><joint name="joint"/></group></robot>)"));
        model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    }
    moveit::core::RobotState state(double q) const {
        moveit::core::RobotState result(model);
        result.setToDefaultValues();
        result.setVariablePosition("joint", q);
        result.update();
        return result;
    }
    CutGraphPlanner::Plan trajectory(std::initializer_list<double> positions) const {
        CutGraphPlanner::Plan result;
        moveit::core::robotStateToRobotStateMsg(state(*positions.begin()), result.start_state_);
        auto& joint_trajectory = result.trajectory_.joint_trajectory;
        joint_trajectory.joint_names = {"joint"};
        for (double q : positions) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {q};
            joint_trajectory.points.push_back(point);
        }
        return result;
    }
    ProcessCandidate candidate(double q, double sampled_score) const {
        ProcessCandidate result;
        result.entry = std::make_shared<moveit::core::RobotState>(state(q));
        result.exit = result.entry;
        result.approach = result.cut = result.retract = {result.entry};
        result.score = sampled_score;
        return result;
    }
    std::optional<ExecutablePlan> run(double limit, bool detour) {
        CutGraphPlanner::Options options;
        options.max_transit_joint_travel_rad = limit;
        CutGraphPlanner planner(model, "arm",
            [&](const moveit::core::RobotState& from, const moveit::core::RobotState& to)
                -> std::optional<CutGraphPlanner::Plan> {
                const double a = from.getVariablePosition("joint");
                const double b = to.getVariablePosition("joint");
                if (detour && b == 0.4) return trajectory({a, 3.0, b});
                return trajectory({a, b});
            },
            [&](const CutSegment&, const ProcessCandidate& candidate)
                -> std::optional<SegmentMotionPlan> {
                const double q = candidate.entry->getVariablePosition("joint");
                const auto plan = trajectory({q, q});
                return SegmentMotionPlan{plan, plan, plan};
            }, options);
        CutPlan cuts;
        cuts.segments.emplace_back();
        // A deliberately pessimistic sampled score must not hide the nearby
        // candidate when the actual process planner returns a shorter path.
        return planner.plan(cuts, {{candidate(0.4, 1000.0), candidate(0.8, 0.0)}},
                            state(0.0), "test", "test");
    }
    double entry(const ExecutablePlan& result) const {
        return result.cuts.front().transit_in.trajectory_.joint_trajectory.points.back().positions.front();
    }
};

TEST_F(CutGraphPlannerTest, SampledScoreDoesNotHideCheaperActualRoute) {
    const auto result = run(0.0, false);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(entry(*result), 0.4);
}

TEST_F(CutGraphPlannerTest, ActualDetourCostsMoreThanFartherDirectMotion) {
    const auto result = run(0.0, true);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(entry(*result), 0.8);
}

TEST_F(CutGraphPlannerTest, TravelLimitRejectsDetourAndTriesAlternative) {
    const auto result = run(1.0, true);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(entry(*result), 0.8);
}

TEST_F(CutGraphPlannerTest, NoRouteWhenEveryTransitExceedsLimit) {
    EXPECT_FALSE(run(0.1, false));
}

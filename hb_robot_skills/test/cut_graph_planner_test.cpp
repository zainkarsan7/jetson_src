#include <gtest/gtest.h>
#include <cmath>
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

    std::optional<ExecutablePlan> multi(bool between, bool final_return, bool fail_link = false,
                                       bool wrong_boundary = false, bool enforce_position = false) {
        CutGraphPlanner::Options options;
        options.return_home_between_cuts = between;
        options.return_home_after_last_cut = final_return;
        if (enforce_position) options.tcp_link = "tip";
        CutGraphPlanner planner(model, "arm",
            [&](const auto& from, const auto& to) -> std::optional<CutGraphPlanner::Plan> {
                const double a = from.getVariablePosition("joint"), b = to.getVariablePosition("joint");
                if (fail_link && a == 1.0 && b == 2.0) return std::nullopt;
                if (wrong_boundary && a == 1.0 && b == 2.0) return trajectory({0.0, b});
                return trajectory({a,b});
            },
            [&](const CutSegment&, const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
                const double q = c.entry->getVariablePosition("joint");
                auto p = trajectory({q,q});
                return SegmentMotionPlan{p,p,p};
            }, options);
        CutPlan cuts;
        cuts.segments.resize(2);
        cuts.segments[0].name = "first";
        cuts.segments[1].name = "second";
        if (enforce_position) cuts.segments[0].start_pose.translation().x() = 1.0;
        return planner.plan(cuts, {{candidate(1.0,0)}, {candidate(2.0,0)}}, state(0), "plan", "estimate");
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


TEST_F(CutGraphPlannerTest, DirectConnectionOwnedOnlyByDestination) {
    const auto result = multi(false, true);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->cuts.size(), 2u);
    EXPECT_FALSE(result->cuts[0].transit_out);
    const auto& link = result->cuts[1].transit_in.trajectory_.joint_trajectory.points;
    EXPECT_DOUBLE_EQ(link.front().positions.front(), 1.0);
    EXPECT_DOUBLE_EQ(link.back().positions.front(), 2.0);
    ASSERT_TRUE(result->cuts[1].transit_out);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_out->trajectory_.joint_trajectory.points.back().positions.front(), 0.0);
}
TEST_F(CutGraphPlannerTest, LegacyModeReturnsHomeBetweenCuts) {
    const auto result = multi(true, true);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->cuts[0].transit_out);
    EXPECT_DOUBLE_EQ(result->cuts[0].transit_out->trajectory_.joint_trajectory.points.back().positions.front(), 0.0);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(), 0.0);
    EXPECT_TRUE(result->cuts[1].transit_out);
}
TEST_F(CutGraphPlannerTest, FinalReturnCanBeOmittedIndependently) {
    for (bool between : {false, true}) {
        const auto result = multi(between, false);
        ASSERT_TRUE(result);
        EXPECT_FALSE(result->cuts.back().transit_out);
        EXPECT_EQ(result->cuts.front().transit_out.has_value(), between);
    }
}
TEST_F(CutGraphPlannerTest, FailedDirectConnectionFallsBackToValidatedHomeRoute) {
    const auto result = multi(false, true, true);
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->cuts.front().transit_out);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(), 0.0);
}
TEST_F(CutGraphPlannerTest, DiscontinuousDirectConnectionIsRejectedAndHomeRouteUsed) {
    const auto result = multi(false, true, false, true);
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->cuts.front().transit_out);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(), 0.0);
}
TEST_F(CutGraphPlannerTest, PilzPathOutsideOriginalPositionTubeIsRejected) {
    EXPECT_FALSE(multi(false, true, false, false, true));
}
TEST_F(CutGraphPlannerTest, AlternativeCandidateSelectedForFeasibleDirectConnection) {
    CutGraphPlanner::Options options;
    options.return_home_after_last_cut = false;
    CutGraphPlanner planner(model, "arm",
        [&](const auto& from, const auto& to) -> std::optional<CutGraphPlanner::Plan> {
            const double a=from.getVariablePosition("joint"), b=to.getVariablePosition("joint");
            if (a == 1.0 && b == 2.0) return std::nullopt;
            return trajectory({a,b});
        }, [&](const CutSegment&, const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
            const double q=c.entry->getVariablePosition("joint");
            auto p=trajectory({q,q});
            return SegmentMotionPlan{p,p,p};
        }, options);
    CutPlan cuts;
    cuts.segments.resize(2);
    const auto result=planner.plan(cuts, {{candidate(1.0,0),candidate(1.2,0)}, {candidate(2.0,0)}}, state(0), "p", "e");
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(entry(*result),1.2);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(),1.2);
}


TEST_F(CutGraphPlannerTest, CartesianPositionToleranceIsHardAndNotExpanded) {
    for (double deviation : {0.002, 0.004}) {
        CutGraphPlanner::Options options;
        options.tcp_link = "tip";
        CutGraphPlanner planner(model, "arm",
            [&](const auto& a, const auto& b) -> std::optional<CutGraphPlanner::Plan> {
                return trajectory({a.getVariablePosition("joint"), b.getVariablePosition("joint")});
            }, [&](const CutSegment&, const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
                const double q=c.entry->getVariablePosition("joint");
                auto p=trajectory({q,q}); return SegmentMotionPlan{p,p,p};
            }, options);
        CutPlan cuts;
        cuts.segments.resize(1);
        auto& seg=cuts.segments[0];
        seg.pos_tol=0.003;
        for (auto* pose : {&seg.approach_pose,&seg.start_pose,&seg.end_pose,&seg.retract_pose})
            pose->translation().x()=deviation;
        EXPECT_EQ(planner.plan(cuts, {{candidate(0.4,0)}}, state(0), "p", "e").has_value(), deviation < seg.pos_tol);
    }
}

TEST_F(CutGraphPlannerTest, InteriorCartesianOvershootRejectedDespiteMatchingEndpoints) {
    auto urdf=urdf::parseURDF(R"(<robot name="slider"><link name="base"/><link name="tip"/>
      <joint name="joint" type="prismatic"><parent link="base"/><child link="tip"/><axis xyz="1 0 0"/>
      <limit lower="-5" upper="5" effort="10" velocity="1"/></joint></robot>)");
    auto srdf=std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf,R"(<robot name="slider"><group name="arm"><joint name="joint"/></group></robot>)"));
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    auto c=candidate(0,0);
    c.exit=std::make_shared<moveit::core::RobotState>(state(1));
    c.cut={c.entry,c.exit};
    CutPlan cuts;
    cuts.segments.resize(1);
    cuts.segments[0].end_pose.translation().x()=1;
    cuts.segments[0].retract_pose.translation().x()=1;
    CutGraphPlanner::Options options;
    options.tcp_link="tip";
    for (bool overshoot : {false,true}) {
        CutGraphPlanner planner(model,"arm",
            [&](const auto& a,const auto& b) -> std::optional<CutGraphPlanner::Plan> {
                return trajectory({a.getVariablePosition("joint"),b.getVariablePosition("joint")});
            }, [&](const CutSegment&,const ProcessCandidate&) -> std::optional<SegmentMotionPlan> {
                return SegmentMotionPlan{trajectory({0,0}),trajectory({0,overshoot?2.0:0.5,1}),trajectory({1,1})};
            },options);
        EXPECT_EQ(planner.plan(cuts,{{c}},state(0),"p","e").has_value(), !overshoot);
    }
}

TEST_F(CutGraphPlannerTest, AlternativeBoxConstraintUsesRequestedPositionAndOrientationBounds) {
    CutPlanner planner(model,"arm","tip");
    CutSegment segment;
    segment.end_pose.translation().x()=0.4;
    const auto constraint=planner.makeBoxConstraints(segment,0.003,0.02);
    ASSERT_EQ(constraint.position_constraints.size(),1u);
    const auto& dimensions=constraint.position_constraints.front().constraint_region.primitives.front().dimensions;
    EXPECT_NEAR(dimensions[0],0.4,1e-12);
    EXPECT_NEAR(std::hypot(dimensions[1]/2,dimensions[2]/2),0.003,1e-12);
    ASSERT_EQ(constraint.orientation_constraints.size(),1u);
    EXPECT_DOUBLE_EQ(constraint.orientation_constraints.front().absolute_x_axis_tolerance,0.02);
    EXPECT_THROW(planner.makeBoxConstraints(segment,0,0.02),std::invalid_argument);
}


TEST_F(CutGraphPlannerTest, GeometricBeamRecoversWhenCheapestApproachIsADeadEnd) {
    const auto* group=model->getJointModelGroup("arm");
    const auto expand=[&](std::size_t phase,const auto& from,std::size_t) {
        std::vector<moveit::core::RobotStatePtr> result;
        if (phase==0) {
            result.push_back(std::make_shared<moveit::core::RobotState>(state(0.1)));
            result.push_back(std::make_shared<moveit::core::RobotState>(state(1.0)));
        } else if (from.getVariablePosition("joint")>0.5) {
            result.push_back(std::make_shared<moveit::core::RobotState>(state(1.0+0.1*phase)));
        }
        return result;
    };
    EXPECT_FALSE(searchConstraintChain(state(0),group,1,64,expand));
    const auto result=searchConstraintChain(state(0),group,3,64,expand);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ((*result)[0]->getVariablePosition("joint"),1.0);
    EXPECT_DOUBLE_EQ((*result)[3]->getVariablePosition("joint"),1.3);
}
TEST_F(CutGraphPlannerTest, GeometricBeamSharesRatherThanMultipliesStageBudget) {
    std::array<std::size_t,4> budgets{};
    const auto result=searchConstraintChain(state(0),model->getJointModelGroup("arm"),3,64,
        [&](std::size_t phase,const auto&,std::size_t budget) {
            budgets[phase]+=budget;
            return std::vector<moveit::core::RobotStatePtr>{
                std::make_shared<moveit::core::RobotState>(state(0.1)),
                std::make_shared<moveit::core::RobotState>(state(0.5)),
                std::make_shared<moveit::core::RobotState>(state(1.0))};
        });
    ASSERT_TRUE(result);
    for (auto budget:budgets) EXPECT_EQ(budget,64u);
}
TEST_F(CutGraphPlannerTest, GeometricBeamNeverReturnsPartialChain) {
    const auto result=searchConstraintChain(state(0),model->getJointModelGroup("arm"),3,64,
        [&](std::size_t phase,const auto&,std::size_t) {
            std::vector<moveit::core::RobotStatePtr> out;
            if (phase<3) out.push_back(std::make_shared<moveit::core::RobotState>(state(0.2)));
            return out;
        });
    EXPECT_FALSE(result);
}
TEST_F(CutGraphPlannerTest, GeometricBeamDeduplicatesEndpointsWithoutLosingBetterHistory) {
    const auto result=searchConstraintChain(state(0),model->getJointModelGroup("arm"),3,64,
        [&](std::size_t phase,const auto&,std::size_t) {
            std::vector<moveit::core::RobotStatePtr> out;
            if (phase==0) {
                out.push_back(std::make_shared<moveit::core::RobotState>(state(2.0)));
                out.push_back(std::make_shared<moveit::core::RobotState>(state(0.5)));
            } else out.push_back(std::make_shared<moveit::core::RobotState>(state(0.5)));
            return out;
        });
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ((*result)[0]->getVariablePosition("joint"),0.5);
}
TEST_F(CutGraphPlannerTest, GeometricBeamAcceptsOneCompleteChainBelowRequestedWidth) {
    const auto result=searchConstraintChain(state(0),model->getJointModelGroup("arm"),3,64,
        [&](std::size_t,const auto&,std::size_t) {
            return std::vector<moveit::core::RobotStatePtr>{std::make_shared<moveit::core::RobotState>(state(0.2))};
        });
    EXPECT_TRUE(result);
}


TEST_F(CutGraphPlannerTest, HomeCompetesWithActualDirectDetourCost) {
    for (bool direct_detour : {false, true}) {
        CutGraphPlanner::Options options;
        options.return_home_after_last_cut = false;
        CutGraphPlanner planner(model, "arm",
            [&](const auto& from, const auto& to) -> std::optional<CutGraphPlanner::Plan> {
                const double a=from.getVariablePosition("joint"), b=to.getVariablePosition("joint");
                // Endpoint estimate prefers direct 1->2; actual path may be wild.
                if (direct_detour && a==1 && b==2) return trajectory({a,-3.0,b});
                return trajectory({a,b});
            }, [&](const CutSegment&, const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
                const double q=c.entry->getVariablePosition("joint");
                auto p=trajectory({q,q}); return SegmentMotionPlan{p,p,p};
            }, options);
        CutPlan cuts; cuts.segments.resize(2);
        const auto result=planner.plan(cuts,{{candidate(1,0)},{candidate(2,0)}},state(0),"p","e");
        ASSERT_TRUE(result);
        EXPECT_EQ(result->cuts.front().transit_out.has_value(),direct_detour);
        EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(),direct_detour?0.0:1.0);
        EXPECT_FALSE(result->cuts.back().transit_out);
    }
}
TEST_F(CutGraphPlannerTest, HomeAlternativeRespectsTravelBudgetAndFailsWhenBothRoutesInvalid) {
    for (bool home_valid : {false,true}) {
        CutGraphPlanner::Options options;
        options.return_home_after_last_cut=false;
        options.max_transit_joint_travel_rad=2.1;
        CutGraphPlanner planner(model,"arm",
            [&](const auto& from,const auto& to) -> std::optional<CutGraphPlanner::Plan> {
                const double a=from.getVariablePosition("joint"),b=to.getVariablePosition("joint");
                if (a==1 && (b==2 || (!home_valid && b==0))) return trajectory({a,-3.0,b});
                return trajectory({a,b});
            },[&](const CutSegment&,const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
                auto p=trajectory({c.entry->getVariablePosition("joint"),c.entry->getVariablePosition("joint")});
                return SegmentMotionPlan{p,p,p};
            },options);
        CutPlan cuts; cuts.segments.resize(2);
        const auto result=planner.plan(cuts,{{candidate(1,0)},{candidate(2,0)}},state(0),"p","e");
        EXPECT_EQ(result.has_value(),home_valid);
        if (result) EXPECT_TRUE(result->cuts.front().transit_out);
    }
}
TEST_F(CutGraphPlannerTest, MixedRouteUsesHomeOnlyAtExpensiveBoundary) {
    CutGraphPlanner::Options options;
    options.return_home_after_last_cut=false;
    CutGraphPlanner planner(model,"arm",
        [&](const auto& from,const auto& to) -> std::optional<CutGraphPlanner::Plan> {
            const double a=from.getVariablePosition("joint"), b=to.getVariablePosition("joint");
            if (a==1 && b==2) return trajectory({a,-3.0,b});
            return trajectory({a,b});
        },[&](const CutSegment&,const ProcessCandidate& c) -> std::optional<SegmentMotionPlan> {
            auto p=trajectory({c.entry->getVariablePosition("joint"),c.entry->getVariablePosition("joint")});
            return SegmentMotionPlan{p,p,p};
        },options);
    CutPlan cuts; cuts.segments.resize(3);
    const auto result=planner.plan(cuts,{{candidate(1,0)},{candidate(2,0)},{candidate(2.2,0)}},state(0),"p","e");
    ASSERT_TRUE(result);
    ASSERT_EQ(result->cuts.size(),3u);
    EXPECT_TRUE(result->cuts[0].transit_out);
    EXPECT_FALSE(result->cuts[1].transit_out);
    EXPECT_FALSE(result->cuts[2].transit_out);
    EXPECT_DOUBLE_EQ(result->cuts[1].transit_in.trajectory_.joint_trajectory.points.front().positions.front(),0.0);
    EXPECT_DOUBLE_EQ(result->cuts[2].transit_in.trajectory_.joint_trajectory.points.front().positions.front(),2.0);
}

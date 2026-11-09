#include "gpu_process_generator.hpp"
#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <thread>

using namespace hb_robot_skills::motion;
using Service = hb_cumotion_cut::srv::SolveProcessPath;

class ProcessBridgeTest : public testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override {
        auto urdf = urdf::parseURDF(R"(<robot name="slide"><link name="world"/><link name="tip"/>
          <joint name="x" type="prismatic"><parent link="world"/><child link="tip"/>
          <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
          </joint></robot>)");
        auto srdf = std::make_shared<srdf::Model>();
        ASSERT_TRUE(srdf->initString(*urdf, R"(<robot name="slide"><group name="arm">
          <joint name="x"/></group></robot>)"));
        model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
        node = std::make_shared<rclcpp::Node>("process_bridge_test");
        service = node->create_service<Service>("/cut_gpu/process",
            [this](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
                res->success = true;
                res->path.joint_names = req->seed.name;
                for (const auto& target : req->poses) {
                    trajectory_msgs::msg::JointTrajectoryPoint p;
                    p.positions = {target.position.x + offset};
                    res->path.points.push_back(p);
                }
            });
        executor.add_node(node);
        spinning = std::thread([this]{ executor.spin(); });
    }
    void TearDown() override {
        executor.cancel();
        if (spinning.joinable()) spinning.join();
    }
    ProcessGenerationResult solve() {
        ProcessGeneratorOptions options;
        options.min_roll_rad = options.max_roll_rad = 0.0;
        options.roll_samples = options.seeds_per_roll = 1;
        GpuProcessGenerator generator(model, "arm", "tip", *node, options);
        CutSegment segment;
        segment.start_pose.translation().x() = 0.01;
        segment.end_pose.translation().x() = 0.05;
        segment.retract_pose.translation().x() = 0.07;
        auto scene = std::make_shared<planning_scene::PlanningScene>(model);
        return generator.generate(segment, scene->getCurrentState(), scene);
    }
    std::atomic<double> offset{0.0};
    moveit::core::RobotModelPtr model;
    rclcpp::Node::SharedPtr node;
    rclcpp::Service<Service>::SharedPtr service;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread spinning;
};

TEST_F(ProcessBridgeTest, ReconstructsThreePhasesWithSharedBoundaries) {
    const auto result = solve();
    ASSERT_EQ(result.candidates.size(), 1u);
    const auto& c = result.candidates.front();
    EXPECT_EQ(c.approach.back(), c.cut.front());
    EXPECT_EQ(c.cut.back(), c.retract.front());
    EXPECT_NEAR(c.exit->getVariablePosition("x"), 0.07, 1e-9);
}

TEST_F(ProcessBridgeTest, RejectsSuccessfulRemoteIKOffTheCutLine) {
    offset = 0.02;
    const auto result = solve();
    EXPECT_TRUE(result.candidates.empty());
    EXPECT_GT(result.stats.tcp_error_failures, 0u);
}

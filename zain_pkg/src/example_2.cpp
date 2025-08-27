#include <pluginlib/class_loader.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger logger = rclcpp::get_logger("planning_scene_example");



int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_opts;
    node_opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("planning_scene_example",node_opts);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor](){executor.spin();}).detach();

    const std::string PLANNING_GROUP = "manipulator";
    robot_model_loader::RobotModelLoader rml(node,"robot_description");
    const moveit::core::RobotModelPtr& rob_mod = rml.getModel();
    moveit::core::RobotStatePtr rob_state(new moveit::core::RobotState(rob_mod));
    const moveit::core::JointModelGroup* jmg = rob_state->getJointModelGroup(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group(node,PLANNING_GROUP);
    std::vector<double> joint_vals = move_group.getCurrentJointValues();
    
    planning_scene::PlanningScenePtr p_scene(new planning_scene::PlanningScene(rob_mod));
    
    p_scene->getCurrentStateNonConst().setJointGroupPositions(jmg,move_group.getCurrentJointValues());
    
    const moveit::core::RobotState& p_state = p_scene->getCurrentState();
    std::vector<double> p_vals;
    p_state.copyJointGroupPositions("manipulator",p_vals);
    // for (std::size_t i = 0; i <p_vals.size(); ++i){
    //     RCLCPP_INFO(logger, "p_scene joint [%zu] = %f ",i,p_vals[i]);
    // }
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    

    if (!node->get_parameter("ompl.planning_plugin",planner_plugin_name)){
        
        RCLCPP_FATAL(logger,"couldnt find");
    }
    try{
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core","planning_interface::PlannerManager"
        ));
      
    }catch(pluginlib::PluginlibException& ex){
        RCLCPP_FATAL(logger," exception creating plugin loader %s",ex.what());

    }

    if (planner_plugin_name.empty()){
        RCLCPP_ERROR(logger, "no plugins defined");
        return -1;
    }
    
    RCLCPP_INFO(logger, "found planner %s",planner_plugin_name.c_str());
    
    try{
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if(!planner_instance->initialize(rob_mod,node,node->get_namespace())){RCLCPP_FATAL(logger,"another thing fucked up");}
        RCLCPP_INFO(logger,"using the interface %s",planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex){
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes){
            ss << cls << " ";
            RCLCPP_ERROR(logger, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                    ex.what(), ss.str().c_str());

        }
        
    }
    
    namespace rvt = rviz_visual_tools;

    moveit_visual_tools::MoveItVisualTools vis_tools(node,"world","move_group_tutorial",move_group.getRobotModel());
    vis_tools.enableBatchPublishing();
    vis_tools.deleteAllMarkers();
    vis_tools.trigger();
    vis_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    vis_tools.publishText(text_pose,"goddamn-finally",rvt::WHITE,rvt::XLARGE);
    vis_tools.trigger();

    vis_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    vis_tools.trigger();
    
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.75;
    pose.pose.position.z = 0.3;
    pose.pose.orientation.w = 1.0;

    std::vector<double> tol_pose(3,0.01);
    std::vector<double> tol_angle(3,0.01);
    vis_tools.publishAxisLabeled(pose.pose, "goal_1");
    vis_tools.publishText(text_pose, "Pose-Goal-(1)", rvt::WHITE, rvt::XLARGE);
    vis_tools.trigger();

    moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ur10e_tool0",pose,tol_pose,tol_angle);
    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -5.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 5.0;

      planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(p_scene,req,res.error_code_);
      
      context->solve(res);
      
      if (res.error_code_.val != res.error_code_.SUCCESS){
        RCLCPP_ERROR(logger,"couldn fucking do it");
        return 0;
        }

    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",1);
    moveit_msgs::msg::DisplayTrajectory dtraj;
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    dtraj.trajectory_start = response.trajectory_start;
    dtraj.trajectory.push_back(response.trajectory);
    vis_tools.publishTrajectoryLine(dtraj.trajectory.back(),move_group.getRobotModel()->getLinkModel("ur10e_tool0"),jmg);
    vis_tools.trigger();
    display_pub->publish(dtraj);
    rob_state->setJointGroupPositions(jmg,response.trajectory.joint_trajectory.points.back().positions);
    p_scene->setCurrentState(*rob_state.get());
    vis_tools.prompt("next demo joint space goal");
    
    /// JOINT SPACE GOAL
    moveit::core::RobotState goal_state(rob_mod);
    std::vector<double> degree_vals = {314,-35,-80,295,-16,0};
    std::vector<double> joint_values;
    for (double deg : degree_vals){
        joint_values.push_back(deg*(M_PI/180.0));
    }
    
    goal_state.setJointGroupPositions(jmg,joint_values);
    moveit_msgs::msg::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state,jmg);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    context = planner_instance->getPlanningContext(p_scene,req,res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS){
        RCLCPP_ERROR(logger,"couldnt do it again");
    }
    res.getMessage(response);
    dtraj.trajectory.push_back(response.trajectory);
    vis_tools.publishTrajectoryLine(dtraj.trajectory.back(),move_group.getRobotModel()->getLinkModel("ur10e_tool0"),jmg);
    vis_tools.trigger();
    display_pub->publish(dtraj);

    rob_state->setJointGroupPositions(jmg,response.trajectory.joint_trajectory.points.front().positions);
    p_scene->setCurrentState(*rob_state.get());
    vis_tools.trigger();

    vis_tools.prompt("next demo constrained motion");
    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0.5;
    pose.pose.position.z = 0.25;
    pose.pose.orientation.w = 1.0;

    moveit_msgs::msg::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("ur10e_tool0",pose,tol_pose,tol_angle);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal_2);


    geometry_msgs::msg::QuaternionStamped quat;
    quat.header.frame_id = "world";
    req.path_constraints = kinematic_constraints::constructGoalConstraints("ur10e_tool0",quat);

//     req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
//     req.workspace_parameters.min_corner.z = -3.0;
// req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
//     req.workspace_parameters.max_corner.z = 3.0;

    context = planner_instance->getPlanningContext(p_scene,req,res.error_code_);
    context->solve(res);
    res.getMessage(response);
    dtraj.trajectory.push_back(response.trajectory);
    vis_tools.publishTrajectoryLine(dtraj.trajectory.back(),move_group.getRobotModel()->getLinkModel("ur10e_tool0"),jmg);
    vis_tools.trigger();
    display_pub->publish(dtraj);
    vis_tools.publishAxisLabeled(pose.pose,"last-pose");
    rob_state->setJointGroupPositions(jmg,response.trajectory.joint_trajectory.points.back().positions);
    p_scene->setCurrentState(*rob_state);
    vis_tools.trigger();
    vis_tools.prompt("shutdown next");

    planner_instance.reset();
    rclcpp::shutdown();


    return 0;
}
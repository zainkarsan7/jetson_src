#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <thread>

using moveit::planning_interface::MoveGroupInterface;

class my_robot_state: public rclcpp::Node{
    public:
    my_robot_state(const rclcpp::NodeOptions &options):Node("rob_state_node",options),
    mgi_node_(std::make_shared<rclcpp::Node>("mgi_node")),
    move_group_interface(mgi_node_,"manipulator"),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
    rml(mgi_node_),
    rob_model_(rml.getModel()),
    rob_state_(std::make_shared<moveit::core::RobotState>(rob_model_)),
    planning_group_("manipulator")
    {
        executor_->add_node(mgi_node_);
        executor_thread_ = std::thread([this](){executor_->spin();
        });
        jmg = rob_model_->getJointModelGroup(planning_group_);
       

    }
        
    
    void solve_IK(){
        std::vector<double> j_vals;
        auto current_pose = move_group_interface.getCurrentPose();
        auto current_j_vals = move_group_interface.getCurrentJointValues();
        
    
        RCLCPP_INFO(this->get_logger(), "Pose x is %f", current_pose.pose.position.x);
    
        // Prepare target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.pose.position.x;
        target_pose.position.y = current_pose.pose.position.y;
        target_pose.position.z = current_pose.pose.position.z + 0.05;
    
        // Initialize robot state with current joint values
        rob_state_->setJointGroupPositions(jmg, current_j_vals);
    
        bool found_ik = rob_state_->setFromIK(jmg,target_pose,0.1);
        
        if (found_ik){
            rob_state_->copyJointGroupPositions(jmg, j_vals);
            
            for (std::size_t i=0; i<current_j_vals.size();++i){
                RCLCPP_INFO(this->get_logger(),"joint %ld: %f",i,j_vals[i]);
            }
        }
        else
        { RCLCPP_INFO(this->get_logger(),"failed");}
        
    }
    

    private:
    rclcpp::Node::SharedPtr mgi_node_;
    MoveGroupInterface move_group_interface;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    robot_model_loader::RobotModelLoader rml;
    
    moveit::core::RobotModelPtr rob_model_;
    moveit::core::RobotStatePtr rob_state_;
    std::string planning_group_;
    const moveit::core::JointModelGroup* jmg;
        
};
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto my_node = std::make_shared<my_robot_state>(node_options);
    
    my_node->solve_IK();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
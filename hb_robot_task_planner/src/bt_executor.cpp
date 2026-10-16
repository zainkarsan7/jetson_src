#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hb_robot_interfaces/action/cut_profile.hpp>


using namespace std::chrono_literals;
using CutProfile = hb_robot_interfaces::action::CutProfile;
class CutProfileNode: public BT::StatefulActionNode{
    public:
    using GoalHandle = rclcpp_action::ClientGoalHandle<CutProfile>;


    CutProfileNode(const std::string& name,
        const BT::NodeConfiguration& config,
        const rclcpp::Node::SharedPtr& node): BT::StatefulActionNode(name,config), node_(node)
    {

        client_ = rclcpp_action::create_client<CutProfile>(node_,"cut_profile");

    }

    static BT::PortsList providedPorts(){
        return{
            BT::InputPort<double>("standoff"),
            BT::InputPort<double>("ang_tol"),
            BT::InputPort<double>("pos_tol")
        };
    }

    BT::NodeStatus onStart() override{
        if(!client_->action_server_is_ready()){
            RCLCPP_ERROR(node_->get_logger(),"server not ready/unavail");
            return BT::NodeStatus::FAILURE;
        }

        CutProfile::Goal goal;
        if(!getInput("standoff",goal.standoff)||
        !getInput("ang_tol",goal.ang_tol)||
        !getInput("pos_tol",goal.pos_tol)    
    ){
        throw BT::RuntimeError("cutprofile input missing");
    }
    goal.execute = false;
    goal_future_ = client_->async_send_goal(goal);
    result_future_ = {};
    goal_handle_.reset();
    return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override{

        if(!goal_handle_){
            if(goal_future_.wait_for(0s)!= std::future_status::ready){
                return BT::NodeStatus::RUNNING;
            }
            goal_handle_ = goal_future_.get();
            if(!goal_handle_){
                RCLCPP_ERROR(node_->get_logger(),"waited but server rejected goal handle");
                return BT::NodeStatus::FAILURE;
            }
            // WE GOT  a goal handle now 
            // wait for a result future
            result_future_ = client_->async_get_result(goal_handle_);

        }
        if(result_future_.wait_for(0s)!=std::future_status::ready){
            return BT::NodeStatus::RUNNING;

        }
        const auto wrapped_result = result_future_.get();
        if(wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED||!wrapped_result.result||!wrapped_result.result->success){
            RCLCPP_ERROR(node_->get_logger(),"got a result, failed");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),"succeeded %s", wrapped_result.result->message.c_str());
        return BT::NodeStatus::SUCCESS;

    }

    void onHalted()override{
        if(goal_handle_){
            client_->async_cancel_goal(goal_handle_);
        }
        else if(goal_future_.valid() && goal_future_.wait_for(0s) == std::future_status::ready){
            auto handle = goal_future_.get();
            if(handle){
                
                client_->async_cancel_goal(handle);
            }
        }
    }



    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<CutProfile>::SharedPtr client_;

        std::shared_future<GoalHandle::SharedPtr> goal_future_;
        std::shared_future<GoalHandle::WrappedResult> result_future_;
        GoalHandle::SharedPtr goal_handle_;
};


int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("hb_bt_executor");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor](){
        executor.spin();
    });
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<CutProfileNode>(
        "CutProfile", [node](const std::string& name,
        const BT::NodeConfiguration& config){
            return std::make_unique<CutProfileNode>(name,config,node);
        }
    );

    const auto tree_file = node->declare_parameter<std::string>("tree_file", "");
    if(tree_file.empty()){
        RCLCPP_ERROR(node->get_logger(),"need tree file parameter");
        executor.cancel();
        ros_thread.join();
        rclcpp::shutdown();
        return 1;
    }
    int exit_code = 0;
    try{
        auto tree = factory.createTreeFromFile(tree_file);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
            status = tree.tickRoot();
            std::this_thread::sleep_for(50ms);

        }
        if(status != BT::NodeStatus::SUCCESS){
            tree.haltTree();
            exit_code = 1;

        }
        RCLCPP_INFO(node->get_logger(),"behavior tree finished with %d",static_cast<int>(status));
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "error %s", e.what());
        exit_code = 1;

    }
    executor.cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return exit_code;


}
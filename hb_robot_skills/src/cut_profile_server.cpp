#include "hb_robot_skills/cut_profile_server.hpp"
#include <chrono>
#include <thread>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;
namespace hb_robot_skills{

    CutProfileServer::CutProfileServer(const rclcpp::NodeOptions &options):Node("cut_profile_server",options){
        planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
        plasma_link_ = declare_parameter<std::string>("plasma_link","ur10e_torch_link");
        planning_time_ = declare_parameter<double>("planning_time",5.0);

        approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_motion",
            std::bind(&CutProfileServer::handleApproval,this,
                std::placeholders::_1,std::placeholders::_2)
        );  

        action_server_ = rclcpp_action::create_server<CutProfile>(this,"cut_profile",
            std::bind(
            &CutProfileServer::handleGoal,
            this,
            std::placeholders::_1,std::placeholders::_2
            ),
        std::bind(
                &CutProfileServer::handleCancel,
                this,
                std::placeholders::_1
            ),
        std::bind(
                &CutProfileServer::handleAccepted,
                this,
                std::placeholders::_1
            ));

        auto estimate_qos = rclcpp::QoS(1).reliable().transient_local();



        profile_estimate_sub_ = create_subscription<hb_robot_interfaces::msg::ProfileEstimate>("perception/profile_estimate",
        estimate_qos, std::bind(&CutProfileServer::profileEstimateCallback,this,
        std::placeholders::_1));

        auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
        visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cut_profile/cut_plan",marker_qos);

    }

    void CutProfileServer::initialize(){

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
        move_group_->setPlanningTime(planning_time_);
        

        
        const auto robot_model =
                move_group_->getRobotModel();
        
        cut_planner_ = std::make_unique<motion::CutPlanner>(            
        move_group_->getRobotModel(),
        planning_group_,
        plasma_link_);
        
        RCLCPP_INFO(
                get_logger(),
                "Plasma link parameter: '%s'",
                plasma_link_.c_str());
        RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
        RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(get_logger(),"End-effector link %s",move_group_->getEndEffectorLink().c_str());
        
        RCLCPP_INFO(get_logger(),"CutProfileServer Initialized"); 


    }




    rclcpp_action::GoalResponse CutProfileServer::handleGoal(const rclcpp_action::GoalUUID & , 
    std::shared_ptr<const CutProfile::Goal> goal){
        
        // RCLCPP_INFO(get_logger(),"recieved inspection goal with %zu viewpoints", goal->viewpoints.size());
        // if (goal-> < 1){
        //     RCLCPP_WARN(get_logger(), "Rejecting, no viewpoints");
        //     return rclcpp_action::GoalResponse::REJECT;
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

    rclcpp_action::CancelResponse CutProfileServer::handleCancel(const std::shared_ptr<GoalHandleCutProfile> ){
        RCLCPP_INFO(get_logger(),"cancel requested");
    
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CutProfileServer::handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle){
        std::thread{
            std::bind(&CutProfileServer::execute,
            this,
            goal_handle        
            )
        }.detach();
    }

    void CutProfileServer::profileEstimateCallback(
        const hb_robot_interfaces::msg::ProfileEstimate::SharedPtr msg
    ){
        std::lock_guard<std::mutex>lock(estimate_mutex_);
        latest_estimate_ = *msg;
    }

    std::optional<hb_robot_interfaces::msg::ProfileEstimate> CutProfileServer::latestProfileEstimate()const{
        std::lock_guard<std::mutex> lock(estimate_mutex_);
        return latest_estimate_;
    }

    void CutProfileServer::publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& stage){
                auto feedback = std::make_shared<CutProfile::Feedback>();
                feedback->stage = stage;
                goal_handle->publish_feedback(feedback);
            }


    void CutProfileServer::execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle){

        //approval machienry 
        std::lock_guard<std::mutex> lock(approval_mutex_);
        {
            motion_approved_ = false;
        }


        //un pack / get the goal
        auto const goal = goal_handle->get_goal();

        //make empty result
        auto result = std::make_shared<CutProfile::Result>();

        publishFeedback(goal_handle,"acquiring estimate");

        const auto estimate_opt = latestProfileEstimate();
        if(!estimate_opt){
            result->success = false;
            result->message= "couldnt get estimate";
            goal_handle->abort(result);
            return;
        }
        //get the estimate;
        const auto estimate = *estimate_opt;
        RCLCPP_INFO(get_logger(),"Using profile estimate %s | rms %.2f | inlier %.2f | score %.2f", 
        estimate.profile_name.c_str(), estimate.rms_distance,estimate.inlier_fraction,estimate.score);
        
        publishFeedback(goal_handle,"loading profile geom");

        const auto profile_opt = hb_perception::ProfileLibrary::find(estimate.profile_name);
        if(!profile_opt){
            result->success = false;
            std::ostringstream ss;
            ss << "looking up " << estimate.profile_name.c_str();
            result->message= ss.str();
            goal_handle->abort(result);
            return;
        }

        publishFeedback(goal_handle,"generating cut plan");

        motion::CutRequest request;
        request.standoff = goal->standoff;
        auto current_state = move_group_->getCurrentState(2.0);

        if(!current_state){
            RCLCPP_ERROR(get_logger(),"couldnt get current state");
            result->success = false;
            result->message= "couldnt get current state";
            goal_handle->abort(result);
            return;
        }
        current_state->update();

        RCLCPP_ERROR(get_logger(),"current state dirty after update: %s",current_state->dirty()?"true":"false");
        

        auto p_scene = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());
        const auto plan_opt = cut_planner_->plan(estimate,*profile_opt,request,*current_state,p_scene);
        if(!plan_opt){
            result->success = false;
            result->message= "planner failed";
            goal_handle->abort(result);
            return;
        }
        const motion::CutPlan plan = *plan_opt;
        RCLCPP_INFO(get_logger(),"generated %zu cut segments",plan.segments.size());


        publishVisualization(plan,estimate.header.frame_id);
        publishFeedback(goal_handle,"visualizing, cut plan ready");


        if(!goal->execute){
            result->success = true;
            result->message = "finished";
            goal_handle->succeed(result);
            return;
        }

        motion_approved_ = true;
        if(motion_approved_){
        RCLCPP_INFO(get_logger(),"skip motion is true, no motion executed");
        }
        else{
            RCLCPP_INFO(get_logger(),"skip motion is false, awaiting approval");
            while(rclcpp::ok()){
                if (goal_handle->is_canceling()){
                    result->message = "cancelled";
                    goal_handle->canceled(result);
                    return;
                }
                std::unique_lock<std::mutex> approval_lock(
                    approval_mutex_
                );
                if(approval_cv_.wait_for(approval_lock, std::chrono::milliseconds(100),
            [this](){return motion_approved_;})){break;}

            }
            RCLCPP_INFO(get_logger(),"motion_approved");
            publishFeedback(goal_handle, "motion approved");
        }

        result->success = true;
        result->message = "cut approved, exec not implemented yet";
        goal_handle->succeed(result);


    }
    
    namespace
{

geometry_msgs::msg::Point
toPoint(
    const Eigen::Vector3f& p)
{
    geometry_msgs::msg::Point msg;

    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();

    return msg;
}


visualization_msgs::msg::Marker
makeArrow(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int id,
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& direction,
    float length)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
        frame_id;

    marker.header.stamp =
        stamp;

    marker.ns = ns;
    marker.id = id;

    marker.type =
        visualization_msgs::msg::Marker::ARROW;

    marker.action =
        visualization_msgs::msg::Marker::ADD;


    marker.points.push_back(
        toPoint(origin));

    marker.points.push_back(
        toPoint(
            origin +
            direction.normalized() * length));


    /*
     * ARROW using points:
     *
     * scale.x = shaft diameter
     * scale.y = head diameter
     * scale.z = head length
     */
    marker.scale.x = 0.003;
    marker.scale.y = 0.007;
    marker.scale.z = 0.010;

    marker.color.a = 1.0;

    return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray
CutProfileServer::makeVisualization(
    const motion::CutPlan& plan,
    const std::string& frame_id) const
{
    visualization_msgs::msg::MarkerArray array;

    const auto stamp =
        now();


    /*
     * Clear previous plan first.
     */
    visualization_msgs::msg::Marker clear;

    clear.action =
        visualization_msgs::msg::Marker::DELETEALL;

    array.markers.push_back(clear);


    int id = 0;


    for (const auto& segment :
         plan.segments)
    {
        if (segment.points.size() < 2) {
            continue;
        }


        const auto& start =
            segment.points.front();

        const auto& end =
            segment.points.back();


        const Eigen::Vector3f midpoint =
            0.5f *
            (start.pos +
             end.pos);


        /*
         * ---------------------------------------------------
         * Cut segment
         * ---------------------------------------------------
         */

        visualization_msgs::msg::Marker line;

        line.header.frame_id =
            frame_id;

        line.header.stamp =
            stamp;

        line.ns =
            "cut_segments";

        line.id =
            id++;

        line.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        line.action =
            visualization_msgs::msg::Marker::ADD;

        line.scale.x =
            0.005;


        line.points.push_back(
            toPoint(start.pos));

        line.points.push_back(
            toPoint(end.pos));


        /*
         * Cyan-ish cut line.
         */
        line.color.r = 0.0;
        line.color.g = 1.0;
        line.color.b = 1.0;
        line.color.a = 1.0;


        array.markers.push_back(
            line);


        /*
         * ---------------------------------------------------
         * Tangent
         * ---------------------------------------------------
         */

        auto tangent =
            makeArrow(
                frame_id,
                stamp,
                "cut_tangents",
                id++,
                midpoint,
                start.tangent,
                0.05f);


        /*
         * Green tangent.
         */
        tangent.color.r = 0.0;
        tangent.color.g = 1.0;
        tangent.color.b = 0.0;


        array.markers.push_back(
            tangent);


        /*
         * ---------------------------------------------------
         * Surface normal
         * ---------------------------------------------------
         */

        auto normal =
            makeArrow(
                frame_id,
                stamp,
                "cut_normals",
                id++,
                midpoint,
                start.srf_norm,
                0.05f);


        /*
         * Red normal.
         */
        normal.color.r = 1.0;
        normal.color.g = 0.0;
        normal.color.b = 0.0;


        array.markers.push_back(
            normal);


        /*
         * ---------------------------------------------------
         * Label
         * ---------------------------------------------------
         */

        visualization_msgs::msg::Marker label;

        label.header.frame_id =
            frame_id;

        label.header.stamp =
            stamp;

        label.ns =
            "cut_labels";

        label.id =
            id++;

        label.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        label.action =
            visualization_msgs::msg::Marker::ADD;


        label.pose.position =
            toPoint(
                midpoint +
                Eigen::Vector3f(
                    0.0f,
                    0.0f,
                    0.025f));


        label.pose.orientation.w =
            1.0;


        label.scale.z =
            0.025;


        label.color.r = 1.0;
        label.color.g = 1.0;
        label.color.b = 1.0;
        label.color.a = 1.0;


        label.text =
            segment.name;


        array.markers.push_back(
            label);
    }


    return array;
}


void
CutProfileServer::publishVisualization(
    const motion::CutPlan& plan,
    const std::string& frame_id)
{
    visualization_pub_->publish(
        makeVisualization(
            plan,
            frame_id));
}

    void CutProfileServer::handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
        std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response){
            
            {std::lock_guard<std::mutex> lock(approval_mutex_);
            motion_approved_ = request->approve;}
            response->accepted = true;
            if(request->approve){
                response->message = "motion approved";
                approval_cv_.notify_all();
            }
            else{
                response->message = "motion not approved";
            }

        }




}


int main (int argc, char** argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions();//.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hb_robot_skills::CutProfileServer>(options);

    try{
    node->initialize();
    RCLCPP_INFO(node->get_logger(),"initalized cut profile server");
    }
    catch(std::exception &e){
        RCLCPP_FATAL(node->get_logger(),"%s",e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
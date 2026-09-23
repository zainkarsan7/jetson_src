#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "hb_robot_interfaces/srv/approve_motion.hpp"


class ApproveMotionServer: public rclcpp::Node{
    public:
    ApproveMotionServer():Node("approve_motion_server"){
        service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_motion",
            std::bind(&ApproveMotionServer::handleApproval,this,
                std::placeholders::_1,std::placeholders::_2)
        );
    };
    private:
        void handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion> request,
        std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion>response){
            
            {std::lock_guard<std::mutex> lock(approval_mutex);
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
        rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr service_;
        std::mutex approval_mutex;
        std::condition_variable approval_cv_;
        bool motion_approved_{false};
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ApproveMotionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
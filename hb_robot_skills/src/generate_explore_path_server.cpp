#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <vector>
#include "tf2_eigen/tf2_eigen.hpp"
#include "hb_robot_interfaces/srv/generate_explore_path.hpp"

using GenerateExplorePath = hb_robot_interfaces::srv::GenerateExplorePath;

class GenerateExplorePathServer : public rclcpp::Node{
    public :
        GenerateExplorePathServer():Node("generate_explore_path_server"){
            service_ = this->create_service<GenerateExplorePath>(
                "generate_explore_path",
                std::bind(&GenerateExplorePathServer::doGenerateExplorePath,
                this,std::placeholders::_1, std::placeholders::_2)
            );
        }
    private:

        std::vector<geometry_msgs::msg::Pose> generateViewpoints(const Eigen::Isometry3d &T_cam, double range_x, double range_y, int num_samples){
            std::vector<geometry_msgs::msg::Pose> viewpoints;
            const int total_pts = num_samples^2;
            viewpoints.reserve(total_pts);
            for (int ix = 0; ix<num_samples; ix++){
                
                const double rx = -range_x + (2.0*range_x * ix)/static_cast<double>(num_samples-1);

                for (int iy = 0; iy<num_samples; iy++){

                    const double ry = -range_y + (range_y*2.0 * iy)/static_cast<double>(num_samples-1);

                    Eigen::AngleAxisd Rx(rx,Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd Ry(ry,Eigen::Vector3d::UnitY());
                    Eigen::Isometry3d T_view = T_cam;
                    T_view.linear() = T_cam.linear()* Rx.toRotationMatrix() * Ry.toRotationMatrix();
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = T_view.translation().x();
                    pose.position.y = T_view.translation().y();
                    pose.position.z = T_view.translation().z();

                    Eigen::Quaterniond q (T_view.linear());
                    q.normalize();
                    pose.orientation.w = q.w();
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();

                    viewpoints.push_back(pose);



                }
            }
            return viewpoints;


        }


        void doGenerateExplorePath(
            const std::shared_ptr<GenerateExplorePath::Request> request,
            std::shared_ptr<GenerateExplorePath::Response> response
        ){
            RCLCPP_INFO(get_logger(),"Generating %d viewpoints around %.3f,%.3f,%.3f, with range %.3f,%.3f",request->number_samples^2,
            request->center_pose.position.x,
            request->center_pose.position.y,
            request->center_pose.position.z,
            request->range_x,
            request->range_y);
            Eigen::Isometry3d T_pose;
            tf2::fromMsg(request->center_pose,T_pose);
            std::vector<geometry_msgs::msg::Pose> generated_vps_ = generateViewpoints(T_pose,request->range_x,request->range_y,request->number_samples);
            response->viewpoints = generated_vps_;

            response->success = true;
            response->message = "Viewpoints Generated";
        }
        rclcpp::Service<GenerateExplorePath>::SharedPtr service_;
};

int main (int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<GenerateExplorePathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
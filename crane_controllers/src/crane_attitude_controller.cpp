#include "crane_controllers/crane_attitude_estimator.hpp"
#include "crane_controllers/crane_attitude_controller.hpp"
#include "crane_controllers/crane_attitude_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace crane_controllers{
    controller_interface::CallbackReturn CraneAttitudeController::on_init(){
        try{
            auto_declare<std::string>(
                "marker_pose_topic",
                "/Trolley/left_spreader_transform"
                
            );
            auto_declare<std::string>(
                "imu_topic",
                "/Spreader_NS_RS/Spreader_RS/imu"
            );


        }
        catch(const std::exception &e){
            RCLCPP_ERROR(get_node()->get_logger(),"failed to declare params, %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CraneAttitudeController::command_interface_configuration() const{
        return {controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::InterfaceConfiguration CraneAttitudeController::state_interface_configuration() const {
        return {controller_interface::interface_configuration_type::NONE};
    };


    controller_interface::CallbackReturn CraneAttitudeController::on_configure(const rclcpp_lifecycle::State &) {
        marker_pose_topic_ = get_node()->get_parameter("marker_pose_topic").as_string();
        imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
        CraneAttitudeState initial_state;
        state_buffer_.writeFromNonRT(initial_state);

        // make subscriptions 
        marker_pose_sub = get_node()->create_subscription<geometry_msgs::msg::TransformStamped>(marker_pose_topic_,
            rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TransformStamped::SharedPtr pose_msg){
                estimator_.update_marker_pose(*pose_msg);
                // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),2000,"marker updated %d",
                // estimator_.state().pose_valid);
                state_buffer_.writeFromNonRT(estimator_.state());
            });


        imu_sub = get_node()->create_subscription<sensor_msgs::msg::Imu>(imu_topic_,
        rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr imu_msg){
        estimator_.update_imu(*imu_msg);
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1000,
        // "imu update %d",estimator_.state().imu_valid);
        state_buffer_.writeFromNonRT(estimator_.state());
    });




    RCLCPP_INFO(get_node()->get_logger(),"controller configured");
    return controller_interface::CallbackReturn::SUCCESS;

    };


    controller_interface::CallbackReturn CraneAttitudeController::on_activate(const rclcpp_lifecycle::State &){

        RCLCPP_INFO(get_node()->get_logger(),"nothing claimed, controller activated");
        return controller_interface::CallbackReturn::SUCCESS;

    };

    controller_interface::CallbackReturn CraneAttitudeController::on_deactivate(const rclcpp_lifecycle::State &){

        RCLCPP_INFO(get_node()->get_logger(),"nothing claimed, controller deactivated");
        return controller_interface::CallbackReturn::SUCCESS;

    };

    controller_interface::return_type CraneAttitudeController::update(const rclcpp::Time &, const rclcpp::Duration &){

        CraneAttitudeState * current_state_ptr= state_buffer_.readFromRT();

        if(current_state_ptr==nullptr){

            RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
        *get_node()->get_clock(),2000,"state ptr is null");

            return controller_interface::return_type::OK;
        }

        if(!current_state_ptr->imu_valid || !current_state_ptr->pose_valid){
            RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
        *get_node()->get_clock(),2000,"state subscriptions imu %d pose %d",
        current_state_ptr->imu_valid,current_state_ptr->pose_valid);

            return controller_interface::return_type::OK;
        }

        // reading thetas and omegas here

        const double roll = current_state_ptr->rpy.x();
        const double pitch = current_state_ptr->rpy.y();
        const double yaw = current_state_ptr->rpy.z();

        const double omega_x = current_state_ptr->omega.x();
        const double omega_y = current_state_ptr->omega.y();
        const double omega_z = current_state_ptr->omega.z();

        RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),5000,
        "RPY: %.3f, %.3f, %.3f || Omega:  %.3f, %.3f, %.3f ",
        roll,pitch,yaw,omega_x,omega_y,omega_z);

        return controller_interface::return_type::OK;

    }



}

PLUGINLIB_EXPORT_CLASS(
  crane_controllers::CraneAttitudeController,
  controller_interface::ControllerInterface
)

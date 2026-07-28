#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace crane_controllers{
    class CraneHomingController:public controller_interface::ControllerInterface{
        public:
         controller_interface::CallbackReturn on_init() override{
            try{
                home_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/start_homing",[this](
                    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
                        if (home_requested_.exchange(true)){
                            response->success = false;
                            response->message = "request to home pending";
                            
                            return;
                        }
                        home_requested_.store(true);
                        response->success = true;
                        response->message = "homing request accepted";
                    });
                
                
            }
            catch (const std::exception &exception){
                RCLCPP_ERROR(get_node()->get_logger(),"failed to init homing controller %s",
            exception.what());
                return controller_interface::CallbackReturn::ERROR;
                
                
                }
            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::InterfaceConfiguration command_interface_configuration() const override{
            controller_interface::InterfaceConfiguration configuration;
            configuration.type=controller_interface::interface_configuration_type::INDIVIDUAL;
            configuration.names={"crane/home"};
            return configuration;
        }
        controller_interface::InterfaceConfiguration state_interface_configuration()const override{
            controller_interface::InterfaceConfiguration configuration;
            configuration.type=controller_interface::interface_configuration_type::INDIVIDUAL;
            configuration.names={"crane/homing", "crane/homed", "crane/home_failed"};
            return configuration;
        }
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override{
            home_requested_.store(false);
            pulse_active_ = false;         //just setup stuff 
            return controller_interface::CallbackReturn::SUCCESS;
        }
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override{
            
            if (command_interfaces_.size()!=0){
                const auto full_name = command_interfaces_[0].get_full_name();
                RCLCPP_INFO(get_node()->get_logger(),"recieved %s", full_name.c_str());
            }
            
            command_interfaces_[0].set_value(0.0);   
            home_requested_.store(false);
            pulse_active_ = false;   
            
            return controller_interface::CallbackReturn::SUCCESS;
        }
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override{
            command_interfaces_[0].set_value(0.0);
            home_requested_.store(false);
            pulse_active_ = false;    

            return controller_interface::CallbackReturn::SUCCESS;
        }
        controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &){
            
            if (home_requested_.exchange(false)){
                command_interfaces_[0].set_value(1.0);
                pulse_active_= true;
            }
            else{
                command_interfaces_[0].set_value(0.0);
                pulse_active_ = false;
            }
            return controller_interface::return_type::OK;
        }
        private: 
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
        std::atomic_bool home_requested_{false};
        bool pulse_active_{false};
    };
}


PLUGINLIB_EXPORT_CLASS(
  crane_controllers::CraneHomingController,
  controller_interface::ControllerInterface
)

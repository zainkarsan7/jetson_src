#ifndef CRANE_HARDWARE__CRANE_HARDWARE_HPP_
#define CRANE_HARDWARE__CRANE_HARDWARE_HPP_


#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace crane_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CraneHardware : public hardware_interface::SystemInterface
    {
        public:
        enum JointIndex{
            SWIVEL = 0,
            LEFT = 1,
            RIGHT = 2
        };

        
        // extending the system interface class from hardware interface
        // all these functions are part of the lifecycle for hardware
        // theres state and command interfaces
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_configure (const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        
        //also read and write stuff
        hardware_interface::return_type read( const rclcpp::Time & time, const rclcpp::Duration & period) override; 
        hardware_interface::return_type write( const rclcpp::Time & time, const rclcpp::Duration & period) override;
        private:
        // serial stuff
        int serial_fd_{-1};
        std::string port_;
        int baud_rate_{115200};
        // motor hardware vector 
        std::vector<double> hw_positions_;
        std::vector<double> hw_commands_;
        std::unordered_map<JointIndex, double> steps_per_unit_;
        // serial functions open and close
        bool open_serial();
        void close_serial();

        bool write_line(const std::string & line);
        bool read_line(std::string & line);

        long unit_to_steps(double value, JointIndex joint) const;
        double steps_to_unit(long steps, JointIndex joint) const;

    };
}

#endif

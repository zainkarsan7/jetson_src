#include "crane_hardware/crane_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"


namespace crane_hardware
{
   hardware_interface::CallbackReturn CraneHardware::on_init(
    const hardware_interface::HardwareInfo & info){
        if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
            {
                return hardware_interface::CallbackReturn::ERROR;

            }
        if (info_.joints.size()!=3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CraneHardware"),"Expected 3 joints got %zu",info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.gpios.size()<1){
            return hardware_interface::CallbackReturn::ERROR;
        }


        const auto & gpio = info_.gpios[0];

    port_ = info_.hardware_parameters.at("port");
    //if theres a baud rate parameter, plug it into the private variable
    if(info_.hardware_parameters.count("baud_rate")){
        baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    }
    //fill the vectors generic
    hw_positions_.assign(3,0.0);
    hw_commands_.assign(3,0.0);
    steps_per_unit_ = {{JointIndex::SWIVEL, 1000.0},
                        {JointIndex::LEFT, 10000.0},
                        {JointIndex::RIGHT, 10000.0}
                    };

    if(info_.hardware_parameters.count("swivel_steps_per_unit")){
        steps_per_unit_.at(JointIndex::SWIVEL)=std::stod(info_.hardware_parameters.at("swivel_steps_per_unit"));
    }

    if(info_.hardware_parameters.count("left_steps_per_unit")){
        steps_per_unit_.at(JointIndex::LEFT)=std::stod(info_.hardware_parameters.at("left_steps_per_unit"));
    }

    if(info_.hardware_parameters.count("right_steps_per_unit")){
        steps_per_unit_.at(JointIndex::RIGHT)=std::stod(info_.hardware_parameters.at("right_steps_per_unit"));
    }

    for (const auto & joint : info_.joints){
        if(joint.command_interfaces.size()!=1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CraneHardware"),
                "joint '%s' must have 1 position command interface",
                joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        

        if(joint.state_interfaces.size()!=1 || 
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CraneHardware"),
                "joint '%s' must have 1 position state interface",
                joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

    }
    return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn CraneHardware::on_configure(
        const rclcpp_lifecycle::State &)
        {
            if (!open_serial()){
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(
                rclcpp::get_logger("CraneHardware"),
                "connected MCU on '%s'",
                port_.c_str()
            );
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    hardware_interface::CallbackReturn CraneHardware::on_activate(
        const rclcpp_lifecycle::State &){
            hw_commands_ = hw_positions_;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn CraneHardware::on_deactivate(
        const rclcpp_lifecycle::State &){
            close_serial();
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    std::vector<hardware_interface::StateInterface>CraneHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i=0; i<info_.joints.size(); ++i){
            //construct the state interface directly
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]
            );
        }

        interfaces.emplace_back("crane","homing", &homing_state_);
        interfaces.emplace_back("crane","homed", &homed_state_);
        interfaces.emplace_back("crane","home_failed", &homing_failed_state_);

            return interfaces;
        }

    std::vector<hardware_interface::CommandInterface>CraneHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i=0; i<info_.joints.size(); ++i){
            //construct the command interface directly
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i]
            );
        }

        interfaces.emplace_back("crane","home", &home_command_);
        return interfaces;
    }
    hardware_interface::return_type CraneHardware::read(
        const rclcpp::Time &,
        const rclcpp::Duration &){
        
        std::string line;

        while(read_line(line)){
                        
            long s = 0;
            long l = 0;
            long r = 0;
            
            if (std::sscanf(line.c_str(),"S %ld L %ld R %ld", &s, &l, &r)==3){
                hw_positions_[0] = steps_to_unit(s, JointIndex::SWIVEL);
                hw_positions_[1] = steps_to_unit(l, JointIndex::LEFT);
                hw_positions_[2] = steps_to_unit(r, JointIndex::RIGHT);
            }
            else if (line == "HOME_STARTED"){
                ard_is_homing_ = true;
                homing_state_ = 1.0;
                homed_state_ = 0.0;
                homing_failed_state_ = 0.0;
            }
            else if(line== "HOME_DONE"){
                ard_is_homing_ = false;
                homing_state_ = 0.0;
                homed_state_ = 1.0;
                homing_failed_state_ = 0.0;
            }
            else if (line== "HOME_FAILED"){
                ard_is_homing_ = false;
                homing_state_ = 0.0;
                homed_state_ = 0.0;
                homing_failed_state_ = 1.0;
            }
            //other arduino messages could be parsed here
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CraneHardware::write(
        const rclcpp::Time &,
        const rclcpp::Duration &){

            const bool home_requested = home_command_ > 0.5;
            const bool home_rising_edge = home_requested && !prev_home_com_;
            prev_home_com_ = home_requested;
            if (home_rising_edge){
                if (!write_line("HOME\n")){
                    return hardware_interface::return_type::ERROR;
                }

                return hardware_interface::return_type::OK;
            }
            if (ard_is_homing_){
                return hardware_interface::return_type::OK;
            }
            const long s = unit_to_steps(hw_commands_[0],JointIndex::SWIVEL);
            const long l = unit_to_steps(hw_commands_[1],JointIndex::LEFT);
            const long r = unit_to_steps(hw_commands_[2],JointIndex::RIGHT);
            
             if (s == last_sent_s_ &&
                l == last_sent_l_ &&
                r == last_sent_r_) {
                return hardware_interface::return_type::OK;
            }


            std::ostringstream command;
            command<<"SETPOS "<< s<<" "<<l<<" "<<r<<"\n";
            
            RCLCPP_INFO(rclcpp::get_logger("CraneHardware"),
        "Commands: %.4f %.4f %.4f -> SETPOS %ld %ld %ld",
        hw_commands_[0],hw_commands_[1],hw_commands_[2],s,l,r);


            if (!write_line(command.str())){
                return hardware_interface::return_type::ERROR;
            }



            last_sent_s_ = s;
            last_sent_l_ = l;
            last_sent_r_ = r;
            return hardware_interface::return_type::OK;
        }
    
    long CraneHardware::unit_to_steps(double value, JointIndex joint_index) const{
        return static_cast<long>(std::llround(value * steps_per_unit_.at(joint_index)));
    }

    double CraneHardware::steps_to_unit(long steps, JointIndex joint_index) const{
        return static_cast<double>(steps)/steps_per_unit_.at(joint_index);
    }

    bool CraneHardware::open_serial(){
        serial_fd_ = open(port_.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0){
            RCLCPP_ERROR(rclcpp::get_logger("CraneHardware"), "couldnt open serial at %s:%s",port_.c_str(),std::strerror(errno));

            return false;
        }


        termios tty{};
        if (tcgetattr(serial_fd_,&tty)!=0){
            RCLCPP_ERROR(rclcpp::get_logger("CraneHardware"), "tcgetattr failed");
            close_serial();
            return false;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CraneHardware"), "tcsetattr failed");
            close_serial();
            return false;
        }

        return true;
    }

    void CraneHardware::close_serial()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            
        }
    }

    bool CraneHardware::write_line(const std::string & line){
        if (serial_fd_<0){
            return false;
        }
        

        std::size_t total_written = 0;

        while (total_written < line.size()) {
            const ssize_t result = ::write(
            serial_fd_,
            line.data() + total_written,
            line.size() - total_written);

            if (result > 0) {
            total_written += static_cast<std::size_t>(result);
            continue;
            }

            if (result < 0 && errno == EINTR) {
            continue;
            }

            if (result < 0 && (
                errno == EAGAIN ||
                errno == EWOULDBLOCK)) {
            return false;
            }

            RCLCPP_ERROR(
            rclcpp::get_logger("CraneHardware"),
            "Serial write failed: %s",
            std::strerror(errno));

            return false;
        }

        return true;

    }

    bool CraneHardware::read_line(std::string & line){
        static std::string buffer;
        if (serial_fd_<0){
                    return false;
                }

        char c;
        while (::read(serial_fd_, &c, 1)==1){
            if (c == '\n') {
            line = buffer;
            buffer.clear();
            return true;
            }
            if (c != '\r') {
            buffer += c;
            }
        }
        return false;
    }

} //namespace crane_hardware

PLUGINLIB_EXPORT_CLASS(
    crane_hardware::CraneHardware,
    hardware_interface::SystemInterface
)
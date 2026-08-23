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
        if (info_.joints.size()!=9)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CraneHardware"),"Expected 9 joints got %zu",info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.gpios.size()<3){
            return hardware_interface::CallbackReturn::ERROR;
        }


      

    port_ = info_.hardware_parameters.at("port");
    //if theres a baud rate parameter, plug it into the private variable
    if(info_.hardware_parameters.count("baud_rate")){
        baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    }
    //fill the vectors generic
    hw_positions_.assign(5,0.0);
    hw_velocities_.assign(4,0.0);
    hw_pos_commands_.assign(5,0.0);
    hw_vel_commands_.assign(2,0.0);
    

    steps_per_unit_ = {{JointIndex::SWIVEL, 1000.0},
                        {JointIndex::LEFT, 10000.0},
                        {JointIndex::RIGHT, 10000.0},
                        {JointIndex::G0, 50.0},
                        {JointIndex::G1, 50.0}
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

    if(info_.hardware_parameters.count("g0_steps_per_unit")){
        steps_per_unit_.at(JointIndex::G0)=std::stod(info_.hardware_parameters.at("g0_steps_per_unit"));
    }

    if(info_.hardware_parameters.count("g1_steps_per_unit")){
        steps_per_unit_.at(JointIndex::G1)=std::stod(info_.hardware_parameters.at("g1_steps_per_unit"));
    }

    for (const auto & joint : info_.joints){
        for (const auto & com_int : joint.command_interfaces){
            if (com_int.name != hardware_interface::HW_IF_POSITION && com_int.name != hardware_interface::HW_IF_VELOCITY){
                RCLCPP_ERROR(
                rclcpp::get_logger("CraneHardware"),
                "joint '%s' must have 1 pos or vel command interface",
                joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
            }
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
            hw_pos_commands_ = hw_positions_;
            hw_vel_commands_ = hw_velocities_;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn CraneHardware::on_deactivate(
        const rclcpp_lifecycle::State &){
            close_serial();
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    std::vector<hardware_interface::StateInterface>CraneHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i=0; i<pos_joint_names_.size(); ++i){
            //populate the pos state interfaces directly
            interfaces.emplace_back(
                pos_joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]
            );
        }
        for (size_t i=0; i<vel_joint_names_.size(); ++i){
            //populate the vel state interfaces directly
            interfaces.emplace_back(
                vel_joint_names_[i],
                hardware_interface::HW_IF_VELOCITY,
                &hw_velocities_[i]
            );
        }

        interfaces.emplace_back("crane","homing", &homing_state_);
        interfaces.emplace_back("crane","homed", &homed_state_);
        interfaces.emplace_back("crane","home_failed", &homing_failed_state_);

            return interfaces;
        }

    std::vector<hardware_interface::CommandInterface>CraneHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i=0; i<pos_joint_names_.size(); ++i){
            //populate the pos state interfaces directly
            interfaces.emplace_back(
                pos_joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_pos_commands_[i]
            );
        }
        interfaces.emplace_back("G0_wheels",hardware_interface::HW_IF_VELOCITY,&hw_vel_commands_[0]);
        interfaces.emplace_back("G1_wheels",hardware_interface::HW_IF_VELOCITY,&hw_vel_commands_[1]);
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
            long g0 = 0;
            long g1 = 0;
            
            long w0_rpm = 0;
            long w1_rpm = 0;
            long w2_rpm = 0;
            long w3_rpm = 0;
            
            if (std::sscanf(line.c_str(),"S %ld L %ld R %ld G0 %ld G1 %ld", &s, &l, &r, &g0, &g1)==5){
                hw_positions_[0] = steps_to_unit(s, JointIndex::SWIVEL);
                hw_positions_[1] = steps_to_unit(l, JointIndex::LEFT);
                hw_positions_[2] = steps_to_unit(r, JointIndex::RIGHT);
                hw_positions_[3] = steps_to_unit(g0, JointIndex::G0);
                hw_positions_[4] = steps_to_unit(g1, JointIndex::G1);
            }

            else if (std::sscanf(line.c_str(),"W0 %ld W1 %ld W2 %ld W3 %ld", &w0_rpm,&w1_rpm,&w2_rpm,&w3_rpm)==4){
                hw_velocities_[0] = rpm_to_rad_sec(w0_rpm);
                hw_velocities_[1] = rpm_to_rad_sec(w1_rpm);
                hw_velocities_[2] = rpm_to_rad_sec(w2_rpm);
                hw_velocities_[3] = rpm_to_rad_sec(w3_rpm);
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
            const long s = unit_to_steps(hw_pos_commands_[0],JointIndex::SWIVEL);
            const long l = unit_to_steps(hw_pos_commands_[1],JointIndex::LEFT);
            const long r = unit_to_steps(hw_pos_commands_[2],JointIndex::RIGHT);
            const long g0 = unit_to_steps(hw_pos_commands_[3],JointIndex::G0);
            const long g1 = unit_to_steps(hw_pos_commands_[4],JointIndex::G1);
            
            const bool pos_changed = (s != last_sent_s_ ||
                l != last_sent_l_ ||
                r != last_sent_r_ || 
                g0 != last_sent_g0_ || 
                g1 != last_sent_g1_            
            ) ;


            if (pos_changed){
                std::ostringstream command;
            command<<"SETPOS "<< s<<" "<<l<<" "<<r<<" "<<g0<<" "<<g1<<"\n";
            RCLCPP_INFO(rclcpp::get_logger("CraneHardware"),
        "Commands: %.4f %.4f %.4f %.4f %.4f -> SETPOS %ld %ld %ld %ld %ld",
        hw_pos_commands_[0],hw_pos_commands_[1],hw_pos_commands_[2],hw_pos_commands_[3],hw_pos_commands_[4],s,l,r,g0,g1);


            if (!write_line(command.str())){
                return hardware_interface::return_type::ERROR;
            }

            last_sent_s_ = s;
            last_sent_l_ = l;
            last_sent_r_ = r;
            last_sent_g0_ = g0;
            last_sent_g1_ = g1;
        
        
        }

            const double g0_wheel_rpm = std::lround(rad_sec_to_rpm(hw_vel_commands_[0]));
            const double g1_wheel_rpm = std::lround(rad_sec_to_rpm(hw_vel_commands_[1]));

            if (g0_wheel_rpm!=last_sent_g0_wheels_){
                std::ostringstream wheel_0_command;

                wheel_0_command<<"GIM 0 RPM "<<g0_wheel_rpm<<"\n";

                if (!write_line(wheel_0_command.str())){
                    return hardware_interface::return_type::ERROR;
                }
                last_sent_g0_wheels_ = g0_wheel_rpm;

            } 
            if (g1_wheel_rpm!=last_sent_g1_wheels_){
                std::ostringstream wheel_1_command;

                wheel_1_command<<"GIM 1 RPM "<<g1_wheel_rpm<<"\n";

                if (!write_line(wheel_1_command.str())){
                    return hardware_interface::return_type::ERROR;
                }
                last_sent_g1_wheels_ = g1_wheel_rpm;

            }

            return hardware_interface::return_type::OK;
        
    }
    
    long CraneHardware::unit_to_steps(double value, JointIndex joint_index) const{
        return static_cast<long>(std::llround(value * steps_per_unit_.at(joint_index)));
    }

    double CraneHardware::steps_to_unit(long steps, JointIndex joint_index) const{
        return static_cast<double>(steps)/steps_per_unit_.at(joint_index);
    }

    double CraneHardware::rad_sec_to_rpm(double rad_sec) const{
        return static_cast<double>(rad_sec * 60.0)/ (2 * M_PI);
    }

    double CraneHardware::rpm_to_rad_sec(long rpm)const{
        return static_cast<double>(rpm * 2 * M_PI)/60.0;
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
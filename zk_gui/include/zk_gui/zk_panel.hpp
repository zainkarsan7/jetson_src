#ifndef ZK_GUI__ZK_PANEL_HPP_
#define ZK_GUI__ZK_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace zk_gui
{
    class zk_panel: public rviz_common::Panel
    {
        Q_OBJECT
        public:
            explicit zk_panel(QWidget*parent=0);
            ~zk_panel() override;
            void onInitialize() override;
        protected:
            std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
            void topicCallback(const sensor_msgs::msg::JointState& msg);
            QLabel *label_;
            QPushButton * button_;
        private Q_SLOTS:
            void buttonActivated();

    };
}
#endif
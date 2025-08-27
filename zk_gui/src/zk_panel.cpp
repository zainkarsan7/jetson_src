#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <zk_gui/zk_panel.hpp>

namespace zk_gui{

    zk_panel::zk_panel(QWidget*parent):Panel(parent){
        const auto layout=new QVBoxLayout(this);
        label_=new QLabel("[no data]");
        button_=new QPushButton("GO!");
        layout->addWidget(label_);
        layout->addWidget(button_);

        QObject::connect(button_,&QPushButton::released,this,&zk_panel::buttonActivated);


    }
    zk_panel::~zk_panel() = default;

    void zk_panel::onInitialize(){
        node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
        rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
        pub_ = node->create_publisher<std_msgs::msg::String>("/out",10);
        sub_ = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&zk_panel::topicCallback,this,std::placeholders::_1));
    }

    void zk_panel::topicCallback(const sensor_msgs::msg::JointState& msg){
        
        std::string label_string = "J0: " + std::to_string(msg.position[0])
        +" J1: " + std::to_string(msg.position[1])
        +" J2: " + std::to_string(msg.position[2])
        +" J3: " + std::to_string(msg.position[3])
        +" J4: " + std::to_string(msg.position[4])
        +" J5: " + std::to_string(msg.position[5]);
        
        label_->setText(QString(label_string.c_str()));
    }

    void zk_panel::buttonActivated(){
        auto message = std_msgs::msg::String();
        message.data = "Button clicked!";
        pub_->publish(message);
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(zk_gui::zk_panel,rviz_common::Panel)
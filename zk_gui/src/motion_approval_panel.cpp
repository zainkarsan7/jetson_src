#include <chrono>
#include <future>
#include <memory>

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <hb_robot_interfaces/srv/approve_motion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace zk_gui
{
namespace
{
using Approval = hb_robot_interfaces::srv::ApproveMotion;
using namespace std::chrono_literals;

class ApprovalRow : public QWidget
{
public:
  ApprovalRow(rclcpp::Node & node, const QString & title, const char * service)
  : client_(node.create_client<Approval>(service))
  {
    auto * layout = new QGridLayout(this);
    auto * label = new QLabel(title);
    label->setToolTip(QString::fromUtf8(client_->get_service_name()));
    layout->addWidget(label, 0, 0);
    layout->addWidget(approve_, 0, 1);
    layout->addWidget(reject_, 0, 2);
    layout->addWidget(status_, 1, 0, 1, 3);
    connect(approve_, &QPushButton::clicked, this, [this] { send(true); });
    connect(reject_, &QPushButton::clicked, this, [this] { send(false); });
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(250);
    refresh();
  }

private:
  void send(bool approve)
  {
    if (pending_.valid() || !client_->service_is_ready()) {
      refresh();
      return;
    }
    auto request = std::make_shared<Approval::Request>();
    request->approve = approve;
    decision_ = approve ? "Approval" : "Rejection";
    try {
      auto sent = client_->async_send_request(request);
      request_id_ = sent.request_id;
      pending_ = std::move(sent.future);
      deadline_ = std::chrono::steady_clock::now() + 5s;
      result_ = decision_ + " sent";
    } catch (const std::exception & e) {
      result_ = QString("Send failed: ") + e.what();
    }
    refresh();
  }

  void refresh()
  {
    if (pending_.valid()) {
      if (pending_.wait_for(0s) == std::future_status::ready) {
        try {
          result_ = decision_ + (pending_.get()->accepted ? " accepted" : " declined");
        } catch (const std::exception & e) {
          result_ = QString("Reply failed: ") + e.what();
        }
      } else if (std::chrono::steady_clock::now() >= deadline_) {
        client_->remove_pending_request(request_id_);
        pending_ = {};
        result_ = "No reply; outcome unknown";
      }
    }
    const bool online = client_->service_is_ready();
    approve_->setEnabled(online && !pending_.valid());
    reject_->setEnabled(online && !pending_.valid());
    const QString availability = online ? "Service online" : "Service offline";
    status_->setText(result_.isEmpty() ? availability : availability + " | " + result_);
  }

  rclcpp::Client<Approval>::SharedPtr client_;
  std::future<Approval::Response::SharedPtr> pending_;
  int64_t request_id_{0};
  std::chrono::steady_clock::time_point deadline_;
  QString decision_;
  QString result_;
  QPushButton * approve_{new QPushButton("Approve")};
  QPushButton * reject_{new QPushButton("Reject")};
  QLabel * status_{new QLabel};
};
}  // namespace

class MotionApprovalPanel : public rviz_common::Panel
{
public:
  void onInitialize() override
  {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    auto * layout = new QVBoxLayout(this);
    layout->addWidget(new ApprovalRow(*node, "Scene inspection", "approve_motion"));
    layout->addWidget(new ApprovalRow(*node, "Profile inspection", "approve_profile_motion"));
    layout->addWidget(new ApprovalRow(*node, "Cut motion", "approve_cut_motion"));
    layout->addStretch();
  }
};
}  // namespace zk_gui

PLUGINLIB_EXPORT_CLASS(zk_gui::MotionApprovalPanel, rviz_common::Panel)

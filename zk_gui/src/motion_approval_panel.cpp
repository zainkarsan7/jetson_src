
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <hb_robot_interfaces/action/return_to_inspection_home.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_interfaces/srv/approve_motion.hpp>
#include <hb_robot_interfaces/srv/clear_torch_fault.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace zk_gui
{

namespace
{

using namespace std::chrono_literals;

using Clock = std::chrono::steady_clock;

using Approval =
    hb_robot_interfaces::srv::ApproveMotion;

using ClearTorch =
    hb_robot_interfaces::srv::ClearTorchFault;

using Trigger =
    std_srvs::srv::Trigger;

using TorchStatus =
    hb_robot_interfaces::msg::TorchStatus;

using TaskState =
    hb_robot_interfaces::msg::TaskState;


// ============================================================
// Return to inspection home
// ============================================================

class HomeRow : public QWidget
{
  using Action =
      hb_robot_interfaces::action::ReturnToInspectionHome;

  using GoalHandle =
      rclcpp_action::ClientGoalHandle<Action>;

public:
  explicit HomeRow(
      const rclcpp::Node::SharedPtr& node)
    : QWidget(nullptr),
      client_(
          rclcpp_action::create_client<Action>(
              node,
              "return_to_inspection_home"))
  {
    auto* layout = new QVBoxLayout(this);

    layout->addWidget(button_);
    layout->addWidget(status_);

    connect(
        button_,
        &QPushButton::clicked,
        this,
        [this]()
        {
          send();
        });

    auto* timer = new QTimer(this);

    connect(
        timer,
        &QTimer::timeout,
        this,
        [this]()
        {
          refresh();
        });

    timer->start(250);

    refresh();
  }

private:
  void send()
  {
    if (pending_.valid() ||
        outcome_.valid() ||
        !client_->action_server_is_ready())
    {
      refresh();
      return;
    }

    button_->setEnabled(false);

    try
    {
      pending_ = client_->async_send_goal(
          Action::Goal{});

      status_->setText(
          "Requesting inspection home");
    }
    catch (const std::exception& e)
    {
      status_->setText(e.what());
    }

    refresh();
  }

  void refresh()
  {
    try
    {
      if (pending_.valid() &&
          pending_.wait_for(0s) ==
              std::future_status::ready)
      {
        auto goal = pending_.get();
        pending_ = {};

        if (goal)
        {
          outcome_ = client_->async_get_result(goal);

          status_->setText(
              "Returning to inspection home");
        }
        else
        {
          status_->setText(
              "Home request rejected");
        }
      }

      if (outcome_.valid() &&
          outcome_.wait_for(0s) ==
              std::future_status::ready)
      {
        auto reply = outcome_.get();
        outcome_ = {};

        if (reply.result)
        {
          status_->setText(
              QString::fromStdString(
                  reply.result->message));
        }
        else
        {
          status_->setText(
              "Home action ended without a result");
        }
      }
    }
    catch (const std::exception& e)
    {
      pending_ = {};
      outcome_ = {};

      status_->setText(e.what());
    }

    button_->setEnabled(
        !pending_.valid() &&
        !outcome_.valid() &&
        client_->action_server_is_ready());
  }

  rclcpp_action::Client<Action>::SharedPtr client_;

  std::shared_future<GoalHandle::SharedPtr> pending_;

  std::shared_future<GoalHandle::WrappedResult> outcome_;

  QPushButton* button_{
      new QPushButton(
          "Return to inspection home")};

  QLabel* status_{
      new QLabel};
};


// ============================================================
// Motion approval / rejection
// ============================================================

class ApprovalRow : public QWidget
{
public:
  ApprovalRow(
      rclcpp::Node& node,
      const QString& title,
      const char* service)
    : QWidget(nullptr),
      client_(
          node.create_client<Approval>(service))
  {
    auto* layout = new QGridLayout(this);

    auto* label = new QLabel(title);

    label->setToolTip(
        QString::fromUtf8(
            client_->get_service_name()));

    layout->addWidget(label, 0, 0);
    layout->addWidget(approve_, 0, 1);
    layout->addWidget(reject_, 0, 2);
    layout->addWidget(status_, 1, 0, 1, 3);

    connect(
        approve_,
        &QPushButton::clicked,
        this,
        [this]()
        {
          send(true);
        });

    connect(
        reject_,
        &QPushButton::clicked,
        this,
        [this]()
        {
          send(false);
        });

    auto* timer = new QTimer(this);

    connect(
        timer,
        &QTimer::timeout,
        this,
        [this]()
        {
          refresh();
        });

    timer->start(250);

    refresh();
  }

private:
  void send(bool approve)
  {
    if (pending_.valid() ||
        !client_->service_is_ready())
    {
      refresh();
      return;
    }

    auto request =
        std::make_shared<Approval::Request>();

    request->approve = approve;

    decision_ = approve
        ? "Approval"
        : "Rejection";

    try
    {
      auto sent =
          client_->async_send_request(request);

      request_id_ = sent.request_id;
      pending_ = std::move(sent.future);

      deadline_ = Clock::now() + 5s;

      result_ = decision_ + " sent";
    }
    catch (const std::exception& e)
    {
      result_ =
          QString("Send failed: ") + e.what();
    }

    refresh();
  }

  void refresh()
  {
    if (pending_.valid())
    {
      if (pending_.wait_for(0s) ==
          std::future_status::ready)
      {
        try
        {
          const auto response = pending_.get();

          pending_ = {};

          result_ = decision_ +
              (response && response->accepted
                  ? " accepted"
                  : " declined");
        }
        catch (const std::exception& e)
        {
          pending_ = {};

          result_ =
              QString("Reply failed: ") + e.what();
        }
      }
      else if (Clock::now() >= deadline_)
      {
        client_->remove_pending_request(
            request_id_);

        pending_ = {};

        result_ =
            "No reply; outcome unknown";
      }
    }

    const bool online =
        client_->service_is_ready();

    approve_->setEnabled(
        online && !pending_.valid());

    reject_->setEnabled(
        online && !pending_.valid());

    const QString availability =
        online
            ? "Service online"
            : "Service offline";

    status_->setText(
        result_.isEmpty()
            ? availability
            : availability + " | " + result_);
  }

  rclcpp::Client<Approval>::SharedPtr client_;

  std::shared_future<
      Approval::Response::SharedPtr> pending_;

  int64_t request_id_{0};

  Clock::time_point deadline_{};

  QString decision_;
  QString result_;

  QPushButton* approve_{
      new QPushButton("Approve")};

  QPushButton* reject_{
      new QPushButton("Reject")};

  QLabel* status_{
      new QLabel};
};


// ============================================================
// Torch fault status and recovery
// ============================================================

class TorchFaultRow : public QGroupBox
{
public:
  explicit TorchFaultRow(
      const rclcpp::Node::SharedPtr& node)
    : QGroupBox("Torch fault recovery"),
      torch_client_(
          node->create_client<ClearTorch>(
              "/cut_profile/clear_torch_fault")),
      process_client_(
          node->create_client<Trigger>(
              "/task_state/clear_process_fault"))
  {
    auto* layout = new QVBoxLayout(this);

    status_->setReadOnly(true);
    status_->setMinimumHeight(140);

    result_->setWordWrap(true);

    clear_button_->setToolTip(
        "Clear the torch controller fault first, "
        "then the TaskState process fault latch. "
        "Does not authorize motion.");

    layout->addWidget(status_);
    layout->addWidget(clear_button_);
    layout->addWidget(result_);

    // Live torch telemetry.
    torch_sub_ =
        node->create_subscription<TorchStatus>(
            "/cut_profile/torch_status",
            rclcpp::QoS(10),
            [this](
                TorchStatus::ConstSharedPtr msg)
            {
              std::lock_guard<std::mutex> lock(
                  mutex_);

              torch_ = *msg;
              torch_received_ = Clock::now();
            });

    // TaskState uses transient-local QoS.
    task_sub_ =
        node->create_subscription<TaskState>(
            "/task_state",
            rclcpp::QoS(1)
                .reliable()
                .transient_local(),
            [this](
                TaskState::ConstSharedPtr msg)
            {
              std::lock_guard<std::mutex> lock(
                  mutex_);

              task_ = *msg;
              task_received_ = Clock::now();
            });

    connect(
        clear_button_,
        &QPushButton::clicked,
        this,
        [this]()
        {
          beginClear();
        });

    auto* timer = new QTimer(this);

    connect(
        timer,
        &QTimer::timeout,
        this,
        [this]()
        {
          refresh();
        });

    timer->start(250);

    refresh();
  }

private:
  enum class Phase
  {
    Idle,
    ClearingTorch,
    WaitingForTorchStatus,
    ClearingProcess,
    AwaitingConfirmation
  };

  struct Snapshot
  {
    std::optional<TorchStatus> torch;
    std::optional<TaskState> task;

    Clock::time_point torch_received{};
    Clock::time_point task_received{};

    bool torch_fresh{false};
    bool task_fresh{false};
  };

  Snapshot snapshot() 
  {
    Snapshot data;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      data.torch = torch_;
      data.task = task_;

      data.torch_received = torch_received_;
      data.task_received = task_received_;
    }

    const auto now = Clock::now();

    data.torch_fresh =
        data.torch.has_value() &&
        now - data.torch_received < 500ms;

    data.task_fresh =
        data.task.has_value() &&
        now - data.task_received < 500ms;

    return data;
  }

  bool safeToRequestClear(
      const Snapshot& data) const
  {
    if (!data.torch_fresh ||
        !data.task_fresh)
    {
      return false;
    }

    const auto& torch = *data.torch;
    const auto& task = *data.task;

    return
        torch.io_connected &&
        !torch.command_on &&
        !torch.output_on &&
        !torch.transfer_detected &&
        task.supervision_valid &&
        task.imu_stable &&
        !task.motion_fault_active;
  }

  void beginClear()
  {
    if (phase_ != Phase::Idle)
    {
      return;
    }

    const auto data = snapshot();

    if (!safeToRequestClear(data))
    {
      result_->setText(
          "Clearance blocked: telemetry stale, "
          "torch not confirmed OFF, "
          "or supervision/motion state invalid.");

      return;
    }

    if (!data.torch->fault &&
        !data.task->process_fault_active)
    {
      result_->setText(
          "No torch or process fault is active.");

      return;
    }

    if (!torch_client_->service_is_ready() ||
        !process_client_->service_is_ready())
    {
      result_->setText(
          "Fault clearance services unavailable.");

      return;
    }

    const auto answer =
        QMessageBox::question(
            this,
            "Confirm fault clearance",
            "Confirm the cutting action has stopped, "
            "the robot is stationary, and the torch "
            "is physically safe.\n\n"
            "Clear the torch fault and process "
            "fault latches?\n\n"
            "This does not authorize robot motion.",
            QMessageBox::Yes |
                QMessageBox::No,
            QMessageBox::No);

    if (answer != QMessageBox::Yes)
    {
      return;
    }

    // Recheck after the confirmation dialog.
    const auto current = snapshot();

    if (!safeToRequestClear(current))
    {
      result_->setText(
          "Clearance canceled: state changed "
          "during operator confirmation.");

      return;
    }

    // If the torch is already fault-free, skip directly
    // to clearing the process latch.
    if (!current.torch->fault)
    {
      sendProcessClear();
      refresh();
      return;
    }

    auto request =
        std::make_shared<ClearTorch::Request>();

    request->operator_approved = true;

    try
    {
      auto sent =
          torch_client_->async_send_request(
              request);

      torch_request_id_ = sent.request_id;
      torch_future_ = std::move(sent.future);

      phase_ = Phase::ClearingTorch;
      deadline_ = Clock::now() + 5s;

      result_->setText(
          "Clearing torch controller fault...");
    }
    catch (const std::exception& e)
    {
      fail(
          QString("Torch clear send failed: ") +
          e.what());
    }

    refresh();
  }

  void sendProcessClear()
  {
    if (!process_client_->service_is_ready())
    {
      fail(
          "Process fault clearance service unavailable.");

      return;
    }

    const auto data = snapshot();

    // Do not clear TaskState if the torch is faulted
    // again or the live state is no longer safe.
    if (!safeToRequestClear(data) ||
        data.torch->fault)
    {
      fail(
          "Process clearance blocked: torch fault "
          "or unsafe/stale telemetry.");

      return;
    }

    auto request =
        std::make_shared<Trigger::Request>();

    try
    {
      auto sent =
          process_client_->async_send_request(
              request);

      process_request_id_ = sent.request_id;
      process_future_ = std::move(sent.future);

      phase_ = Phase::ClearingProcess;
      deadline_ = Clock::now() + 5s;

      result_->setText(
          "Clearing TaskState process fault...");
    }
    catch (const std::exception& e)
    {
      fail(
          QString("Process clear send failed: ") +
          e.what());
    }
  }

  void fail(const QString& message)
  {
    phase_ = Phase::Idle;

    result_->setText(message);
  }

  void pollRequests()
  {
    const auto now = Clock::now();

    try
    {
      // ------------------------------------------------------
      // Stage 1: Torch controller clear service
      // ------------------------------------------------------
      if (phase_ == Phase::ClearingTorch)
      {
        if (torch_future_.valid() &&
            torch_future_.wait_for(0s) ==
                std::future_status::ready)
        {
          auto response = torch_future_.get();

          torch_future_ = {};

          if (!response || !response->success)
          {
            fail(
                response
                    ? QString::fromStdString(
                          response->message)
                    : QString(
                          "Empty torch clear response"));

            return;
          }

          phase_ = Phase::WaitingForTorchStatus;
          phase_started_ = Clock::now();
          deadline_ = phase_started_ + 5s;

          result_->setText(
              "Torch clear accepted. "
              "Waiting for fault-free telemetry...");
        }
        else if (now >= deadline_)
        {
          torch_client_->remove_pending_request(
              torch_request_id_);

          torch_future_ = {};

          fail(
              "Torch clear timed out; outcome unknown.");

          return;
        }
      }

      // ------------------------------------------------------
      // Stage 2: Confirm torch telemetry is fault-free
      // ------------------------------------------------------
      if (phase_ == Phase::WaitingForTorchStatus)
      {
        const auto data = snapshot();

        const bool confirmed =
            data.torch_fresh &&
            data.torch_received >= phase_started_ &&
            !data.torch->fault &&
            data.torch->io_connected &&
            !data.torch->command_on &&
            !data.torch->output_on &&
            !data.torch->transfer_detected;

        if (confirmed)
        {
          sendProcessClear();
        }
        else if (now >= deadline_)
        {
          fail(
              "Torch did not confirm fault-free status.");

          return;
        }
      }

      // ------------------------------------------------------
      // Stage 3: TaskState process clear service
      // ------------------------------------------------------
      if (phase_ == Phase::ClearingProcess)
      {
        if (process_future_.valid() &&
            process_future_.wait_for(0s) ==
                std::future_status::ready)
        {
          auto response = process_future_.get();

          process_future_ = {};

          if (!response || !response->success)
          {
            fail(
                response
                    ? QString::fromStdString(
                          response->message)
                    : QString(
                          "Empty process clear response"));

            return;
          }

          phase_ = Phase::AwaitingConfirmation;
          phase_started_ = Clock::now();
          deadline_ = phase_started_ + 3s;

          result_->setText(
              "Process clear accepted. "
              "Waiting for TaskState confirmation...");
        }
        else if (now >= deadline_)
        {
          process_client_->remove_pending_request(
              process_request_id_);

          process_future_ = {};

          fail(
              "Process clear timed out; outcome unknown.");

          return;
        }
      }

      // ------------------------------------------------------
      // Stage 4: Confirm both latches are clear
      // ------------------------------------------------------
      if (phase_ == Phase::AwaitingConfirmation)
      {
        const auto data = snapshot();

        const bool confirmed =
            data.torch_fresh &&
            data.task_fresh &&
            data.task_received >= phase_started_ &&
            !data.torch->fault &&
            !data.task->process_fault_active;

        if (confirmed)
        {
          phase_ = Phase::Idle;

          result_->setText(
              "Torch and process fault latches cleared. "
              "Motion requires separate authorization.");
        }
        else if (now >= deadline_)
        {
          fail(
              "TaskState did not confirm "
              "fault clearance.");

          return;
        }
      }
    }
    catch (const std::exception& e)
    {
      fail(
          QString("Clearance error: ") +
          e.what());
    }
  }

  void refresh()
  {
    pollRequests();

    const auto data = snapshot();

    QString text;

    // --------------------------------------------------------
    // Torch status
    // --------------------------------------------------------
    if (!data.torch_fresh)
    {
      text +=
          "Torch status: STALE / UNAVAILABLE\n";
    }
    else
    {
      const auto& torch = *data.torch;

      text += QString("Torch fault: %1\n")
          .arg(
              torch.fault
                  ? "ACTIVE"
                  : "CLEAR");

      const QString reason =
          torch.fault_reason.empty()
              ? QString("None reported")
              : QString::fromStdString(
                    torch.fault_reason);

      text += "Reason: " + reason + "\n";

      text += QString("I/O: %1\n")
          .arg(
              torch.io_connected
                  ? "Connected"
                  : "LOST");

      text += QString("Torch command: %1\n")
          .arg(
              torch.command_on
                  ? "ON"
                  : "OFF");

      text += QString("Torch output: %1\n")
          .arg(
              torch.output_on
                  ? "ON"
                  : "OFF");

      text += QString("Arc transfer: %1\n")
          .arg(
              torch.transfer_detected
                  ? "DETECTED"
                  : "NONE");

      if (!torch.segment_name.empty())
      {
        text +=
            "Segment: " +
            QString::fromStdString(
                torch.segment_name) +
            "\n";
      }
    }

    text += "\n";

    // --------------------------------------------------------
    // TaskState status
    // --------------------------------------------------------
    if (!data.task_fresh)
    {
      text +=
          "TaskState: STALE / UNAVAILABLE\n";
    }
    else
    {
      const auto& task = *data.task;

      text += QString("Process fault: %1\n")
          .arg(
              task.process_fault_active
                  ? "ACTIVE"
                  : "CLEAR");

      text += QString("Motion fault: %1\n")
          .arg(
              task.motion_fault_active
                  ? "ACTIVE"
                  : "CLEAR");

      text += QString("Supervision: %1\n")
          .arg(
              task.supervision_valid
                  ? "VALID"
                  : "INVALID");

      text += QString("IMU: %1\n")
          .arg(
              task.imu_stable
                  ? "STABLE"
                  : "UNSTABLE");
    }

    status_->setPlainText(text.trimmed());

    // --------------------------------------------------------
    // Button availability
    // --------------------------------------------------------
    bool fault_active = false;

    if (data.torch_fresh &&
        data.task_fresh)
    {
      fault_active =
          data.torch->fault ||
          data.task->process_fault_active;
    }

    const bool services_ready =
        torch_client_->service_is_ready() &&
        process_client_->service_is_ready();

    clear_button_->setEnabled(
        phase_ == Phase::Idle &&
        safeToRequestClear(data) &&
        fault_active &&
        services_ready);
  }

  // ----------------------------------------------------------
  // ROS telemetry
  // ----------------------------------------------------------

  std::mutex mutex_;

  std::optional<TorchStatus> torch_;
  std::optional<TaskState> task_;

  Clock::time_point torch_received_{};
  Clock::time_point task_received_{};

  rclcpp::Subscription<TorchStatus>::SharedPtr
      torch_sub_;

  rclcpp::Subscription<TaskState>::SharedPtr
      task_sub_;

  // ----------------------------------------------------------
  // ROS service clients
  // ----------------------------------------------------------

  rclcpp::Client<ClearTorch>::SharedPtr
      torch_client_;

  rclcpp::Client<Trigger>::SharedPtr
      process_client_;

  std::shared_future<
      ClearTorch::Response::SharedPtr>
      torch_future_;

  std::shared_future<
      Trigger::Response::SharedPtr>
      process_future_;

  int64_t torch_request_id_{0};
  int64_t process_request_id_{0};

  // ----------------------------------------------------------
  // Clearance state machine
  // ----------------------------------------------------------

  Phase phase_{Phase::Idle};

  Clock::time_point phase_started_{};
  Clock::time_point deadline_{};

  // ----------------------------------------------------------
  // Qt widgets
  // ----------------------------------------------------------

  QTextEdit* status_{
      new QTextEdit};

  QPushButton* clear_button_{
      new QPushButton(
          "Clear torch + process fault")};

  QLabel* result_{
      new QLabel(
          "No fault clearance requested")};
};

}  // namespace


// ============================================================
// RViz panel
// ============================================================

class MotionApprovalPanel : public rviz_common::Panel
{
public:
  void onInitialize() override
  {
    auto abstraction =
        getDisplayContext()
            ->getRosNodeAbstraction()
            .lock();

    if (!abstraction)
    {
      return;
    }

    auto node =
        abstraction->get_raw_node();

    auto* layout = new QVBoxLayout(this);

    // Return to inspection home.
    layout->addWidget(
        new HomeRow(node));

    // Inspection motion approval.
    layout->addWidget(
        new ApprovalRow(
            *node,
            "Profile inspection",
            "approve_profile_motion"));

    // Cutting motion approval.
    layout->addWidget(
        new ApprovalRow(
            *node,
            "Cut motion",
            "approve_cut_motion"));

    // Torch and process fault recovery.
    layout->addWidget(
        new TorchFaultRow(node));

    layout->addStretch();
  }
};

}  // namespace zk_gui


PLUGINLIB_EXPORT_CLASS(
    zk_gui::MotionApprovalPanel,
    rviz_common::Panel)

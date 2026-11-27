#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hb_robot_interfaces/srv/approve_motion.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.h>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using MGI = moveit::planning_interface::MoveGroupInterface;
using TaskState = hb_robot_interfaces::msg::TaskState;
using Approval = hb_robot_interfaces::srv::ApproveMotion;

class AwpExcitationServer final : public rclcpp::Node {
public:
  explicit AwpExcitationServer(bool cutting = false) : Node(cutting ? "cut_pose_experiment" : "awp_excitation_server"), cutting_(cutting) {
    group_ = declare_parameter<std::string>("planning_group", "manipulator");
    home_ = declare_parameter<std::string>("home_named_target", cutting ? "inspection_home" : "excitation_home");
    nominal_targets_ = declare_parameter<std::vector<std::string>>("nominal_targets", cutting ? std::vector<std::string>{"cut_forward","cut_down","cut_up"} : std::vector<std::string>{"excitation_home"});
    joint_names_ = declare_parameter<std::vector<std::string>>("excited_joints", std::vector<std::string>{});
    delta_ = declare_parameter<double>("joint_delta_rad", 0.035);
    scales_ = declare_parameter<std::vector<double>>("velocity_scales", {0.05, 0.10});
    accel_scale_ = declare_parameter<double>("acceleration_scale", 0.05);
    repetitions_ = declare_parameter<int>("repetitions", 1);
    min_observation_ = declare_parameter<double>("min_observation_s", 4.0);
    settle_timeout_ = declare_parameter<double>("settle_timeout_s", 15.0);
    settle_hold_ = declare_parameter<double>("settle_hold_s", 0.8);
    settle_window_ = declare_parameter<double>("settle_window_s", 0.5);
    accel_rms_limit_ = declare_parameter<double>("settle_accel_rms_m_s2", 0.20);
    gyro_rms_limit_ = declare_parameter<double>("settle_gyro_rms_rad_s", 0.025);
    max_sample_age_ = declare_parameter<double>("max_imu_age_s", 0.3);
    start_tolerance_ = declare_parameter<double>("start_joint_tolerance_rad", 0.02);
    planning_time_ = declare_parameter<double>("planning_time_s", 8.0);
    execute_motion_ = declare_parameter<bool>("execute_motion", false);
    require_approval_ = declare_parameter<bool>("require_motion_approval", true);
    const auto uni_topic = declare_parameter<std::string>("imu_1_topic", "/mpu6050/imu");
    const auto mpu_topic = declare_parameter<std::string>("imu_2_topic", "/mpu6050_2/imu");
    for (const double v : {delta_, accel_scale_, min_observation_, settle_timeout_, settle_hold_,
         settle_window_, accel_rms_limit_, gyro_rms_limit_, max_sample_age_, start_tolerance_, planning_time_})
      if (!std::isfinite(v)) throw std::invalid_argument("Non-finite experiment parameter");
    if (nominal_targets_.empty() || joint_names_.size() > 6 || !(delta_ > 0 && delta_ <= 0.15) ||
        repetitions_ < 1 || repetitions_ > 5 || scales_.empty() ||
        !std::all_of(scales_.begin(), scales_.end(), [](double v){return std::isfinite(v) && v > 0 && v <= 0.5;}) ||
        !(accel_scale_ > 0 && accel_scale_ <= 0.5) || min_observation_ < 0 ||
        settle_timeout_ < min_observation_ || settle_hold_ <= 0 || settle_window_ <= 0 ||
        accel_rms_limit_ <= 0 || gyro_rms_limit_ <= 0 || max_sample_age_ <= 0 ||
        start_tolerance_ <= 0 || planning_time_ <= 0) {
      throw std::invalid_argument("Invalid excitation parameters");
    }
    events_ = create_publisher<std_msgs::msg::String>("~/events", rclcpp::QoS(100).reliable());
    display_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    auto qos = rclcpp::SensorDataQoS();
    uni_sub_ = create_subscription<sensor_msgs::msg::Imu>(uni_topic, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){addImu(0, *m);});
    mpu_sub_ = create_subscription<sensor_msgs::msg::Imu>(mpu_topic, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){addImu(1, *m);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){
        std::lock_guard<std::mutex> l(mutex_); joint_received_ = Clock::now();
        joint_state_ = *m;
      });
    motion_fault_sub_ = create_subscription<std_msgs::msg::Bool>("/motion_fault", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data) fault("/motion_fault");});
    task_sub_ = create_subscription<TaskState>("/task_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](TaskState::ConstSharedPtr m){
        if (m->motion_fault_active || m->process_fault_active) fault("TaskState fault");
      });
    approval_service_ = create_service<Approval>("~/approve_motion",
      [this](const Approval::Request::SharedPtr req, Approval::Response::SharedPtr resp){
        std::lock_guard<std::mutex> l(mutex_);
        if (!approval_pending_) {
          resp->accepted = false; setApprovalMessage(*resp, false, "No pending trial approval"); return;
        }
        approved_ = req->approve; rejected_ = !req->approve;
        resp->accepted = true;
        setApprovalMessage(*resp, req->approve, req->approve ? "Approved pending step" : "Rejected pending step");
        cv_.notify_all();
      });
    start_service_ = create_service<std_srvs::srv::Trigger>("~/start",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr resp){
        if (!initialized_.load()) {resp->success=false;resp->message="Not initialized";return;}
        if (fault_latched_.load()) {resp->success=false;resp->message="Fault latched; restart node after diagnosis";return;}
        bool expected=false;
        if (!busy_.compare_exchange_strong(expected,true)) {
          resp->success=false;resp->message="Campaign already active";return;
        }
        if (worker_.joinable()) worker_.join();
        stop_.store(false);
        worker_=std::thread([this]{run();busy_.store(false);});
        resp->success=true;resp->message="Campaign started; awaiting per-step approval if enabled";
      });
    watchdog_ = create_wall_timer(100ms,[this]{
      if (busy_.load() && execute_motion_ && (!imuFresh() || !jointFeedbackFresh()))
        requestStop("Sensor or joint feedback stale");
    });
    stop_service_ = create_service<std_srvs::srv::Trigger>("~/stop",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr resp){
        requestStop("Operator stop");resp->success=true;resp->message="Stop requested";
      });
  }

  void initialize() {
    move_group_ = std::make_shared<MGI>(shared_from_this(), group_);
    move_group_->setPlanningTime(planning_time_);
    const auto * jmg = move_group_->getRobotModel()->getJointModelGroup(group_);
    if (!jmg) throw std::runtime_error("Unknown MoveIt group " + group_);
    const auto variables = jmg->getVariableNames();
    if (joint_names_.empty()) joint_names_ = variables;
    if (variables.size() != 6) throw std::runtime_error("Expected a six-joint UR planning group");
    auto unique_names=joint_names_;
    std::sort(unique_names.begin(),unique_names.end());
    if (std::adjacent_find(unique_names.begin(),unique_names.end()) != unique_names.end())
      throw std::runtime_error("Duplicate excited joint name");
    for (const auto & n : joint_names_) {
      if (std::find(variables.begin(), variables.end(), n) == variables.end())
        throw std::runtime_error("Joint not in planning group: " + n);
    }
    {
      auto required=nominal_targets_; required.push_back(home_);
      for (const auto & target : required) {
        const auto names=move_group_->getNamedTargets();
        if (std::find(names.begin(), names.end(), target)==names.end())
          throw std::runtime_error("SRDF named target missing: " + target);
      }
    }
    initialized_.store(true);
    RCLCPP_WARN(get_logger(), "Excitation node initialized: execute_motion=%d approval=%d; torch MUST remain disabled", execute_motion_, require_approval_);
  }

  ~AwpExcitationServer() override {
    stop_.store(true); cv_.notify_all();
    if (busy_.load() && initialized_.load()) {try {move_group_->stop();} catch (...) {}}
    if (worker_.joinable()) worker_.join();
  }

private:
  template<class Response> static void setApprovalMessage(Response& r, bool ok, const std::string& text) {
    if constexpr (std::is_assignable_v<decltype(r.message)&, std::string>) r.message=text;
    else r.message=ok;
  }
  struct Sample {Clock::time_point received; double ax,ay,az,gx,gy,gz;};
  void addImu(size_t i, const sensor_msgs::msg::Imu & m) {
    if (m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000u ||
        m.angular_velocity_covariance[0]<0 || m.linear_acceleration_covariance[0]<0) return;
    const double age=(now()-rclcpp::Time(m.header.stamp)).seconds();
    if (!std::isfinite(age) || age < -0.005 || age > max_sample_age_) return;
    const auto & a=m.linear_acceleration; const auto & g=m.angular_velocity;
    if (!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||
        !std::isfinite(g.x)||!std::isfinite(g.y)||!std::isfinite(g.z)) return;
    std::lock_guard<std::mutex> l(mutex_);
    auto & q=imu_[i];q.push_back({Clock::now()-std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::max(0.0,age))),a.x,a.y,a.z,g.x,g.y,g.z});
    while(q.size()>1000 || (!q.empty() && Clock::now()-q.front().received>3s)) q.pop_front();
  }
  void event(const std::string & kind, const std::string & details="") {
    std_msgs::msg::String m;
    std::ostringstream os;
    os<<"stamp_ns="<<now().nanoseconds()<<" event="<<kind<<" "<<details;
    m.data=os.str();events_->publish(m);
    RCLCPP_INFO(get_logger(), "%s", m.data.c_str());
  }
  void requestStop(const std::string & why) {
    const bool first=!stop_.exchange(true);
    {std::lock_guard<std::mutex> l(mutex_);cv_.notify_all();}
    if (first) {
      event("STOP_REQUESTED", "reason="+why);
      if (initialized_.load()) move_group_->stop();
    }
  }
  void fault(const std::string & why) {fault_latched_.store(true);requestStop(why);}
  bool alive() const {return rclcpp::ok() && !stop_.load();}
  bool waitSeconds(double seconds) {
    const auto until=Clock::now()+std::chrono::duration<double>(seconds);
    while(alive() && Clock::now()<until) std::this_thread::sleep_for(20ms);
    return alive();
  }
  bool imuFresh() {
    std::lock_guard<std::mutex> l(mutex_);
    for(const auto & q:imu_)
      if(q.empty() || std::chrono::duration<double>(Clock::now()-q.back().received).count()>max_sample_age_) return false;
    return true;
  }
  bool settled() {
    std::lock_guard<std::mutex> l(mutex_);
    for(const auto & q:imu_) {
      if(q.empty() || std::chrono::duration<double>(Clock::now()-q.back().received).count()>max_sample_age_) return false;
      const auto cutoff=Clock::now()-std::chrono::duration<double>(settle_window_);
      double means[6]={};size_t n=0;
      for(const auto & s:q) if(s.received>=cutoff){
        const double v[]={s.ax,s.ay,s.az,s.gx,s.gy,s.gz};
        for(int k=0;k<6;++k)means[k]+=v[k];++n;
      }
      Clock::time_point first{};
      for(const auto & s:q) if(s.received>=cutoff) {first=s.received;break;}
      if(n<10 || first==Clock::time_point{} ||
         std::chrono::duration<double>(q.back().received-first).count()<settle_window_*0.9) return false;
      for(double & v:means)v/=n;
      double aa=0,gg=0;
      for(const auto & s:q) if(s.received>=cutoff){
        const double v[]={s.ax,s.ay,s.az,s.gx,s.gy,s.gz};
        for(int k=0;k<3;++k){aa+=(v[k]-means[k])*(v[k]-means[k]);gg+=(v[k+3]-means[k+3])*(v[k+3]-means[k+3]);}
      }
      if(std::sqrt(aa/n)>accel_rms_limit_ || std::sqrt(gg/n)>gyro_rms_limit_) return false;
    }
    return true;
  }
  bool observeAndSettle(const std::string & label) {
    event("SETTLING_START", label);
    const auto started=Clock::now();
    Clock::time_point stable_since{};
    while(alive()) {
      const double elapsed=std::chrono::duration<double>(Clock::now()-started).count();
      if(elapsed>settle_timeout_) {event("SETTLING_TIMEOUT",label);return false;}
      if(!imuFresh()){event("SENSOR_STALE",label);return false;}
      if(elapsed>=min_observation_ && settled()) {
        if(stable_since==Clock::time_point{}) stable_since=Clock::now();
        if(std::chrono::duration<double>(Clock::now()-stable_since).count()>=settle_hold_) {
          event("SETTLED",label+" elapsed_s="+std::to_string(elapsed));return true;
        }
      } else stable_since={};
      std::this_thread::sleep_for(40ms);
    }
    return false;
  }
  bool approve(const std::string & label) {
    if (!require_approval_ || !execute_motion_) return alive();
    {
      std::lock_guard<std::mutex> l(mutex_);
      approval_pending_=true;approved_=false;rejected_=false;
    }
    event("APPROVAL_REQUIRED",label);
    std::unique_lock<std::mutex> l(mutex_);
    cv_.wait(l,[this]{return approved_||rejected_||stop_.load()||!rclcpp::ok();});
    const bool ok=approved_ && alive();
    approval_pending_=false;approved_=false;rejected_=false;
    l.unlock();
    event(ok?"APPROVED":"APPROVAL_DENIED",label);
    return ok;
  }
  bool jointFeedbackFresh() {
    std::lock_guard<std::mutex> l(mutex_);
    return joint_received_!=Clock::time_point{} && Clock::now()-joint_received_<500ms;
  }
  bool moveTo(const std::string & label, const std::vector<double> & target,
              double velocity, double acceleration) {
    if (!alive() || !jointFeedbackFresh() || !imuFresh()) {event("PRECONDITION_FAILED",label);return false;}
    auto current=move_group_->getCurrentState(2.0);
    if (!current) {event("NO_ROBOT_STATE",label);return false;}
    const auto * jmg=move_group_->getRobotModel()->getJointModelGroup(group_);
    std::vector<double> actual;current->copyJointGroupPositions(jmg,actual);
    if (actual.size()!=target.size()) return false;
    move_group_->setStartStateToCurrentState();
    move_group_->setMaxVelocityScalingFactor(velocity);
    move_group_->setMaxAccelerationScalingFactor(acceleration);
    if (!move_group_->setJointValueTarget(target)) {event("INVALID_TARGET",label);return false;}
    MGI::Plan plan;
    if (move_group_->plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS) {event("PLAN_FAILED",label);return false;}
    const auto & jt=plan.trajectory_.joint_trajectory;
    if(jt.points.empty() || jt.joint_names.empty()) {event("EMPTY_PLAN",label);return false;}
    auto fresh=move_group_->getCurrentState(2.0);
    if(!fresh || !jointFeedbackFresh() || !imuFresh()) {event("STALE_BEFORE_APPROVAL",label);return false;}
    for(size_t i=0;i<jt.joint_names.size();++i) {
      if(jt.points.front().positions.size()!=jt.joint_names.size() ||
         std::abs(fresh->getVariablePosition(jt.joint_names[i])-jt.points.front().positions[i])>start_tolerance_) {
        event("START_MISMATCH",label);return false;
      }
    }
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id=move_group_->getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(*fresh,display.trajectory_start);
    display.trajectory.push_back(plan.trajectory_);display_->publish(display);
    event("PLAN_READY",label+" velocity="+std::to_string(velocity)+" acceleration="+std::to_string(acceleration));
    if(!execute_motion_) {event("PREVIEW_ONLY",label);return true;}
    if(!approve(label)) return false;
    fresh=move_group_->getCurrentState(2.0);
    if(!alive() || !fresh || !jointFeedbackFresh() || !imuFresh()) return false;
    for(size_t i=0;i<jt.joint_names.size();++i)
      if(std::abs(fresh->getVariablePosition(jt.joint_names[i])-jt.points.front().positions[i])>start_tolerance_) {
        event("START_CHANGED_AFTER_APPROVAL",label);return false;
      }
    event("MOTION_START",label);
    const auto code=move_group_->execute(plan);
    event("MOTION_END",label+" moveit_code="+std::to_string(code.val));
    if(!alive() || code!=moveit::core::MoveItErrorCode::SUCCESS) return false;
    auto reached=move_group_->getCurrentState(2.0);
    if(!reached || !jointFeedbackFresh()) return false;
    for(size_t i=0;i<jt.joint_names.size();++i)
      if(std::abs(reached->getVariablePosition(jt.joint_names[i])-jt.points.back().positions[i])>start_tolerance_) {
        event("END_MISMATCH",label);return false;
      }
    return true;
  }
  bool namedPosition(const std::string & name,std::vector<double> & q) {
    const auto names=move_group_->getNamedTargets();
    if(std::find(names.begin(),names.end(),name)==names.end()) return false;
    const auto values=move_group_->getNamedTargetValues(name);
    auto state=move_group_->getCurrentState(2.0);
    if(!state) return false;
    state->setVariablePositions(values);
    state->update();
    const auto * jmg=move_group_->getRobotModel()->getJointModelGroup(group_);
    state->copyJointGroupPositions(jmg,q);
    return q.size()==jmg->getVariableCount();
  }
  void run() {
    event("CAMPAIGN_START", "home="+home_+" delta_rad="+std::to_string(delta_)+" acceleration_scale="+std::to_string(accel_scale_));
    bool completed=true;
    try {
      if(!imuFresh()) {event("CAMPAIGN_ABORT", "reason=IMU_not_fresh");return;}
      const auto * jmg=move_group_->getRobotModel()->getJointModelGroup(group_);
      const auto model_names=jmg->getVariableNames();
      if (cutting_) { runCutPostures(); return; }
      for(const auto & nominal:nominal_targets_) {
        if(!alive()) break;
        std::vector<double> q0;
        if(!namedPosition(nominal,q0)) throw std::runtime_error("Named target unavailable: "+nominal);
        const std::string nominal_label="nominal="+nominal;
        if(!moveTo(nominal_label+" phase=enter",q0,scales_.front(),accel_scale_)) {completed=false;break;}
        if(!execute_motion_) { event("PREVIEW_COMPLETE",nominal_label); return; }
        if(!observeAndSettle(nominal_label+" phase=baseline")) {completed=false;break;}
        event("BASELINE_COMPLETE",nominal_label);
        for(const auto & joint:joint_names_) {
          if(!alive()) break;
          const auto it=std::find(model_names.begin(),model_names.end(),joint);
          if(it==model_names.end()) throw std::runtime_error("Unknown joint "+joint);
          const size_t index=std::distance(model_names.begin(),it);
          for(const double speed:scales_) {
            if(!alive()) break;
            for(int repeat=0;repeat<repetitions_;++repeat) {
              if(!alive()) break;
              for(int direction:{1,-1}) {
                if(!alive()) break;
                auto q=q0;q[index]+=direction*delta_;
                const std::string label=nominal_label+" joint="+joint+" sign="+
                  std::to_string(direction)+" repeat="+std::to_string(repeat)+
                  " speed="+std::to_string(speed);
                if(!moveTo(label+" phase=excite",q,speed,accel_scale_) ||
                   !observeAndSettle(label+" phase=excite")) {completed=false;requestStop("Excitation failed");break;}
                if(!moveTo(label+" phase=return",q0,speed,accel_scale_) ||
                   !observeAndSettle(label+" phase=return")) {completed=false;requestStop("Return failed");break;}
                event("TRIAL_COMPLETE",label);
              }
            }
          }
        }
      }
      event((alive() && completed)?"CAMPAIGN_COMPLETE":"CAMPAIGN_ABORT");
    } catch(const std::exception & e) {
      event("CAMPAIGN_EXCEPTION",std::string("reason=")+e.what());
      requestStop("Exception");
    }
  }

  void runCutPostures() {
    std::vector<double> home;
    if (!namedPosition(home_,home)) throw std::runtime_error("Missing home target: "+home_);
    if (!moveTo("pose="+home_+" phase=initial_home",home,scales_.front(),accel_scale_))
      throw std::runtime_error("Initial home failed");
    if (!execute_motion_) {event("PREVIEW_COMPLETE","pose="+home_);return;}
    if (!observeAndSettle("pose="+home_+" phase=home_baseline")) throw std::runtime_error("Home baseline failed");
    for (int repeat=0;repeat<repetitions_;++repeat) for (const auto& name:nominal_targets_) {
      if (!alive()) throw std::runtime_error("Campaign stopped");
      std::vector<double> q;
      if (!namedPosition(name,q)) throw std::runtime_error("Missing pose "+name);
      const auto label="pose="+name+" repeat="+std::to_string(repeat);
      if (!moveTo(label+" phase=cut_posture",q,scales_.front(),accel_scale_) ||
          !observeAndSettle(label+" phase=measurement")) throw std::runtime_error("Posture trial failed");
      event("MEASUREMENT_COMPLETE",label);
      if (!moveTo("pose="+home_+" phase=return_home",home,scales_.front(),accel_scale_) ||
          !observeAndSettle("pose="+home_+" phase=return_baseline")) throw std::runtime_error("Home return failed");
    }
    event("CAMPAIGN_COMPLETE");
  }
  bool cutting_;
  std::string group_,home_;
  std::vector<std::string> nominal_targets_,joint_names_;
  std::vector<double> scales_;
  double delta_,accel_scale_,min_observation_,settle_timeout_,settle_hold_,settle_window_;
  double accel_rms_limit_,gyro_rms_limit_,max_sample_age_,start_tolerance_,planning_time_;
  int repetitions_;
  bool execute_motion_,require_approval_;
  std::shared_ptr<MGI> move_group_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Sample> imu_[2];
  sensor_msgs::msg::JointState joint_state_;
  Clock::time_point joint_received_{};
  bool approval_pending_=false,approved_=false,rejected_=false;
  std::atomic<bool> initialized_{false},busy_{false},stop_{false},fault_latched_{false};
  std::thread worker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr uni_sub_,mpu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_fault_sub_;
  rclcpp::Subscription<TaskState>::SharedPtr task_sub_;
  rclcpp::Service<Approval>::SharedPtr approval_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_,stop_service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

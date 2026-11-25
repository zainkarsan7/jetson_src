#pragma once
// Included after Context and registerAction in bt_executor.cpp.

inline bool knownInterruptedFeature(const std::shared_ptr<Context>& context) {
  using Status = Context::FeaturePlanEntry::Status;
  if (!context->independent_batch || !context->batch_interrupted ||
      context->interrupted_plan_id.empty() || context->interrupted_feature_id.empty()) return false;
  const auto ledger = context->feature_recovery.find(context->interrupted_feature_id);
  return ledger != context->feature_recovery.end() && ledger->second.interrupted &&
      !ledger->second.original_plan_id.empty() &&
      std::any_of(context->feature_plans.begin(), context->feature_plans.end(), [&](const auto& e) {
        return e.feature_id == context->interrupted_feature_id && e.status == Status::Interrupted;
      });
}

inline bool featureReadyForInspection(const std::shared_ptr<Context>& context) {
  using Status = Context::FeaturePlanEntry::Status;
  const auto ledger = context->feature_recovery.find(context->recovery_feature);
  if (context->batch_interrupted || ledger == context->feature_recovery.end() || ledger->second.interrupted)
    return false;
  return std::any_of(context->feature_plans.begin(), context->feature_plans.end(), [&](const auto& e) {
    return e.feature_id == context->recovery_feature &&
        (e.status == Status::Completed || e.status == Status::RecoveredPendingInspection);
  });
}

inline bool markFeatureRecovered(const std::shared_ptr<Context>& context) {
  if (!context->recovery_admitted || !context->recovery_home_reached || !knownInterruptedFeature(context))
    return false;
  for (auto& entry : context->feature_plans) {
    if (entry.feature_id != context->interrupted_feature_id) continue;
    entry.status = Context::FeaturePlanEntry::Status::RecoveredPendingInspection;
    entry.message = "Recovery motion completed; cut outcome awaits inspection";
    auto& feature = context->feature_recovery.at(entry.feature_id);
    feature.interrupted = false;
    feature.evidence = recovery::Evidence{};
    feature.verdict = {};
    context->recovery_feature = entry.feature_id;
    // Original lineage, motion epoch and write-ahead attempt history survive.
    context->batch_interrupted = false;
    context->recovery_home_reached = false;
    context->interrupted_plan_id.clear();
    context->interrupted_feature_id.clear();
    context->plan_id.clear();
    context->preparing_candidate = false;
    return true;
  }
  return false;
}

inline bool hasDeferredFeatures(const std::shared_ptr<Context>& context) {
  return std::any_of(context->feature_plans.begin(),context->feature_plans.end(),[](const auto& e) {
    return e.status==Context::FeaturePlanEntry::Status::Deferred;
  });
}

inline bool allActiveFeaturesVerified(const std::shared_ptr<Context>& context) {
  const auto state=context->snapshot();
  if(context->required_segments.empty()) return false;
  for(const auto& name:context->required_segments) {
    const auto entry=std::find_if(context->feature_plans.begin(),context->feature_plans.end(),
        [&](const auto& e){return e.feature_id==name;});
    if(entry!=context->feature_plans.end() && entry->status==Context::FeaturePlanEntry::Status::Deferred)
      continue;
    const auto it=context->feature_recovery.find(name);
    if(it==context->feature_recovery.end() || !it->second.verified(state,context->cut_id,name)) return false;
  }
  return true;
}

inline bool allFeaturesVerified(const std::shared_ptr<Context>& context) {
  return !hasDeferredFeatures(context) && allActiveFeaturesVerified(context);
}

// Called only by the tick thread after session containment/authorization checks.
inline bool deferInspectionFeature(const std::shared_ptr<Context>& context) {
  if (!featureReadyForInspection(context)) return false;
  const auto it=std::find_if(context->feature_plans.begin(),context->feature_plans.end(),
      [&](const auto& e){return e.feature_id==context->recovery_feature;});
  if (it==context->feature_plans.end()) return false;
  it->status=Context::FeaturePlanEntry::Status::Deferred;
  it->message="Operator deferred inspection/repair; cut outcome remains unverified";
  // Preserve every motion/evidence epoch and attempted interval. No verdict is invented.
  context->plan_id.clear();
  context->preparing_candidate=false;
  context->recovery_feature.clear();
  return true;
}

template<class LegacyPlanGoal,class LegacyPlanResult,class LegacyExecuteGoal,class LegacyExecuteResult>
void installRecovery(BT::BehaviorTreeFactory& factory,const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Context>& context,double standoff,double pos_tol,double ang_tol,
    LegacyPlanGoal old_plan_goal,LegacyPlanResult old_plan_result,
    LegacyExecuteGoal old_execute_goal,LegacyExecuteResult old_execute_result) {
  const auto active=[=]() -> recovery::FeatureRecoveryState* {
    const auto it=context->feature_recovery.find(context->recovery_feature);
    return it==context->feature_recovery.end()?nullptr:&it->second;
  };
  const auto verdict=[=]() {
    auto* f=active();
    return context->independent_batch?(f?recovery::evaluateFeature(f->evidence,context->snapshot(),
      context->cut_id,context->recovery_feature,f->last_motion_epoch):recovery::Verdict{}):
      recovery::evaluate(context->evidence,context->snapshot(),context->cut_id);
  };
  const auto condition=[&](const std::string& name,auto predicate) {
    factory.registerSimpleCondition(name,[predicate](BT::TreeNode&) {
      return predicate()?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
    });
  };
  condition("ValidateKerf",[=] {
    context->verdict=verdict();
    if(auto* f=active()) f->verdict=context->verdict;
    // Only new, valid geometric evidence resolves an interrupted cut outcome.
    if (featureReadyForInspection(context) && !context->batch_interrupted &&
        (context->verdict.status==recovery::State::VERIFIED ||
         context->verdict.status==recovery::State::RESIDUAL_MATERIAL))
      context->execution_uncertain=false;
    return context->verdict.status!=recovery::State::INVALID_OBSERVATION;
  });
  condition("KerfVerified",[=]{return verdict().status==recovery::State::VERIFIED;});
  condition("ResidualDetected",[=]{return verdict().status==recovery::State::RESIDUAL_MATERIAL;});
  condition("KerfInconclusive",[=]{return verdict().status==recovery::State::INCONCLUSIVE;});
  condition("AllFeaturesVerified",[=]{return allFeaturesVerified(context);});
  condition("AllActiveFeaturesVerified",[=]{return allActiveFeaturesVerified(context);});
  condition("FeatureReadyForInspection",[=] {return featureReadyForInspection(context);});
  // Compatibility with older installed trees; use the same stricter semantics.
  condition("FeatureAlreadyExecuted",[=] {return featureReadyForInspection(context);});
  factory.registerBuilder<hb_robot_task_planner::ForEachPending>("ForEachRecoveryFeature",
    [=](const auto& name,const auto& config) {
      return std::make_unique<hb_robot_task_planner::ForEachPending>(name,config,[=] {
        return !allActiveFeaturesVerified(context);
      });
    });
  factory.registerSimpleAction("SelectRecoveryFeature",[=](BT::TreeNode&) {
    using Status=Context::FeaturePlanEntry::Status;
    if(!context->independent_batch || context->batch_interrupted || !context->plan_id.empty())
      return BT::NodeStatus::FAILURE;
    const auto state=context->snapshot();
    auto choose=[&](auto& entry) {
      auto& f=context->feature_recovery.at(entry.feature_id);
      context->recovery_feature=entry.feature_id;
      if(entry.status==Status::Ready && !f.original_executed) {
        context->plan_id=entry.plan_id;entry.status=Status::Selected;
      }
      return BT::NodeStatus::SUCCESS;
    };
    // Finish the selected feature, then originals, then recheck evidence made
    // historical by later executions. Every final verdict uses the live epoch.
    for(auto& entry:context->feature_plans)
      if(entry.feature_id==context->recovery_feature && (entry.status==Status::Completed || entry.status==Status::RecoveredPendingInspection) &&
         !context->feature_recovery.at(entry.feature_id).verified(state,context->cut_id,entry.feature_id)) return choose(entry);
    for(auto& entry:context->feature_plans)
      if(entry.status==Status::RecoveredPendingInspection &&
         !context->feature_recovery.at(entry.feature_id).verified(state,context->cut_id,entry.feature_id)) return choose(entry);
    for(auto& entry:context->feature_plans)
      if(entry.status==Status::Ready && !context->feature_recovery.at(entry.feature_id).original_executed)
        return choose(entry);
    for(auto& entry:context->feature_plans)
      if((entry.status==Status::Completed || entry.status==Status::RecoveredPendingInspection) && !context->feature_recovery.at(entry.feature_id).verified(state,context->cut_id,entry.feature_id)) return choose(entry);
    context->last_error="Unplanned, rejected or interrupted features require operator recovery";
    return BT::NodeStatus::FAILURE;
  });

  registerAction<Kerf>(factory,"InspectKerf",node,"inspect_cut",[=](auto& goal) {
    const auto state=context->snapshot();
    // authorized() includes the live PlatformSupervision uncertainty gate.
    if(!context->authorized(state)) return false;
    auto* f=active();
    if(context->independent_batch && (!featureReadyForInspection(context) || !f || ++f->inspection_attempts>12)) return false;
    if(!context->independent_batch && ++context->inspection_attempts>12) return false;
    {std::lock_guard<std::mutex> lock(context->mutex);goal.estimate=context->estimate;}
    goal.cut_id=context->cut_id;
    goal.required_segments=context->independent_batch?std::vector<std::string>{context->recovery_feature}:context->required_segments;
    goal.geometry_version=state.geometry_version;goal.pose_version=state.pose_version;
    goal.execution_version=state.execution_version; // Current observation epoch, not last feature motion.
    goal.samples_per_viewpoint=3;goal.camera_standoff=0.30;
    if(f) {f->evidence=recovery::Evidence{};f->verdict={};}
    context->evidence=recovery::Evidence{};return true;
  },[=](const auto& result) {
    if(context->independent_batch) {if(auto* f=active()) f->evidence=result.evidence;else return false;}
    else context->evidence=result.evidence;
    return result.success;
  },context);

  registerAction<Cut>(factory,"PlanRecut",node,"cut_profile",[=](auto& goal) {
    if(!context->independent_batch) return old_plan_goal(goal,true);
    auto* f=active();const auto state=context->snapshot();const auto v=verdict();
    if(!featureReadyForInspection(context) || !f || f->original_plan_id.empty() || f->recut_attempts>=4 ||
       v.status!=recovery::State::RESIDUAL_MATERIAL || !context->authorized(state) ||
       !context->captured_scene ||
       !recovery::sameVersions(state,*context->captured_scene) ||
       state.cut_environment_id!=context->captured_scene->cut_environment_id) return false;
    const auto intervals=recovery::repairIntervals(v.residuals);
    const auto it=std::find_if(intervals.begin(),intervals.end(),[&](const auto& i) {
      return std::none_of(f->attempted.begin(),f->attempted.end(),[&](const auto& used){return recovery::overlaps(i,used);});
    });
    if(it==intervals.end()) {context->last_error="All residual intervals overlap previous attempts";return false;}
    f->planned_interval=*it;f->recut_plan_id.clear();
    goal.execute=false;goal.cut_id=context->cut_id;goal.parent_plan_id=f->original_plan_id;goal.intervals={*it};
    goal.geometry_version=state.geometry_version;goal.pose_version=state.pose_version;goal.scene_version=state.scene_version;
    goal.standoff=static_cast<float>(standoff);goal.pos_tol=static_cast<float>(pos_tol);goal.ang_tol=static_cast<float>(ang_tol);
    context->requested_plan_state=state;context->preparing_candidate=true;return true;
  },[=](const auto& result) {
    if(!context->independent_batch) return old_plan_result(result);
    auto* f=active();
    if(!f || !result.success || result.executed || result.cut_id!=context->cut_id ||
       result.parent_plan_id!=f->original_plan_id || result.feature_plans.size()!=1 ||
       !context->planMatches(context->snapshot())) return false;
    const auto& plan=result.feature_plans.front();
    if(plan.feature_id!=context->recovery_feature || plan.status!=hb_robot_interfaces::msg::CutFeaturePlan::READY || plan.plan_id.empty()) return false;
    f->recut_plan_id=plan.plan_id;context->plan_id=plan.plan_id;return true;
  },context);

  registerAction<Cut>(factory,"ExecuteRecut",node,"cut_profile",[=](auto& goal) {
    if(!context->independent_batch) return old_execute_goal(goal);
    auto* f=active();const auto state=context->snapshot();
    if(!featureReadyForInspection(context) || !f || f->recut_plan_id.empty() || !context->authorized(state) ||
       !context->planMatches(state) || !context->captured_scene ||
       state.cut_environment_id!=context->captured_scene->cut_environment_id) return false;
    goal.execute=true;goal.plan_id=f->recut_plan_id;goal.cut_id=context->cut_id;
    goal.standoff=static_cast<float>(standoff);goal.pos_tol=static_cast<float>(pos_tol);goal.ang_tol=static_cast<float>(ang_tol);
    f->attempted.push_back(f->planned_interval);++f->recut_attempts;
    f->recut_plan_id.clear();f->evidence=recovery::Evidence{};f->verdict={};
    context->preparing_candidate=false;return true;
  },[=](const auto& result) {
    if(!context->independent_batch) return old_execute_result(result);
    auto* f=active();const auto submitted_plan=context->plan_id;context->plan_id.clear();
    if(!f) return false;
    if (result.cut_id!=context->cut_id || result.plan_id!=submitted_plan) {
      context->execution_uncertain=true;context->batch_interrupted=true;
      context->last_error="Recut result identity mismatch; physical outcome unknown";
      return false;
    }
    if(result.executed) f->last_motion_epoch=result.execution_version;
    f->interrupted=result.interrupted || (result.executed && (!result.success || result.completed_segments!=1));
    context->execution_uncertain=f->interrupted;context->batch_interrupted=f->interrupted;
    context->execution_version=result.execution_version;context->executed|=result.executed;
    if(f->interrupted) {
      context->interrupted_plan_id=result.plan_id;context->interrupted_feature_id=context->recovery_feature;
      for(auto& entry:context->feature_plans) if(entry.feature_id==context->recovery_feature)
        entry.status=Context::FeaturePlanEntry::Status::Interrupted;
    }
    return result.success && result.executed && !result.interrupted && result.completed_segments==1;
  },context);
}

#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <map>
#include <string>
struct Scenario {
  std::map<std::string,int> calls;
  int evidence=2; // 1 inconclusive, 2 verified, 3 residual
  bool execution_ok=true, capture_ok=true;
  bool confirmed=true, localized=true, versions_match=true;
  bool profile_restores_pose=false, second_view_verifies=false, fault_active=false;
  BT::NodeStatus runVariant(const std::string& kind) {
    const std::string full_tree=TREE_FILE;
    const auto file=full_tree.substr(0,full_tree.find_last_of('/')+1)+kind+"_repair.xml";
    return run(file.c_str());
  }
  BT::NodeStatus run(const char* file=TREE_FILE) {
    BT::BehaviorTreeFactory f;
    for(const std::string name:{"FaultActive","ReferenceExists","WorkpiecePoseValid",
        "InspectProfile","AwaitPlanningReference","AwaitGeometry","RevalidateWorkpiece","CaptureCutEnvironment","PlanCut","ValidateTaskState",
        "ExecuteCut","InspectKerf","ValidateKerf","KerfVerified","KerfInconclusive",
        "ResidualDetected","PlanRecut","ExecuteRecut","Escape"}) {
      auto tick=[&,name](BT::TreeNode&){
        ++calls[name];
        bool success=true;
        if(name=="FaultActive") success=fault_active;
        if(name=="AwaitPlanningReference" && profile_restores_pose) confirmed=true;
        if(name=="WorkpiecePoseValid") success=confirmed;
        if(name=="RevalidateWorkpiece") success=localized;
        if(name=="ValidateTaskState") success=versions_match;
        if(name=="CaptureCutEnvironment") success=capture_ok;
        if(name=="ExecuteCut") success=execution_ok;
        if(name=="KerfVerified") success=evidence==2 || calls["ExecuteRecut"]>0 ||
          (second_view_verifies && calls["InspectKerf"]==2);
        if(name=="KerfInconclusive") success=evidence==1;
        if(name=="ResidualDetected") success=evidence==3;
        return success?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
      };
      if(name=="FaultActive" || name=="ReferenceExists" || name=="WorkpiecePoseValid" ||
         name=="ValidateTaskState" || name=="ValidateKerf" || name=="KerfVerified" ||
         name=="ResidualDetected" || name=="KerfInconclusive") f.registerSimpleCondition(name,tick);
      else f.registerSimpleAction(name,tick);
    }
    auto tree=f.createTreeFromFile(file);
    return tree.tickRoot();
  }
};
TEST(RecoveryTree, VerifiedFinishesWithoutRecut) {
  Scenario s;EXPECT_EQ(s.run(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(s.calls["InspectKerf"],1);EXPECT_EQ(s.calls["ExecuteRecut"],0);
}
TEST(RecoveryTree, UnknownReinspectsButNeverRecuts) {
  Scenario s;s.evidence=1;EXPECT_EQ(s.run(),BT::NodeStatus::FAILURE);
  EXPECT_EQ(s.calls["InspectKerf"],2);EXPECT_EQ(s.calls["ExecuteRecut"],0);
  EXPECT_EQ(s.calls["Escape"],1);
}
TEST(RecoveryTree, ResidualPlansBeforeSingleExecutionAndReinspection) {
  Scenario s;s.evidence=3;EXPECT_EQ(s.run(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(s.calls["PlanRecut"],1);EXPECT_EQ(s.calls["ExecuteRecut"],1);
  EXPECT_EQ(s.calls["InspectKerf"],2);
}
TEST(RecoveryTree, InterruptedCutContainsFaultWithoutBlindRetry) {
  Scenario s;s.execution_ok=false;EXPECT_EQ(s.run(),BT::NodeStatus::FAILURE);
  EXPECT_EQ(s.calls["ExecuteCut"],1);EXPECT_EQ(s.calls["Escape"],1);
  EXPECT_EQ(s.calls["ExecuteRecut"],0);EXPECT_EQ(s.calls["InspectKerf"],0);
}

TEST(Baseline, UnconfirmedPoseStillPlansButCannotExecuteWithoutFreshEvidence) {
  Scenario s;s.confirmed=false;s.localized=false;
  EXPECT_EQ(s.run(BASELINE_FILE),BT::NodeStatus::FAILURE);
  EXPECT_EQ(s.calls["PlanCut"],1);EXPECT_EQ(s.calls["RevalidateWorkpiece"],1);
  EXPECT_EQ(s.calls["ExecuteCut"],0);
}
TEST(Baseline, CorrectedPoseBlocksCandidateAndConfirmedSamePosePermitsIt) {
  Scenario moved;moved.confirmed=false;moved.versions_match=false;
  EXPECT_EQ(moved.run(BASELINE_FILE),BT::NodeStatus::FAILURE);
  EXPECT_EQ(moved.calls["ExecuteCut"],0);
  Scenario same;same.confirmed=false;
  EXPECT_EQ(same.run(BASELINE_FILE),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(same.calls["ExecuteCut"],1);
}

TEST(RecoveryTree, PlansBeforePostGapConfirmationAndBlocksChangedCandidate) {
  for(const bool unchanged : {false,true}) {
    Scenario s;
    s.confirmed=false;
    s.versions_match=unchanged;
    EXPECT_EQ(s.run(),unchanged ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    EXPECT_EQ(s.calls["PlanCut"],1);
    EXPECT_EQ(s.calls["RevalidateWorkpiece"],1);
    EXPECT_EQ(s.calls["ExecuteCut"],unchanged ? 1 : 0);
  }
}

TEST(Ablations, SameVerifiedOutcomeAndFaultContainment) {
  for (const std::string kind : {"process", "perception", "environment"}) {
    SCOPED_TRACE(kind);
    Scenario nominal;
    EXPECT_EQ(nominal.runVariant(kind), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(nominal.calls["ExecuteCut"], 1);
    EXPECT_EQ(nominal.calls["InspectKerf"], 1);
    EXPECT_EQ(nominal.calls["ExecuteRecut"], 0);
    Scenario fault;
    fault.fault_active = true;
    EXPECT_EQ(fault.runVariant(kind), BT::NodeStatus::FAILURE);
    EXPECT_EQ(fault.calls["Escape"], 1);
    EXPECT_EQ(fault.calls["ExecuteCut"], 0);
  }
}

TEST(Ablations, OnlyAssignedStrategyRepairsEachKerfVerdict) {
  for (const std::string kind : {"process", "perception", "environment"}) {
    SCOPED_TRACE(kind);
    Scenario residual;
    residual.evidence = 3;
    const bool recut = kind == "process";
    EXPECT_EQ(residual.runVariant(kind), recut ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    EXPECT_EQ(residual.calls["PlanRecut"], recut ? 1 : 0);
    EXPECT_EQ(residual.calls["ExecuteRecut"], recut ? 1 : 0);
    EXPECT_EQ(residual.calls["InspectKerf"], recut ? 2 : 1);
    Scenario uncertain;
    uncertain.evidence = 1;
    uncertain.second_view_verifies = true;
    const bool reinspect = kind == "perception";
    EXPECT_EQ(uncertain.runVariant(kind), reinspect ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    EXPECT_EQ(uncertain.calls["InspectKerf"], reinspect ? 2 : 1);
    EXPECT_EQ(uncertain.calls["ExecuteRecut"], 0);
  }
}

TEST(Ablations, InvalidPoseUsesOnlyAssignedRepair) {
  for (const std::string kind : {"process", "perception", "environment"}) {
    SCOPED_TRACE(kind);
    Scenario s;
    s.confirmed = false;
    s.profile_restores_pose = true;
    EXPECT_EQ(s.runVariant(kind), kind == "process" ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
    EXPECT_EQ(s.calls["InspectProfile"], kind == "perception" ? 1 : 0);
    EXPECT_EQ(s.calls["RevalidateWorkpiece"], kind == "environment" ? 1 : 0);
    EXPECT_EQ(s.calls["ExecuteCut"], kind == "process" ? 0 : 1);
  }
  Scenario moved;
  moved.confirmed = false;
  moved.versions_match = false;
  EXPECT_EQ(moved.runVariant("environment"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(moved.calls["ExecuteCut"], 0);
}

TEST(RecoveryTree, FailedCapturePreventsPlanning) {
  Scenario s;s.capture_ok=false;
  EXPECT_EQ(s.run(),BT::NodeStatus::FAILURE);
  EXPECT_EQ(s.calls["PlanCut"],0);
  EXPECT_EQ(s.calls["ExecuteCut"],0);
}

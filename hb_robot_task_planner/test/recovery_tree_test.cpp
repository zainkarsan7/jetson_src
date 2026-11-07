#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <map>
#include <string>
struct Scenario {
  std::map<std::string,int> calls;
  int evidence=2; // 1 inconclusive, 2 verified, 3 residual
  bool execution_ok=true;
  bool confirmed=true, localized=true, versions_match=true;
  BT::NodeStatus run(const char* file=TREE_FILE) {
    BT::BehaviorTreeFactory f;
    for(const std::string name:{"FaultActive","ReferenceExists","WorkpiecePoseValid",
        "InspectProfile","AwaitPlanningReference","AwaitGeometry","RevalidateWorkpiece","PlanCut","ValidateTaskState",
        "ExecuteCut","InspectKerf","ValidateKerf","KerfVerified","KerfInconclusive",
        "ResidualDetected","PlanRecut","ExecuteRecut","Escape"}) {
      auto tick=[&,name](BT::TreeNode&){
        ++calls[name];
        bool success=true;
        if(name=="FaultActive") success=false;
        if(name=="WorkpiecePoseValid") success=confirmed;
        if(name=="RevalidateWorkpiece") success=localized;
        if(name=="ValidateTaskState") success=versions_match;
        if(name=="ExecuteCut") success=execution_ok;
        if(name=="KerfVerified") success=evidence==2 || calls["ExecuteRecut"]>0;
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

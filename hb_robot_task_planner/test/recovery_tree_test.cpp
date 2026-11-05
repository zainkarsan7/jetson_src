#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <map>
#include <string>
struct Scenario {
  std::map<std::string,int> calls;
  int evidence=2; // 1 inconclusive, 2 verified, 3 residual
  bool execution_ok=true;
  BT::NodeStatus run() {
    BT::BehaviorTreeFactory f;
    for(const std::string name:{"FaultActive","ReferenceExists","WorkpiecePoseValid",
        "InspectProfile","AwaitGeometry","RevalidateWorkpiece","PlanCut","ValidateTaskState",
        "ExecuteCut","InspectKerf","ValidateKerf","KerfVerified","KerfInconclusive",
        "ResidualDetected","PlanRecut","ExecuteRecut","Escape"}) {
      f.registerSimpleAction(name,[&,name](BT::TreeNode&){
        ++calls[name];
        bool success=true;
        if(name=="FaultActive") success=false;
        if(name=="ExecuteCut") success=execution_ok;
        if(name=="KerfVerified") success=evidence==2 || calls["ExecuteRecut"]>0;
        if(name=="KerfInconclusive") success=evidence==1;
        if(name=="ResidualDetected") success=evidence==3;
        return success?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
      });
    }
    auto tree=f.createTreeFromFile(TREE_FILE);
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

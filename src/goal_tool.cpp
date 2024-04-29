#include "rviz2_panel_nav2_client/goal_tool.hpp"
#include "rviz2_panel_nav2_client/goal_common.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>

namespace custom_rviz2_plugins
{

  GoalTool::GoalTool()
    : rviz_default_plugins::tools::PoseTool()
  {
    shortcut_key_ = 'g';
  }

  GoalTool::~GoalTool()
  {}

  void GoalTool::onInitialize()
  {
    PoseTool::onInitialize();
    setName("Nav2 Client Goal");
    setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
  }

  void GoalTool::onPoseSet(double x, double y, double theta)
  {
    // Set goal pose on global object GoalUpdater to update nav2 Panel
    GoalUpdater.setGoal(x, y, theta, context_->getFixedFrame());
  }

}  // namespace custom_rviz2_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_rviz2_plugins::GoalTool, rviz_common::Tool)

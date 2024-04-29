#pragma once

// ROS2
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_default_plugins/visibility_control.hpp>

// Qt
#include <QObject>

namespace custom_rviz2_plugins
{

  class GoalTool : public rviz_default_plugins::tools::PoseTool
  {
    Q_OBJECT

  public:
    GoalTool();
    ~GoalTool() override;

    void onInitialize() override;

  protected:
    void onPoseSet(double x, double y, double theta) override;
  };

}  // namespace custom_rviz2_plugins

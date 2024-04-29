#pragma once
#include <QObject>

namespace custom_rviz2_plugins
{

/// Class to set and update goal pose by emitting signal
class GoalPoseUpdater : public QObject
{
  Q_OBJECT

public:
  GoalPoseUpdater() {}
  ~GoalPoseUpdater() {}

  void setGoal(const double x, const double y, const double theta, const QString frame)
  {
    emit updateGoal(x, y, theta, frame);
  }

signals:
  void updateGoal(const double x, const double y, const double theta, const QString frame);
};

}  // namespace custom_rviz2_plugins

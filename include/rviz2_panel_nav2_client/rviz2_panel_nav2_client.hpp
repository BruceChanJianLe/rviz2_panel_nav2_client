#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_util/geometry_utils.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>

#include "rviz2_panel_nav2_client/const_defs.hpp"

// RVIZ2
#include <rviz_common/panel.hpp>
// Qt
#include <QtWidgets>
#include <ui_rviz2_panel_nav2_client.h>

#include <fmt/core.h>
#include <fmt/color.h>

namespace custom_rviz2_plugins
{
  class Rviz2PanelNav2Client : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit Rviz2PanelNav2Client(QWidget *parent = nullptr);
    ~Rviz2PanelNav2Client();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  private Q_SLOTS:
    void on_updateGoal(const double x, const double y, const double yaw, const QString frame_id);
    void on_lineEditPlannerID_editingFinished();
    void on_lineEditControllerID_editingFinished();
    void on_lineEditGoalCheckerID_editingFinished();
    void on_pushButtonNavToGoal_clicked();
    void on_pushButtonCancelNavToGoal_clicked();
    void on_pushButtonComputePathToGoal_clicked();

  private:
    std::unique_ptr<Ui::gui> ui_;
    std::string controller_id_, planner_id_, goal_checker_id_;
    std::unique_ptr<std::thread> callback_handler_thread_;

    /// Path Action Callbacks
    void pathGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>>);
    void pathFeedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>>, const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback>);
    void pathResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult);

    /// Nav Action Callbacks
    void navGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>>);
    void navFeedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>>, const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>);
    void navResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult);

  protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr path_ac_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr nav_ac_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    nav_msgs::msg::Path goal_path_;

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions path_opts_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions nav_opts_;
  };
} // custom_rviz2_plugins

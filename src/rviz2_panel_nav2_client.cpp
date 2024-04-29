#include "rviz2_panel_nav2_client/rviz2_panel_nav2_client.hpp"
#include "rviz2_panel_nav2_client/goal_common.hpp"

namespace custom_rviz2_plugins
{
  GoalPoseUpdater GoalUpdater;

  Rviz2PanelNav2Client::Rviz2PanelNav2Client(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui>())
    , controller_id_{""}
    , planner_id_{""}
    , node_{nullptr}
    , path_ac_{nullptr}
    , nav_ac_{nullptr}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz2_panel_nav2_client_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    // Create action clients
    path_ac_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      node_, "compute_path_to_pose");

    nav_ac_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
      node_, "follow_path");

    // Prepare msg
    goal_pose_.pose.orientation.w = 1.0;

    // Update action client callbacks
    path_opts_ = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    path_opts_.goal_response_callback = [this](const auto gh){ pathGoalResponseCallback(gh); };
    path_opts_.feedback_callback = [this](auto gh, const auto fd){ pathFeedbackCallback(gh, fd); };
    path_opts_.result_callback = [this](const auto res){ pathResultCallback(res); };

    nav_opts_ = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
    nav_opts_.goal_response_callback = [this](const auto gh){ navGoalResponseCallback(gh); };
    nav_opts_.feedback_callback = [this](auto gh, const auto fd){ navFeedbackCallback(gh, fd); };
    nav_opts_.result_callback = [this](const auto res){ navResultCallback(res); };

    // Spin callback handler
    callback_handler_thread_ = std::make_unique<std::thread>(
        [this]()
        {
          rclcpp::Rate r{10};
          while(rclcpp::ok())
          {
            rclcpp::spin_some(node_);
            r.sleep();
          }
        }
      );

    connect(&GoalUpdater, SIGNAL(updateGoal(const double, const double, const double, const QString)),
            this, SLOT(on_updateGoal(const double, const double, const double, const QString)));
  }

  Rviz2PanelNav2Client::~Rviz2PanelNav2Client()
  {
  }

  void Rviz2PanelNav2Client::load(const rviz_common::Config &config)
  {
    Panel::load(config);
    if (auto rpnc_config = config.mapGetChild(CONFIG_NAMESPACE); rpnc_config.isValid())
    {
      if (QVariant planner_id {QVariant::String}; rpnc_config.mapGetValue({"planner_id"}, &planner_id))
      {
        ui_->lineEditPlannerID->setText(planner_id.toString());
        planner_id_ = planner_id.toString().toStdString();
      }

      if (QVariant controller_id {QVariant::String}; rpnc_config.mapGetValue({"controller_id"}, &controller_id))
      {
        ui_->lineEditControllerID->setText(controller_id.toString());
        controller_id_ = controller_id.toString().toStdString();
      }

      if (QVariant goal_checker_id {QVariant::String}; rpnc_config.mapGetValue({"goal_checker_id"}, &goal_checker_id))
      {
        ui_->lineEditGoalCheckerID->setText(goal_checker_id.toString());
        goal_checker_id_ = goal_checker_id.toString().toStdString();
      }
    }
  }

  void Rviz2PanelNav2Client::save(rviz_common::Config config) const
  {
    Panel::save(config);
    rviz_common::Config rpnc_config = config.mapMakeChild(CONFIG_NAMESPACE);
    rpnc_config.mapSetValue(QString{"planner_id"}, ui_->lineEditPlannerID->text());
    rpnc_config.mapSetValue(QString{"controller_id"}, ui_->lineEditControllerID->text());
    rpnc_config.mapSetValue(QString{"goal_checker_id"}, ui_->lineEditGoalCheckerID->text());
  }

  void Rviz2PanelNav2Client::on_updateGoal(const double x, const double y, const double yaw, const QString frame_id)
  {
    goal_pose_.pose.position.x = x;
    goal_pose_.pose.position.y = y;
    goal_pose_.pose.position.z = 0.0;
    goal_pose_.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    goal_pose_.header.frame_id = frame_id.toStdString();

    RCLCPP_INFO_STREAM(node_->get_logger(),
        fmt::format(fmt::fg(fmt::color::yellow), "Goal: x: {}, y: {}, theta: {} in frame '{}'", x, y, yaw, frame_id.toStdString()));

    ui_->labelActionStatus->setText(QString::fromStdString(fmt::format(STATUS_MSG, x, y, yaw)));
  }

  void Rviz2PanelNav2Client::on_lineEditControllerID_editingFinished()
  {
    controller_id_ = ui_->lineEditControllerID->text().toStdString();
  }

  void Rviz2PanelNav2Client::on_lineEditGoalCheckerID_editingFinished()
  {
    goal_checker_id_ = ui_->lineEditGoalCheckerID->text().toStdString();
  }

  void Rviz2PanelNav2Client::on_lineEditPlannerID_editingFinished()
  {
    planner_id_ = ui_->lineEditPlannerID->text().toStdString();
  }

  void Rviz2PanelNav2Client::on_pushButtonNavToGoal_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(),
        fmt::format(fmt::fg(fmt::color::green), "Navigating to goal!");
        );

    if (!nav_ac_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
          "follow_path action server is not available."
          " Is the initial pose set?");
      return;
    }
    
    if (goal_path_.poses.empty())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
          "store path is empty, have path been computed?");
      return;
    }

    nav2_msgs::action::FollowPath::Goal path_to_goal;
    path_to_goal.path = goal_path_;
    path_to_goal.controller_id = controller_id_;
    path_to_goal.goal_checker_id = goal_checker_id_;
    nav_ac_->async_send_goal(path_to_goal, nav_opts_);
  }

  void Rviz2PanelNav2Client::on_pushButtonCancelNavToGoal_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(),
        fmt::format(fmt::fg(fmt::color::red), "Cancel navigation to goal!");
        );
        nav_ac_->async_cancel_all_goals();
  }

  void Rviz2PanelNav2Client::on_pushButtonComputePathToGoal_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(),
        fmt::format(fmt::fg(fmt::color::light_sea_green), "Computing path to goal!");
        );

    if (!path_ac_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
          "compute_to_pose action server is not available."
          " Is the initial pose set?");
      return;
    }

    nav2_msgs::action::ComputePathToPose::Goal compute_path_to_goal;
    compute_path_to_goal.use_start = false;
    compute_path_to_goal.planner_id = planner_id_;
    compute_path_to_goal.goal = goal_pose_;
    compute_path_to_goal.goal.header.stamp = rclcpp::Clock().now();
    path_ac_->async_send_goal(compute_path_to_goal, path_opts_);
  }

  void Rviz2PanelNav2Client::pathGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>> gh)
  {
    if (!gh)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Compute path to goal was rejected!");
    }
    else
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Compute path to goal was accepted, waiting for server result!");
    }
  }

  void Rviz2PanelNav2Client::pathFeedbackCallback([[maybe_unused]] std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>> gh, [[maybe_unused]] const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback> fb)
  {
  }

  void Rviz2PanelNav2Client::pathResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult res)
  {
    switch (res.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "Finished compute path to goal, "
            "storing path for navigation.");
        goal_path_ = res.result->path;
        return;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Compute path to goal was aborted.");
        return;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Compute path to goal was cancelled.");
        return;

      default:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown result code!");
        return;
    }
  }

  void Rviz2PanelNav2Client::navGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>> gh)
  {
    if (!gh)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Follow path to goal was rejected!");
    }
    else
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Follow path to goal was accepted, waiting for server result!");
    }
  }

  void Rviz2PanelNav2Client::navFeedbackCallback([[maybe_unused]] std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>> gh, const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> fb)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(),
        fmt::format(fmt::fg(fmt::color::gold), "Distance to goal: {:.4}, speed: {:.4}", fb->distance_to_goal, fb->speed)
        );
  }

  void Rviz2PanelNav2Client::navResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult res)
  {
    switch (res.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "Successfully follow path to goal, "
            "clearing stored path for navigation.");
        goal_path_ = nav_msgs::msg::Path{};
        return;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Follow path to goal was aborted.");
        return;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Follow path to goal was cancelled.");
        return;

      default:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown result code!");
        return;
    }
  }

} // custom_rviz2_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(custom_rviz2_plugins::Rviz2PanelNav2Client, rviz_common::Panel)

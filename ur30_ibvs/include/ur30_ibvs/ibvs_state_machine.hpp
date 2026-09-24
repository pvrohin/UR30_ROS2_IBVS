#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ur30_ibvs/edge_tracker.hpp"
#include "ur30_ibvs/ibvs_controller.hpp"
#include "ur30_ibvs/panel_detector.hpp"
#include "ur30_ibvs/panel_geometry.hpp"
#include "ur30_ibvs/servo_supervisor.hpp"

namespace ur30_ibvs
{

enum class State
{
  WAIT_READY,      // waiting for camera info, TF, the trajectory action and both services
  GO_TO_SURVEY,    // trajectory controller moves the arm to the survey pose
  SEARCH,          // hover and detect the panel
  SWITCH_CONTROL,  // hand the joints from the trajectory to the velocity controller
  APPROACH,        // look-then-move: steer to the point above the chosen long edge
  SERVO,           // vpMeLine + vpServo on that edge, holding the standoff
  FAILED           // zero twists, stay put
};

const char * toString(State state);

// Detects the panel, approaches one of its long edges and then servos on that edge,
// sending Cartesian twists to MoveIt Servo.
class IbvsStateMachine : public rclcpp::Node
{
public:
  IbvsStateMachine();

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ServoCommandType = moveit_msgs::srv::ServoCommandType;

  struct Params
  {
    std::string image_topic, camera_info_topic, twist_topic;
    std::string world_frame, command_frame, camera_frame;
    std::string jtc_action, position_controller, velocity_controller;
    std::vector<std::string> joint_names;
    std::vector<double> survey_joint_positions;
    double survey_duration_sec, control_rate_hz, stale_command_sec;

    double panel_length, panel_width, panel_top_height;
    PanelDetectorParams detector;
    EdgeTrackerParams edge;
    int consecutive_detections;
    double search_timeout_sec, depth_consistency_m, size_tolerance_m;

    double follow_edge_sign, servo_standoff;
    double approach_kp, approach_max_speed, approach_max_rot_speed;
    double approach_tolerance, approach_rot_tolerance, approach_timeout_sec;

    double servo_lambda, desired_rho, standoff_kp, standoff_max_speed, slide_speed;
    double converged_threshold;
    // Give up (FAILED) when the edge has been missing this long, or no image has
    // arrived for this long, while servoing.
    double edge_lost_timeout_sec, image_timeout_sec;
  };

  Params loadParams();
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void onTimer();

  // State handlers.
  void tickWaitReady();
  void sendSurveyGoal();
  void handleSearch(const cv::Mat & bgr);
  void beginSwitchControl();
  void tickApproach();
  void handleServo(const cv::Mat & bgr);
  bool seedEdgeTracker(const cv::Mat & bgr, const cv::Matx44d & world_T_cam);

  void transitionTo(State next);
  void fail(const std::string & reason);
  void publishTwist(const std::array<double, 6> & twist_camera_frame);

  std::optional<cv::Matx44d> lookup(const std::string & target, const std::string & source);
  // Distance along the optical axis to the panel's top-face plane.
  std::optional<double> distanceToPanelPlane(const cv::Matx44d & world_T_cam) const;

  const Params params_;
  State state_ = State::WAIT_READY;
  ServoSupervisor supervisor_;

  std::unique_ptr<PanelDetector> detector_;
  std::unique_ptr<EdgeTracker> tracker_;
  std::unique_ptr<IbvsController> controller_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr jtc_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<ServoCommandType>::SharedPtr servo_type_client_;

  std::optional<cv::Matx33d> K_;
  std::optional<cv::Matx44d> tool_T_cam_;  // fixed: the camera pose in the command frame

  // SEARCH
  int good_detections_ = 0;
  rclcpp::Time search_started_{0, 0, RCL_ROS_TIME};

  // APPROACH
  PanelEdge edge_{};
  cv::Matx44d approach_target_ = cv::Matx44d::eye();
  rclcpp::Time approach_started_{0, 0, RCL_ROS_TIME};
  bool velocity_active_ = false;

  // SERVO
  bool tracker_active_ = false;
  bool converged_reported_ = false;
  std::array<double, 6> servo_command_{};
  rclcpp::Time servo_command_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace ur30_ibvs

#include "ur30_ibvs/ibvs_state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/visual_features/vpFeatureLine.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace ur30_ibvs
{
namespace
{
std::string seconds(double s)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f s", s);
  return text;
}

template<typename... Args>
std::string format(const char * fmt, Args... args)
{
  char text[160];
  std::snprintf(text, sizeof(text), fmt, args...);
  return text;
}

const cv::Scalar kGreen(0, 220, 0);
const cv::Scalar kRed(0, 0, 255);
const cv::Scalar kCyan(255, 255, 0);
const cv::Scalar kYellow(0, 220, 255);

void label(cv::Mat & image, const std::string & text, int row)
{
  const cv::Point at(20, 40 + 34 * row);
  cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
  cv::putText(
    image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
}

// Draw ViSP's line  i cos(theta) + j sin(theta) = rho  (i = row, j = column) across the image.
void drawPixelLine(cv::Mat & image, double rho, double theta, const cv::Scalar & color)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  const double w = image.cols;
  const auto at = [](double x, double y) {return cv::Point(cvRound(x), cvRound(y));};
  if (std::abs(c) > 1e-6) {
    cv::line(image, at(0.0, rho / c), at(w, (rho - w * s) / c), color, 2, cv::LINE_AA);
  } else {
    cv::line(image, at(rho / s, 0.0), at(rho / s, image.rows), color, 2, cv::LINE_AA);
  }
}
}  // namespace

const char * toString(State state)
{
  switch (state) {
    case State::WAIT_READY: return "WAIT_READY";
    case State::GO_TO_SURVEY: return "GO_TO_SURVEY";
    case State::SEARCH: return "SEARCH";
    case State::SWITCH_CONTROL: return "SWITCH_CONTROL";
    case State::APPROACH: return "APPROACH";
    case State::SERVO: return "SERVO";
    case State::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

IbvsStateMachine::Params IbvsStateMachine::loadParams()
{
  const auto str = [this](const char * n, const char * d) {
      return declare_parameter<std::string>(n, d);
    };
  const auto num = [this](const char * n, double d) {return declare_parameter<double>(n, d);};
  const auto integer = [this](const char * n, int d) {return declare_parameter<int>(n, d);};

  Params p;
  p.image_topic = str("image_topic", "/camera/color/image_raw");
  p.camera_info_topic = str("camera_info_topic", "/camera/color/camera_info");
  p.twist_topic = str("twist_topic", "/servo_node/delta_twist_cmds");
  p.world_frame = str("world_frame", "world");
  p.command_frame = str("command_frame", "tool0");
  p.camera_frame = str("camera_frame", "camera_color_optical_frame");
  p.jtc_action = str("jtc_action", "/joint_trajectory_controller/follow_joint_trajectory");
  p.position_controller = str("position_controller", "joint_trajectory_controller");
  p.velocity_controller = str("velocity_controller", "forward_velocity_controller");
  p.joint_names = declare_parameter<std::vector<std::string>>(
    "joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
      "wrist_2_joint", "wrist_3_joint"});
  p.survey_joint_positions = declare_parameter<std::vector<double>>(
    "survey_joint_positions", {-0.2619, -1.3611, 0.9311, -1.1408, -1.5708, -0.2619});
  p.survey_duration_sec = num("survey_duration_sec", 6.0);
  p.control_rate_hz = num("control_rate_hz", 50.0);
  p.stale_command_sec = num("stale_command_sec", 0.25);
  p.start_delay_sec = num("start_delay_sec", 5.0);

  p.panel_length = num("panel_length", 0.9);
  p.panel_width = num("panel_width", 0.55);
  p.panel_top_height = num("panel_top_height", 0.805);
  p.detector.panel_length = p.panel_length;
  p.detector.panel_width = p.panel_width;
  p.detector.dark_threshold = integer("dark_threshold", 150);
  p.detector.min_area_fraction = num("min_area_fraction", 0.35);
  p.detector.max_area_fraction = num("max_area_fraction", 0.85);
  p.detector.min_rectangularity = num("min_rectangularity", 0.85);
  p.detector.aspect_tolerance = num("aspect_tolerance", 0.2);
  p.detector.max_reprojection_px = num("max_reprojection_px", 3.0);
  p.detector.expected_x_axis_angle_deg = num("expected_x_axis_angle_deg", 0.0);
  p.consecutive_detections = integer("consecutive_detections", 10);
  p.search_timeout_sec = num("search_timeout_sec", 30.0);
  p.depth_consistency_m = num("depth_consistency_m", 0.03);
  p.size_tolerance_m = num("size_tolerance_m", 0.03);

  p.follow_edge_sign = num("follow_edge_sign", 1.0);
  p.servo_standoff = num("servo_standoff", 0.20);
  p.approach_kp = num("approach_kp", 1.0);
  p.approach_max_speed = num("approach_max_speed", 0.05);
  p.approach_max_rot_speed = num("approach_max_rot_speed", 0.1);
  p.approach_tolerance = num("approach_tolerance", 0.01);
  p.approach_rot_tolerance = num("approach_rot_tolerance", 0.03);
  p.approach_timeout_sec = num("approach_timeout_sec", 90.0);

  p.servo_lambda = num("servo_lambda", 0.6);
  p.desired_rho = num("desired_rho", 0.0);
  p.standoff_kp = num("standoff_kp", 1.0);
  p.standoff_max_speed = num("standoff_max_speed", 0.03);
  p.slide_speed = num("slide_speed", 0.0);
  p.converged_threshold = num("converged_threshold", 0.005);
  p.edge_lost_timeout_sec = num("edge_lost_timeout_sec", 2.0);
  p.image_timeout_sec = num("image_timeout_sec", 2.0);

  p.edge.range = static_cast<unsigned int>(integer("me_range", 10));
  p.edge.sample_step = num("me_sample_step", 5.0);
  p.edge.threshold = num("me_threshold", 20.0);
  p.edge.snap_radius = num("edge_snap_radius", 50.0);
  p.edge.min_tracked_fraction = num("edge_min_tracked_fraction", 0.5);
  return p;
}

IbvsStateMachine::IbvsStateMachine()
: Node("ibvs_state_machine"), params_(loadParams()),
  supervisor_(params_.image_timeout_sec, params_.edge_lost_timeout_sec)
{
  detector_ = std::make_unique<PanelDetector>(params_.detector);
  tracker_ = std::make_unique<EdgeTracker>(params_.edge);
  IbvsControllerParams controller_params;
  controller_params.lambda = params_.servo_lambda;
  controller_params.desired_rho = params_.desired_rho;
  controller_ = std::make_unique<IbvsController>(controller_params);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    params_.camera_info_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {onCameraInfo(msg);});
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    params_.image_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(params_.twist_topic, 10);
  state_pub_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
  debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);

  jtc_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, params_.jtc_action);
  switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller");
  servo_type_client_ = create_client<ServoCommandType>("/servo_node/switch_command_type");

  timer_ = create_timer(
    std::chrono::duration<double>(1.0 / params_.control_rate_hz), [this]() {onTimer();});

  transitionTo(State::WAIT_READY);
}

void IbvsStateMachine::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  const auto & k = msg->k;
  K_ = cv::Matx33d(k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]);
}

void IbvsStateMachine::onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  supervisor_.onImage(now().seconds());
  image_header_ = msg->header;
  if (!K_ || (state_ != State::SEARCH && state_ != State::SERVO)) {
    return;
  }
  cv_bridge::CvImageConstPtr image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
    return;
  }
  if (state_ == State::SEARCH) {
    handleSearch(image->image);
  } else {
    handleServo(image->image);
  }
}

void IbvsStateMachine::onTimer()
{
  switch (state_) {
    case State::WAIT_READY:
      tickWaitReady();
      break;
    case State::SEARCH:
      if ((now() - search_started_).seconds() > params_.search_timeout_sec) {
        fail("the panel was not found within the search timeout");
      }
      break;
    case State::APPROACH:
      tickApproach();
      break;
    case State::SERVO: {
        const double t = now().seconds();
        switch (supervisor_.check(t)) {
          case SupervisorVerdict::CAMERA_LOST:
            fail("the camera feed stopped: no image for " + seconds(supervisor_.secondsSinceImage(t)));
            return;
          case SupervisorVerdict::EDGE_LOST:
            fail("the edge was not found for " + seconds(supervisor_.secondsEdgeMissing(t)));
            return;
          case SupervisorVerdict::OK:
            break;
        }
        const bool fresh =
          (now() - servo_command_stamp_).seconds() < params_.stale_command_sec;
        publishTwist(tracker_active_ && fresh ? servo_command_ : std::array<double, 6>{});
        break;
      }
    case State::FAILED:
      if (velocity_active_) {
        publishTwist({});
      }
      break;
    default:
      break;
  }
}

void IbvsStateMachine::tickWaitReady()
{
  std::string missing;
  const auto need = [&missing](bool ok, const char * what) {
      if (!ok) {
        missing += std::string(missing.empty() ? "" : ", ") + what;
      }
    };
  need(K_.has_value(), "camera_info");
  need(lookup(params_.world_frame, params_.camera_frame).has_value(), "TF world->camera");
  if (!tool_T_cam_) {
    tool_T_cam_ = lookup(params_.command_frame, params_.camera_frame);
  }
  need(tool_T_cam_.has_value(), "TF command frame->camera");
  need(jtc_client_->action_server_is_ready(), "trajectory action");
  need(switch_client_->service_is_ready(), "switch_controller service");
  need(servo_type_client_->service_is_ready(), "servo switch_command_type service");
  if (!missing.empty()) {
    ready_since_.reset();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for: %s", missing.c_str());
    return;
  }
  if (!ready_since_) {
    ready_since_ = now();
    RCLCPP_INFO(
      get_logger(), "everything is up; the robot starts moving in %.1f s", params_.start_delay_sec);
  }
  if ((now() - *ready_since_).seconds() < params_.start_delay_sec) {
    return;
  }
  sendSurveyGoal();
}

void IbvsStateMachine::sendSurveyGoal()
{
  transitionTo(State::GO_TO_SURVEY);
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = params_.joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = params_.survey_joint_positions;
  point.time_from_start = rclcpp::Duration::from_seconds(params_.survey_duration_sec);
  goal.trajectory.points.push_back(point);

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
  options.goal_response_callback = [this](const GoalHandle::SharedPtr & handle) {
      if (!handle) {
        fail("the survey trajectory goal was rejected");
      }
    };
  options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      if (state_ != State::GO_TO_SURVEY) {
        return;
      }
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        fail("the survey trajectory did not succeed");
        return;
      }
      good_detections_ = 0;
      search_started_ = now();
      transitionTo(State::SEARCH);
    };
  jtc_client_->async_send_goal(goal, options);
}

void IbvsStateMachine::handleSearch(const cv::Mat & bgr)
{
  const auto world_T_cam = lookup(params_.world_frame, params_.camera_frame);
  if (!world_T_cam) {
    return;
  }
  const auto detection = detector_->detect(bgr, *K_);
  publishSearchDebug(bgr, detection ? &*detection : nullptr);
  if (!detection) {
    good_detections_ = 0;
    return;
  }

  // Locate the panel on the known table plane rather than trusting the PnP tilt,
  // then check it against what we know about the panel and the camera height.
  const auto panel = panelPoseFromCorners(
    *K_, *world_T_cam, detection->corners_px, params_.panel_top_height);
  const auto distance = distanceToPanelPlane(*world_T_cam);
  if (!panel || !distance) {
    good_detections_ = 0;
    return;
  }
  const bool size_ok = std::abs(panel->length - params_.panel_length) < params_.size_tolerance_m &&
    std::abs(panel->width - params_.panel_width) < params_.size_tolerance_m;
  const bool depth_ok = std::abs(detection->tvec[2] - *distance) < params_.depth_consistency_m;
  if (!size_ok || !depth_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "rejecting detection: size %.3f x %.3f m (expected %.3f x %.3f), PnP depth %.3f m vs "
      "TF %.3f m",
      panel->length, panel->width, params_.panel_length, params_.panel_width,
      detection->tvec[2], *distance);
    good_detections_ = 0;
    return;
  }

  if (++good_detections_ < params_.consecutive_detections) {
    return;
  }
  edge_ = longEdge(*panel, params_.panel_width, params_.follow_edge_sign);
  approach_target_ = approachTarget(edge_, params_.servo_standoff, *world_T_cam);
  RCLCPP_INFO(
    get_logger(),
    "panel found: centre (%.3f, %.3f, %.3f), yaw %.1f deg; following the edge at (%.3f, %.3f)",
    panel->center[0], panel->center[1], panel->center[2], panel->yaw * 180.0 / M_PI,
    edge_.midpoint[0], edge_.midpoint[1]);
  beginSwitchControl();
}

void IbvsStateMachine::beginSwitchControl()
{
  transitionTo(State::SWITCH_CONTROL);
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = {params_.velocity_controller};
  request->deactivate_controllers = {params_.position_controller};
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout.sec = 5;

  switch_client_->async_send_request(
    request, [this](rclcpp::Client<SwitchController>::SharedFuture future) {
      if (!future.get()->ok) {
        fail("the controller manager rejected the switch to the velocity controller");
        return;
      }
      velocity_active_ = true;
      auto type = std::make_shared<ServoCommandType::Request>();
      type->command_type = ServoCommandType::Request::TWIST;
      servo_type_client_->async_send_request(
        type, [this](rclcpp::Client<ServoCommandType>::SharedFuture f) {
          if (!f.get()->success) {
            fail("Servo refused the TWIST command type");
            return;
          }
          approach_started_ = now();
          transitionTo(State::APPROACH);
        });
    });
}

void IbvsStateMachine::tickApproach()
{
  const auto world_T_cam = lookup(params_.world_frame, params_.camera_frame);
  if (!world_T_cam) {
    publishTwist({});
    return;
  }
  if ((now() - approach_started_).seconds() > params_.approach_timeout_sec) {
    fail("the approach did not reach the target within its timeout");
    return;
  }
  const auto command = approachTwist(
    *world_T_cam, approach_target_, params_.approach_kp, params_.approach_kp,
    params_.approach_max_speed, params_.approach_max_rot_speed);
  if (command.position_error < params_.approach_tolerance &&
    command.rotation_error < params_.approach_rot_tolerance)
  {
    publishTwist({});
    tracker_active_ = false;
    converged_reported_ = false;
    supervisor_.start(now().seconds());
    transitionTo(State::SERVO);
    return;
  }
  publishTwist(command.twist);
}

bool IbvsStateMachine::seedEdgeTracker(const cv::Mat & bgr, const cv::Matx44d & world_T_cam)
{
  const auto distance = distanceToPanelPlane(world_T_cam);
  if (!distance) {
    return false;
  }
  // Seeds well inside the visible part of the edge, projected from the panel pose.
  // They are only approximate: init() snaps them onto the real edge.
  const double half_span = 0.6 * *distance * (*K_)(0, 2) / (*K_)(0, 0);
  const auto a = projectToPixel(*K_, world_T_cam, edge_.midpoint - edge_.direction * half_span);
  const auto b = projectToPixel(*K_, world_T_cam, edge_.midpoint + edge_.direction * half_span);
  if (!a || !b || !tracker_->init(bgr, *a, *b)) {
    return false;
  }
  const vpCameraParameters cam((*K_)(0, 0), (*K_)(1, 1), (*K_)(0, 2), (*K_)(1, 2));
  vpFeatureLine s;
  tracker_->feature(cam, s);
  controller_->matchPolarity(s.getTheta());
  return true;
}

void IbvsStateMachine::handleServo(const cv::Mat & bgr)
{
  const auto world_T_cam = lookup(params_.world_frame, params_.camera_frame);
  if (!world_T_cam) {
    return;
  }
  const auto distance = distanceToPanelPlane(*world_T_cam);
  if (!distance) {
    return;
  }

  const double t = now().seconds();
  if (!tracker_active_) {
    servo_command_ = {};
    if (seedEdgeTracker(bgr, *world_T_cam)) {
      tracker_active_ = true;
      supervisor_.onEdgeLocked();
      RCLCPP_INFO(get_logger(), "edge locked (%d tracked points)", tracker_->trackedPoints());
    } else {
      // Retry on every frame; the supervisor gives up once this has lasted too long.
      supervisor_.onEdgeMissing(t);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "could not find the edge (missing for %.1f s, giving up after %.1f s)",
        supervisor_.secondsEdgeMissing(t), params_.edge_lost_timeout_sec);
    }
    publishServoDebug(bgr, *distance, nullptr);
    return;
  }

  if (!tracker_->track(bgr)) {
    tracker_active_ = false;
    servo_command_ = {};
    converged_reported_ = false;
    supervisor_.onEdgeMissing(t);
    RCLCPP_WARN(get_logger(), "lost the edge; re-seeding from the panel pose");
    publishServoDebug(bgr, *distance, nullptr);
    return;
  }
  supervisor_.onEdgeLocked();

  const vpCameraParameters cam((*K_)(0, 0), (*K_)(1, 1), (*K_)(0, 2), (*K_)(1, 2));
  vpFeatureLine s;
  tracker_->feature(cam, s);
  auto command = controller_->computeTwist(s.getRho(), s.getTheta(), *distance);
  // One line cannot see the standoff or the position along the edge, so those come
  // from the TF distance and the configured slide speed.
  command[0] = params_.slide_speed;
  const double standoff_error = *distance - params_.servo_standoff;
  command[2] = std::clamp(
    params_.standoff_kp * standoff_error, -params_.standoff_max_speed, params_.standoff_max_speed);
  servo_command_ = command;
  servo_command_stamp_ = now();
  publishServoDebug(bgr, *distance, &command);

  const bool converged = controller_->errorNorm() < params_.converged_threshold &&
    std::abs(standoff_error) < 0.005;
  if (converged && !converged_reported_) {
    RCLCPP_INFO(
      get_logger(), "converged: feature error %.4f, standoff %.3f m", controller_->errorNorm(),
      *distance);
  }
  converged_reported_ = converged;
}

bool IbvsStateMachine::debugWanted() const {return debug_pub_->get_subscription_count() > 0;}

void IbvsStateMachine::publishDebugImage(const cv::Mat & annotated)
{
  debug_pub_->publish(*cv_bridge::CvImage(image_header_, "bgr8", annotated).toImageMsg());
}

void IbvsStateMachine::publishSearchDebug(const cv::Mat & bgr, const PanelDetection * detection)
{
  if (!debugWanted()) {
    return;
  }
  cv::Mat image = bgr.clone();
  if (detection) {
    std::vector<cv::Point> outline;
    for (const auto & c : detection->corners_px) {
      outline.emplace_back(cvRound(c.x), cvRound(c.y));
    }
    cv::polylines(image, outline, true, kGreen, 3, cv::LINE_AA);
    for (size_t i = 0; i < outline.size(); ++i) {
      cv::circle(image, outline[i], 9, kYellow, -1, cv::LINE_AA);
      cv::putText(
        image, std::to_string(i), outline[i] + cv::Point(12, -12), cv::FONT_HERSHEY_SIMPLEX, 0.9,
        kYellow, 2, cv::LINE_AA);
    }
    label(image, format("SEARCH: panel found (%d of %d frames)", good_detections_,
      params_.consecutive_detections), 0);
  } else {
    label(image, "SEARCH: panel not found", 0);
  }
  label(image, "green: detected panel outline, corners 0-3", 1);
  publishDebugImage(image);
}

void IbvsStateMachine::publishServoDebug(
  const cv::Mat & bgr, double distance, const std::array<double, 6> * command)
{
  if (!debugWanted()) {
    return;
  }
  cv::Mat image = bgr.clone();
  const double t = now().seconds();
  if (tracker_active_) {
    // The desired line is horizontal at normalised y = desired_rho / sin(theta*), and
    // sin(theta*) is the polarity sign (+1 dark below the edge, -1 above). It is drawn
    // thick and underneath, and the tracked line thin on top, so both stay visible when
    // they coincide at convergence.
    const vpCameraParameters cam((*K_)(0, 0), (*K_)(1, 1), (*K_)(0, 2), (*K_)(1, 2));
    vpFeatureLine s;
    tracker_->feature(cam, s);
    const double polarity = s.getTheta() >= 0.0 ? 1.0 : -1.0;
    const double row = (*K_)(1, 2) + (*K_)(1, 1) * params_.desired_rho / polarity;
    cv::line(
      image, cv::Point(0, cvRound(row)), cv::Point(image.cols, cvRound(row)), kCyan, 7, cv::LINE_AA);
    for (const auto & p : tracker_->sitePixels()) {
      cv::circle(image, cv::Point(cvRound(p.x), cvRound(p.y)), 3, kGreen, -1, cv::LINE_AA);
    }
    const cv::Vec2d rho_theta = tracker_->rhoTheta();
    drawPixelLine(image, rho_theta[0], rho_theta[1], kRed);
    label(image, format("SERVO: edge locked, %d points", tracker_->trackedPoints()), 0);
  } else {
    label(image, format("SERVO: edge missing for %.1f s", supervisor_.secondsEdgeMissing(t)), 0);
  }
  label(image, format("standoff %.3f m (target %.3f m)", distance, params_.servo_standoff), 1);
  if (command) {
    label(image, format("vy %+.4f m/s  wz %+.4f rad/s  error %.4f", (*command)[1], (*command)[5],
      controller_->errorNorm()), 2);
  }
  label(image, "red: tracked edge   cyan: desired edge   green: moving-edge points", 3);
  publishDebugImage(image);
}

void IbvsStateMachine::transitionTo(State next)
{
  state_ = next;
  RCLCPP_INFO(get_logger(), "state -> %s", toString(next));
  std_msgs::msg::String msg;
  msg.data = toString(next);
  state_pub_->publish(msg);
}

void IbvsStateMachine::fail(const std::string & reason)
{
  RCLCPP_ERROR(get_logger(), "FAILED: %s", reason.c_str());
  servo_command_ = {};
  transitionTo(State::FAILED);
}

void IbvsStateMachine::publishTwist(const std::array<double, 6> & twist_camera_frame)
{
  if (!tool_T_cam_) {
    return;
  }
  const auto t = twistInParent(*tool_T_cam_, twist_camera_frame);
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = params_.command_frame;
  msg.twist.linear.x = t[0];
  msg.twist.linear.y = t[1];
  msg.twist.linear.z = t[2];
  msg.twist.angular.x = t[3];
  msg.twist.angular.y = t[4];
  msg.twist.angular.z = t[5];
  twist_pub_->publish(msg);
}

std::optional<cv::Matx44d> IbvsStateMachine::lookup(
  const std::string & target, const std::string & source)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero).transform;
    const double n = std::sqrt(
      tf.rotation.x * tf.rotation.x + tf.rotation.y * tf.rotation.y +
      tf.rotation.z * tf.rotation.z + tf.rotation.w * tf.rotation.w);
    const double x = tf.rotation.x / n, y = tf.rotation.y / n, z = tf.rotation.z / n,
      w = tf.rotation.w / n;
    return cv::Matx44d(
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), tf.translation.x,
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), tf.translation.y,
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), tf.translation.z,
      0, 0, 0, 1);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<double> IbvsStateMachine::distanceToPanelPlane(const cv::Matx44d & world_T_cam) const
{
  // z component of the optical axis in the world; about -1 when looking straight down.
  const double axis_z = rotationOf(world_T_cam)(2, 2);
  if (axis_z > -0.5) {
    return std::nullopt;
  }
  return (translationOf(world_T_cam)[2] - params_.panel_top_height) / -axis_z;
}

}  // namespace ur30_ibvs

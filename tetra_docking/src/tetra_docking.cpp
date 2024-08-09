// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <mutex>
#include <memory>
#include <thread>
#include <future>
#include <chrono>
#include <string>
#include <algorithm>
#include <utility>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <tetra_docking/DockRobotAction.h>
#include <tetra_docking/UndockRobotAction.h>


#include "angles/angles.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"


using namespace std;

typedef tetra_docking::DockRobotAction DockRobot;
typedef tetra_docking::UndockRobotAction UndockRobot;

using DockingActionServer = actionlib::SimpleActionServer<DockRobot>;
using UndockingActionServer = actionlib::SimpleActionServer<UndockRobot>;


bool preempt_requested_{false};


// Frequency to run control loops
double controller_frequency = 50.0;
// Timeout for initially detecting the charge dock
double initial_perception_timeout = 5.0;
// Timeout after making contact with dock for charging to start
// If this is exceeded, the robot returns to the staging pose and retries
double wait_charge_timeout = 5.0;
// Timeout to approach into the dock and reset its approach is retrying
double dock_approach_timeout = 30.0;
// When undocking, these are the tolerances for arriving at the staging pose
double undock_linear_tolerance, undock_angular_tolerance = 0.05;
// Maximum number of times the robot will return to staging pose and retry docking
int max_retries, num_retries = 3;
// This is the root frame of the robot - typically "base_link"
std::string base_frame = "base_link";
// This is our fixed frame for controlling - typically "odom"
std::string fixed_frame = "odom";
// Does the robot drive backwards onto the dock? Default is forwards
bool dock_backwards = false;
// The tolerance to the dock's staging pose not requiring navigation
double dock_prestaging_tolerance = 0.5;


// Mutex for dynamic parameters and dock database
std::shared_ptr<std::mutex> mutex_;

// This is a class member so it can be accessed in publish feedback
ros::Time action_start_time_;


std::string curr_dock_type_;

std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

double k_phi_, k_delta_, beta_, lambda_;
double slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_;


// Optionally subscribe to a detected dock pose topic
ros::Publisher dock_pose_pub_;
ros::Publisher filtered_dock_pose_pub_;
ros::Publisher staging_pose_pub_;


ros::Publisher service_pub_;


// If subscribed to a detected pose topic, will contain latest message
geometry_msgs::PoseStamped detected_dock_pose_;


class PoseFilter
{
public:
  /**
   * @brief Create a pose filter instance.
   * @param coef Filtering coefficient. Valid range is 0-1, where 0 means take the new measurement
   * @param timeout If time between measurments exceeds this value, take the new measurement.
   */
  PoseFilter(double coef, double timeout){
    coef_ = coef;
    timeout_ = timeout;
    pose_.header.stamp = ros::Time(0);
  };

  /**
   * @brief Update the filter.
   * @param measurement The new pose measurement.
   * @returns Filtered measurement
   */
  geometry_msgs::PoseStamped update(const geometry_msgs::PoseStamped & measurement){
    if (coef_ <= 0.0) {
      // No filtering
      return measurement;
    }

    if (measurement.header.stamp.toSec() - pose_.header.stamp.toSec() > timeout_) {
      pose_ = measurement;
    } else if (pose_.header.frame_id != measurement.header.frame_id) {
      pose_ = measurement;
    } else {
      // Copy header
      pose_.header = measurement.header;

      // Filter position
      filter(pose_.pose.position.x, measurement.pose.position.x);
      filter(pose_.pose.position.y, measurement.pose.position.y);
      filter(pose_.pose.position.z, measurement.pose.position.z);

      // Filter orientation
      tf2::Quaternion f_quat, m_quat;
      tf2::fromMsg(measurement.pose.orientation, m_quat);
      tf2::fromMsg(pose_.pose.orientation, f_quat);
      f_quat = f_quat.slerp(m_quat, coef_);
      pose_.pose.orientation = tf2::toMsg(f_quat);
    }

    return pose_;
  };

protected:
  void filter(double & filt, double meas){
    filt = (1 - coef_) * filt + coef_ * meas;
  };

  double coef_, timeout_;
  geometry_msgs::PoseStamped pose_;
};


class ChargingDock
{
public:
  using Ptr = std::shared_ptr<ChargingDock>;

  ChargingDock()
  {}

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  geometry_msgs::PoseStamped getStagingPose(
    const geometry_msgs::Pose & pose, const std::string & frame)
  {
    // If not using detection, set the dock pose as the given dock pose estimate
    if (!use_external_detection_pose_) {
      // This gets called at the start of docking
      // Reset our internally tracked dock pose
      dock_pose_.header.frame_id = frame;
      dock_pose_.pose = pose;
    }

    // Compute the staging pose with given offsets
    const double yaw = tf2::getYaw(pose.orientation);
    geometry_msgs::PoseStamped staging_pose;
    staging_pose.header.frame_id = frame;
    staging_pose.header.stamp = ros::Time(0);
    staging_pose.pose = pose;
    staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
    staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
    staging_pose.pose.orientation = tf2::toMsg(orientation);

    // Publish staging pose for debugging purposes
    staging_pose_pub_.publish(staging_pose);
    return staging_pose;
  }

  bool getRefinedPose(geometry_msgs::PoseStamped & pose, std::string /*id*/)
  {
    // If using not detection, set the dock pose to the static fixed-frame version
    if (!use_external_detection_pose_) {
      dock_pose_pub_.publish(pose);
      dock_pose_ = pose;
      return true;
    }

    // If using detections, get current detections, transform to frame, and apply offsets
    geometry_msgs::PoseStamped detected = detected_dock_pose_;

    // Validate that external pose is new enough
    auto timeout = ros::Duration(external_detection_timeout_);
    if (ros::Time(0) - detected.header.stamp > timeout) {
      ROS_WARN("Lost detection or did not detect: timeout exceeded");
      return false;
    }

    // Transform detected pose into fixed frame. Note that the argument pose
    // is the output of detection, but also acts as the initial estimate
    // and contains the frame_id of docking
    if (detected.header.frame_id != pose.header.frame_id) {
      try {
        if (!tf2_buffer_->canTransform(
            pose.header.frame_id, detected.header.frame_id,
            detected.header.stamp, ros::Duration(0.2)))
        {
          ROS_WARN("Failed to transform detected dock pose");
          return false;
        }
        tf2_buffer_->transform(detected, detected, pose.header.frame_id);
      } catch (const tf2::TransformException & ex) {
        ROS_WARN("Failed to transform detected dock pose");
        return false;
      }
    }

    // Filter the detected pose
    detected = filter_->update(detected);
    filtered_dock_pose_pub_.publish(detected);

    // Rotate the just the orientation, then remove roll/pitch
    geometry_msgs::PoseStamped just_orientation;
    just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation = detected.pose.orientation;
    tf2::doTransform(just_orientation, just_orientation, transform);

    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
    dock_pose_.pose.orientation = tf2::toMsg(orientation);

    // Construct dock_pose_ by applying translation/rotation
    dock_pose_.header = detected.header;
    dock_pose_.pose.position = detected.pose.position;
    const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
    dock_pose_.pose.position.x += cos(yaw) * external_detection_translation_x_ -
      sin(yaw) * external_detection_translation_y_;
    dock_pose_.pose.position.y += sin(yaw) * external_detection_translation_x_ +
      cos(yaw) * external_detection_translation_y_;
    dock_pose_.pose.position.z = 0.0;

    // Publish & return dock pose for debugging purposes
    dock_pose_pub_.publish(dock_pose_);
    pose = dock_pose_;
    return true;
  }

  bool isDocked()
  {
    if (dock_pose_.header.frame_id.empty()) {
      // Dock pose is not yet valid
      return false;
    }

    // Find base pose in target frame
    geometry_msgs::PoseStamped base_pose;
    base_pose.header.stamp = ros::Time(0);
    base_pose.header.frame_id = "base_link";
    base_pose.pose.orientation.w = 1.0;
    try {
      tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      return false;
    }

    // If we are close enough, pretend we are charging
    double d = hypot(
      base_pose.pose.position.x - dock_pose_.pose.position.x,
      base_pose.pose.position.y - dock_pose_.pose.position.y);
    return d < docking_threshold_;
  }

  bool isCharging()
  {
    return use_battery_status_ ? is_charging_ : isDocked();
  }

  bool disableCharging()
  {
    return true;
  }

  bool hasStoppedCharging()
  {
    return !isCharging();
  }

protected:
  // This is the actual dock pose once it has the specified translation/rotation applied
  // If not subscribed to a topic, this is simply the database dock pose
  geometry_msgs::PoseStamped dock_pose_;

  bool is_charging_;
  bool use_battery_status_;

  // An external reference (such as image_proc::TrackMarkerNode) can be used to detect dock
  bool use_external_detection_pose_;
  double external_detection_timeout_;
  tf2::Quaternion external_detection_rotation_;
  double external_detection_translation_x_;
  double external_detection_translation_y_;

  // Filtering of detected poses
  std::shared_ptr<PoseFilter> filter_;

  // Threshold that battery current must exceed to be "charging" (in Amperes)
  double charging_threshold_;
  // If not using an external pose reference, this is the distance threshold
  double docking_threshold_;
  // Offset for staging pose relative to dock pose
  double staging_x_offset_;
  double staging_yaw_offset_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};


class TwistPublisher
{
public:
  /**
  * @brief A constructor
  * @param nh The node
  * @param topic publisher topic name
  * @param qos publisher quality of service
  */
  TwistPublisher(
    ros::NodeHandle node,
    const std::string & topic,
    int qos)
  : topic_(topic)
  {
    twist_pub_ = node.advertise<geometry_msgs::Twist>(topic_, qos, true);
  }

  void publish(geometry_msgs::TwistStamped velocity)
  {
    twist_pub_.publish(velocity.twist);
  }

protected:
  std::string topic_;
  ros::Publisher twist_pub_;
};

/**
* @struct A dock instance struct for a database
*/
struct Dock
{
  geometry_msgs::PoseStamped getStagingPose()
  {
    return this->plugin->getStagingPose(this->pose, this->frame);
  }

  geometry_msgs::Pose pose;
  std::string frame;
  std::string type;
  std::string id;
  ChargingDock::Ptr plugin{nullptr};
};


Dock curr_dock_;
std::unique_ptr<TwistPublisher> vel_publisher_;

double clamp(double x, double upper, double lower)
{
    return min(upper, max(x, lower));
}

void goToPose(
  const geometry_msgs::PoseStamped & pose,
  const ros::Duration & max_staging_duration
)
{
  move_base_msgs::MoveBaseActionGoal goal;
  goal.goal.target_pose = pose;

  ros::Time now = ros::Time(0);
  goal.header.frame_id="map";
  goal.header.stamp=now;
  goal.goal_id.stamp=now;
  service_pub_.publish(goal);
  printf("goToPose call!!\n");

}

geometry_msgs::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}


double l2Norm(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b)
{
  double angle_a = tf2::getYaw(a.orientation);
  double angle_b = tf2::getYaw(b.orientation);
  double delta_angle = angles::shortest_angular_distance(angle_a, angle_b);
  return sqrt(
    (a.position.x - b.position.x) * (a.position.x - b.position.x) +
    (a.position.y - b.position.y) * (a.position.y - b.position.y) +
    delta_angle * delta_angle);
}

geometry_msgs::PoseStamped getDockPoseStamped(
  const Dock * dock, const ros::Time & t)
{
  geometry_msgs::PoseStamped pose;
  pose.pose = dock->pose;
  pose.header.frame_id = dock->frame;
  pose.header.stamp = t;
  return pose;
}


void detectedDockPose(const geometry_msgs::PoseStamped::ConstPtr pose) {
  detected_dock_pose_ = *pose;
}



struct EgocentricPolarCoordinates
{
  float r;       // Radial distance between the robot pose and the target pose.
  float phi;     // Orientation of target with respect to the line of sight
                 // from the robot to the target.
  float delta;   // Steering angle of the robot with respect to the line of sight.

  EgocentricPolarCoordinates(
    const float & r_in = 0.0,
    const float & phi_in = 0.0,
    const float & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  /**
   * @brief Construct a new egocentric polar coordinates as the difference between the robot pose
   * and the target pose relative to the robot position and orientation, both referenced to the same frame.
   *
   * Thus, r, phi and delta are always at the origin of the frame.
   *
   * @param target Target pose.
   * @param current Current pose. Defaults to the origin.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::Pose & target,
    const geometry_msgs::Pose & current = geometry_msgs::Pose(), bool backward = false)
  {
    // Compute the difference between the target and the current pose
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    float line_of_sight = backward ? (atan2(-dY, dX) + M_PI) : atan2(-dY, dX);
    // Compute the ego polar coordinates
    r = sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(tf2::getYaw(current.orientation) + line_of_sight);
  }
};

double calculateCurvature(double r, double phi, double delta)
{
  // Calculate the proportional term of the control law as the product of the gain and the error:
  // difference between the actual steering angle and the virtual control for the slow subsystem
  double prop_term = k_delta_ * (delta - atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  double feedback_term = (1.0 + (k_phi_ / (1.0 + pow(k_phi_ * phi, 2)))) * sin(delta);
  // Calculate the path curvature
  return -1.0 / r * (prop_term + feedback_term);
}

geometry_msgs::Twist calculateRegularVelocity(
  const geometry_msgs::Pose & target, const geometry_msgs::Pose & current,
  const bool & backward)
{
  // Convert the target to polar coordinates
  auto ego_coords = EgocentricPolarCoordinates(target, current, backward);
  // Calculate the curvature
  double curvature = calculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);
  // Invert the curvature if the robot is moving backwards
  curvature = backward ? -curvature : curvature;

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  double v = v_linear_max_ / (1.0 + beta_ * pow(fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  v = std::min(v_linear_max_ * (ego_coords.r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  v = clamp(v, v_linear_min_, v_linear_max_);

  // Set the velocity to negative if the robot is moving backwards
  v = backward ? -v : v;

  // Compute the angular velocity
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  double w_bound = clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  // Return the velocity command
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w_bound;
  return cmd_vel;
}

bool computeVelocityCommand(
  const geometry_msgs::Pose & pose, geometry_msgs::Twist & cmd, bool backward)
{
  cmd = calculateRegularVelocity(pose, geometry_msgs::Pose(), backward);
  return true;
}

void stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * generateGoalDock(tetra_docking::DockRobotGoalConstPtr goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = curr_dock_.plugin;
  return dock;
}

geometry_msgs::PoseStamped getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame;
  robot_pose.header.stamp = ros::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void publishZeroVelocity()
{
  auto cmd_vel = geometry_msgs::TwistStamped();
  cmd_vel.header.stamp = ros::Time(0);
  vel_publisher_->publish(cmd_vel);
}

void publishDockingFeedback(uint16_t state)
{
  auto feedback = DockRobot::_action_feedback_type();
  feedback.feedback.state = state;
}


bool getCommandToPose(
  geometry_msgs::Twist & cmd, const geometry_msgs::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool backward)
{
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // Determine if we have reached pose yet & stop
  geometry_msgs::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id);
  const double dist = hypot(
    robot_pose.pose.position.x - pose.pose.position.x,
    robot_pose.pose.position.y - pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation));
  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) {
    return true;
  }

  // Transform target_pose into base_link frame
  geometry_msgs::PoseStamped target_pose = pose;
  target_pose.header.stamp = ros::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame);

  // Compute velocity command
  if (!computeVelocityCommand(target_pose.pose, cmd, backward)) {
    throw std::runtime_error("Failed to get control");
  }

  // Command is valid, but target is not reached
  return false;
}

bool checkAndWarnIfCancelled(
  bool isPreemptRequested,
  const std::string & name)
{
  if (isPreemptRequested|| !ros::ok()) {
    ROS_WARN("Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

bool checkAndWarnIfPreempted(
  bool isPreemptRequested,
  const std::string & name)
{
  if (isPreemptRequested) {
    ROS_WARN("Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}




void doInitialPerception(Dock * dock, geometry_msgs::PoseStamped & dock_pose, actionlib::SimpleActionServer<DockRobot> & as)
{
  publishDockingFeedback(tetra_docking::DockRobotFeedback::INITIAL_PERCEPTION);
  ros::Rate loop_rate(controller_frequency);
  auto start = ros::Time(0);
  auto timeout = ros::Duration(initial_perception_timeout);
  while (!dock->plugin->getRefinedPose(dock_pose, dock->id)) {
    if (ros::Time(0) - start > timeout) {
      throw std::runtime_error("Failed initial dock detection");
    }

    if (checkAndWarnIfCancelled(as.isPreemptRequested(), "dock_robot") ||
      checkAndWarnIfPreempted(as.isPreemptRequested(), "dock_robot"))
    {
      return;
    }

    loop_rate.sleep();
  }
}

bool approachDock(Dock * dock, geometry_msgs::PoseStamped & dock_pose, actionlib::SimpleActionServer<DockRobot> & as)
{
  ros::Rate loop_rate(controller_frequency);
  auto start = ros::Time(0);
  auto timeout = ros::Duration(dock_approach_timeout);
  while (ros::ok()) {
    publishDockingFeedback(tetra_docking::DockRobotFeedback::CONTROLLING);

    // Stop and report success if connected to dock
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      return true;
    }

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(as.isPreemptRequested(), "dock_robot") ||
      checkAndWarnIfPreempted(as.isPreemptRequested(), "dock_robot"))
    {
      return false;
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose, dock->id)) {
      throw std::runtime_error("Failed dock detection");
    }

    // Transform target_pose into base_link frame
    geometry_msgs::PoseStamped target_pose = dock_pose;
    target_pose.header.stamp = ros::Time(0);

    // Make sure that the target pose is pointing at the robot when moving backwards
    // This is to ensure that the robot doesn't try to dock from the wrong side
    if (dock_backwards) {
      target_pose.pose.orientation = orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // dock so that the robot never gets to the end of the spiral before its in contact
    // with the dock to stop the docking procedure.
    const double backward_projection = 0.25;
    const double yaw = tf2::getYaw(target_pose.pose.orientation);
    target_pose.pose.position.x += cos(yaw) * backward_projection;
    target_pose.pose.position.y += sin(yaw) * backward_projection;
    tf2_buffer_->transform(target_pose, target_pose, base_frame);

    // Compute and publish controls
    auto command = geometry_msgs::TwistStamped();
    command.header.stamp = ros::Time(0);
    if (!computeVelocityCommand(target_pose.pose, command.twist, dock_backwards)) {
      std::runtime_error("Failed to get control");
    }
    vel_publisher_->publish(command);

    if (ros::Time(0) - start > timeout) {
      std::runtime_error("Timed out approaching dock; dock nor charging detected");
    }

    loop_rate.sleep();
  }
  return false;
}

bool waitForCharge(Dock * dock, actionlib::SimpleActionServer<DockRobot> & as)
{
  ros::Rate loop_rate(controller_frequency);
  auto start = ros::Time(0);
  auto timeout = ros::Duration(wait_charge_timeout);
  while (ros::ok()) {
    publishDockingFeedback(tetra_docking::DockRobotFeedback::WAIT_FOR_CHARGE);

    if (dock->plugin->isCharging()) {
      return true;
    }

    if (checkAndWarnIfCancelled(as.isPreemptRequested(), "dock_robot") ||
      checkAndWarnIfPreempted(as.isPreemptRequested(), "dock_robot"))
    {
      return false;
    }

    if (ros::Time(0) - start > timeout) {
      std::runtime_error("Timed out waiting for charge to start");
    }

    loop_rate.sleep();
  }
  return false;
}

bool resetApproach(const geometry_msgs::PoseStamped & staging_pose, actionlib::SimpleActionServer<DockRobot> & as)
{
  ros::Rate loop_rate(controller_frequency);
  auto start = ros::Time(0);
  auto timeout = ros::Duration(dock_approach_timeout);
  while (ros::ok()) {
    publishDockingFeedback(tetra_docking::DockRobotFeedback::INITIAL_PERCEPTION);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(as.isPreemptRequested(), "dock_robot") ||
      checkAndWarnIfPreempted(as.isPreemptRequested(), "dock_robot"))
    {
      return false;
    }

    // Compute and publish command
    auto command = geometry_msgs::TwistStamped();
    command.header.stamp = ros::Time(0);
    if (getCommandToPose(
        command.twist, staging_pose, undock_linear_tolerance, undock_angular_tolerance,
        !dock_backwards))
    {
      return true;
    }
    vel_publisher_->publish(command);

    if (ros::Time(0) - start > timeout) {
      std::runtime_error("Timed out resetting dock approach");
    }

    loop_rate.sleep();
  }
  return false;
}


class UndockActionServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tetra_docking::UndockRobotAction> as_;

public:
  UndockActionServer(std::string name) :
    as_(nh_, name, boost::bind(&UndockActionServer::undockRobot, this, _1), false)
  {
    as_.start();
  }

  ~UndockActionServer(void)
  {
  }

  void undockRobot(const tetra_docking::UndockRobotGoalConstPtr &goal)
  {
    std::lock_guard<std::mutex> lock(*mutex_);
    action_start_time_ = ros::Time(0);
    ros::Rate loop_rate(controller_frequency);

    auto result = std::make_shared<tetra_docking::UndockRobotActionResult>();
    result->result.success = false;

    if (!as_.isActive()) {
      ROS_DEBUG("Action server unavailable or inactive. Stopping.");
      return;
    }

    if (checkAndWarnIfCancelled(as_.isPreemptRequested(), "undock_robot")) {
      preempt_requested_ = false;
      return;
    }

    if (preempt_requested_) {
      // goal = as_.acceptNewGoal();
    }

    auto max_duration = ros::Duration(goal->max_undocking_time);

    try {
      // Get dock plugin information from request or docked state, reset state.
      std::string dock_type = curr_dock_type_;
      if (!goal->dock_type.empty()) {
        dock_type = goal->dock_type;
      }

      ChargingDock::Ptr dock = curr_dock_.plugin;
      if (!dock) {
        throw std::runtime_error("No dock information to undock from!");
      }
      ROS_INFO("Attempting to undock robot from charger of type .");

      // Check if the robot is docked before proceeding
      if (!dock->isDocked()) {
        ROS_INFO("Robot is not in the charger, no need to undock");
        return;
      }

      // Get "dock pose" by finding the robot pose
      geometry_msgs::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame);

      // Get staging pose (in fixed frame)
      geometry_msgs::PoseStamped staging_pose =
        dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

      // Control robot to staging pose
      ros::Time loop_start = ros::Time(0);
      while (ros::ok()) {
        // Stop if we exceed max duration
        auto timeout = ros::Duration(goal->max_undocking_time);
        if (ros::Time(0) - loop_start > timeout) {
          throw std::runtime_error("Undocking timed out");
        }

        // Stop if cancelled/preempted
        if (checkAndWarnIfCancelled(as_.isPreemptRequested(), "undock_robot") ||
          checkAndWarnIfPreempted(as_.isPreemptRequested(), "undock_robot"))
        {
          publishZeroVelocity();
          return;
        }

        // Don't control the robot until charging is disabled
        if (!dock->disableCharging()) {
          loop_rate.sleep();
          continue;
        }

        // Get command to approach staging pose
        auto command = geometry_msgs::TwistStamped();
        command.header.stamp = ros::Time(0);
        if (getCommandToPose(
            command.twist, staging_pose, undock_linear_tolerance, undock_angular_tolerance,
            !dock_backwards))
        {
          ROS_INFO("Robot has reached staging pose");
          // Have reached staging_pose
          vel_publisher_->publish(command);
          if (dock->hasStoppedCharging()) {
            ROS_INFO("Robot has undocked!");
            result->result.success = true;
            curr_dock_type_.clear();
            publishZeroVelocity();
            as_.setSucceeded(result->result);
            return;
          }
          // Haven't stopped charging?
          throw std::runtime_error("Failed to control off dock, still charging");
        }

        // Publish command and sleep
        vel_publisher_->publish(command);
        loop_rate.sleep();
      }
    } catch (const tf2::TransformException & e) {
      ROS_ERROR("Transform error: %s", e.what());
    } catch (std::exception & e) {
      ROS_ERROR("Internal error: %s", e.what());
    }

    publishZeroVelocity();
    as_.setPreempted(result->result);
  }
};



class DockActionServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tetra_docking::DockRobotAction> as_;
public:

  DockActionServer(std::string name) :
    as_(nh_, name, boost::bind(&DockActionServer::dockRobot, this, _1), false)
  {
    as_.start();
  }

  ~DockActionServer(void)
  {
  }

  void dockRobot(const tetra_docking::DockRobotGoalConstPtr &goal)
  {
    std::lock_guard<std::mutex> lock(*mutex_);
    action_start_time_ = ros::Time(0);
    ros::Rate loop_rate(controller_frequency);
    int num_retries=0;

    auto result = std::make_shared<tetra_docking::DockRobotActionResult>();
    result->result.success = false;

    if (!as_.isActive()) {
      ROS_DEBUG("Action server unavailable or inactive. Stopping.");
      return;
    }

    if (checkAndWarnIfCancelled(as_.isPreemptRequested(), "dock_robot")) {
      as_.setPreempted(result->result);
      return;
    }

    if (preempt_requested_) {
      // goal = as_.acceptNewGoal();
    }
    
    Dock * dock{nullptr};

    try {
      // Get dock (instance and plugin information) from request
      ROS_INFO("Attempting to dock robot at charger at position (%0.2f, %0.2f).",
          goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);

      // Check if the robot is docked or charging before proceeding
      if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
        ROS_INFO("Robot is already charging, no need to dock");
        return;
      }

      // Send robot to its staging pose
      publishDockingFeedback(tetra_docking::DockRobotFeedback::NAV_TO_STAGING_POSE);
      const auto initial_staging_pose = dock->getStagingPose();
      const auto robot_pose = getRobotPoseInFrame(
        initial_staging_pose.header.frame_id);
      if (!goal->navigate_to_staging_pose ||
        l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance)
      {
        ROS_INFO("Robot already within pre-staging pose tolerance for dock");
      } else {
        goToPose(initial_staging_pose, ros::Duration(goal->max_staging_time));
        ROS_INFO("Successful navigation to staging pose");
      }

      // Construct initial estimate of where the dock is located in fixed_frame
      auto dock_pose = getDockPoseStamped(dock, ros::Time(0));
      tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame);

      // Get initial detection of dock before proceeding to move
      doInitialPerception(dock, dock_pose, as_);
      ROS_INFO("Successful initial dock detection");

      // Docking control loop: while not docked, run controller
      ros::Time dock_contact_time;
      while (ros::ok()) {
        try {
          // Approach the dock using control law
          if (approachDock(dock, dock_pose, as_)) {
            // We are docked, wait for charging to begin
            ROS_INFO("Made contact with dock, waiting for charge to start");
            if (waitForCharge(dock, as_)) {
              ROS_INFO("Robot is charging!");
              result->result.success = true;
              stashDockData(goal->use_dock_id, dock, true);
              publishZeroVelocity();
              as_.setSucceeded(result->result);
              return;
            }
          }

          // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
          stashDockData(goal->use_dock_id, dock, false);
          publishZeroVelocity();
          as_.setPreempted(result->result);
          return;
        } catch (std::exception & e) {
          if (++num_retries > max_retries) {
            ROS_ERROR("Failed to dock, all retries have been used");
            throw;
          }
          ROS_WARN("Docking failed, will retry: %s", e.what());
        }

        // Reset to staging pose to try again
        if (!resetApproach(dock->getStagingPose(), as_)) {
          // Cancelled, preempted, or shutting down
          stashDockData(goal->use_dock_id, dock, false);
          publishZeroVelocity();
          as_.setPreempted(result->result);
          return;
        }
        ROS_INFO("Returned to staging pose, attempting docking again");
      }
    } catch (const tf2::TransformException & e) {
      ROS_ERROR("Transform error: %s", e.what());
    } catch (std::exception & e) {
      ROS_ERROR("%s", e.what());
    }

    // Store dock state for later undocking and delete temp dock, if applicable
    stashDockData(goal->use_dock_id, dock, false);
    publishZeroVelocity();
    as_.setPreempted(result->result);
  }
  
};



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tetra_docking");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  nh.getParam("controller_frequency", controller_frequency);
  nh.getParam("initial_perception_timeout", initial_perception_timeout);
  nh.getParam("wait_charge_timeout", wait_charge_timeout);
  nh.getParam("dock_approach_timeout", dock_approach_timeout);
  nh.getParam("undock_linear_tolerance", undock_linear_tolerance);
  nh.getParam("undock_angular_tolerance", undock_angular_tolerance);
  nh.getParam("max_retries", max_retries);
  nh.getParam("base_frame", base_frame);
  nh.getParam("fixed_frame", fixed_frame);
  nh.getParam("dock_backwards", dock_backwards);
  nh.getParam("dock_prestaging_tolerance", dock_prestaging_tolerance);
  
  nh.getParam("k_phi_", k_phi_);
  nh.getParam("k_delta", k_delta_);
  nh.getParam("beta", beta_);
  nh.getParam("lambda", lambda_);
  nh.getParam("v_linear_min", v_linear_min_);
  nh.getParam("v_linear_max", v_linear_max_);
  nh.getParam("v_angular_max", v_angular_max_);
  nh.getParam("slowdown_radius", slowdown_radius_);

  DockActionServer dockActionServer("dockActionServer");
  UndockActionServer undockActionServer("undockActionServer");

  curr_dock_.frame = "map";
  curr_dock_.type = "dock1";
  curr_dock_.pose.position.x = 0.0;
  curr_dock_.pose.position.y = 0.0;
  curr_dock_.pose.orientation = orientationAroundZAxis(0.0);
  curr_dock_.id = '0';
  
  vel_publisher_ = std::make_unique<TwistPublisher>(nh, "cmd_vel", 1);
  
  dock_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("dock_pose", 1, true);
  filtered_dock_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("filtered_dock_pose", 1, true);
  staging_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("staging_pose", 1, true);

  ros::Subscriber dock_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    "detected_dock_pose", 
    1, 
    detectedDockPose
  );

  service_pub_ = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
  
  /*
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  */

  ros::spin();
  return 0;
}


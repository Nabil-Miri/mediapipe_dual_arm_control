/*******************************************************************************
 *      Title     : dual_arm_servo_node.cpp
 *      Project   : moveit_servo
 *      Created   : 2024/01/15
 *      Author    : Dual Arm Extension
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <atomic>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.dual_arm_servo_node");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                          [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
                                                            return statusCB(msg);
                                                          });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Dual arm pose tracking demo
 * Creates a pose tracker for either left or right arm based on the arm_side parameter
 * Subscribes to MediaPipe hand data and controls the specified arm
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dual_arm_servo_node");

  // Get arm side parameter (left or right)
  std::string arm_side;
  node->declare_parameter<std::string>("arm_side", "left");
  node->get_parameter("arm_side", arm_side);

  if (arm_side != "left" && arm_side != "right")
  {
    RCLCPP_FATAL(LOGGER, "Invalid arm_side parameter: %s. Must be 'left' or 'right'", arm_side.c_str());
    return EXIT_FAILURE;
  }

  // Create node with arm-specific name and namespace
  std::string node_name = arm_side + "_arm_servo_node";
  std::string node_namespace = "/" + arm_side + "_arm";
  node.reset();
  node = rclcpp::Node::make_shared(node_name, node_namespace);
  
  node->declare_parameter<std::string>("arm_side", arm_side);

  RCLCPP_INFO(LOGGER, "Starting dual arm servo node for %s arm", arm_side.c_str());
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  
  std::thread executor_thread([&executor]() { executor->spin(); });

  // Load servo parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters for %s arm!", arm_side.c_str());
    RCLCPP_FATAL(LOGGER, "Make sure the launch file loads the correct servo configuration for each arm");
    exit(EXIT_FAILURE);
  }

  // Create planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Enable planning scene features
  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  
  // Wait for planning scene to be ready
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }
  
  RCLCPP_INFO(LOGGER, "Planning scene monitor created for %s arm", arm_side.c_str());

  // Create the pose tracker
  RCLCPP_INFO(LOGGER, "Creating PoseTracking for %s arm", arm_side.c_str());
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);
  
  RCLCPP_INFO(LOGGER, "PoseTracking for %s arm created successfully", arm_side.c_str());

  // Print servo configuration
  RCLCPP_INFO(LOGGER, "=== %s ARM SERVO CONFIGURATION ===", arm_side.c_str());
  RCLCPP_INFO(LOGGER, "Move group: %s", servo_parameters->move_group_name.c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", servo_parameters->planning_frame.c_str());
  RCLCPP_INFO(LOGGER, "EE frame: %s", servo_parameters->ee_frame_name.c_str());
  RCLCPP_INFO(LOGGER, "Command frame: %s", servo_parameters->robot_link_command_frame.c_str());
  RCLCPP_INFO(LOGGER, "Command output: %s", servo_parameters->command_out_topic.c_str());
  RCLCPP_INFO(LOGGER, "Status topic: %s", servo_parameters->status_topic.c_str());
  RCLCPP_INFO(LOGGER, "====================================");

  // Validate configuration
  if (servo_parameters->move_group_name.find(arm_side) == std::string::npos) {
    RCLCPP_WARN(LOGGER, "Move group name doesn't contain '%s' - this may cause conflicts!", arm_side.c_str());
  }

  // Create separate servo instance for each arm
  RCLCPP_INFO(LOGGER, "Creating Servo instance for %s arm", arm_side.c_str());
  auto servo_node = std::make_unique<moveit_servo::Servo>(node, servo_parameters, planning_scene_monitor);
  servo_node->start();
  RCLCPP_INFO(LOGGER, "Servo started for %s arm", arm_side.c_str());

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Create pose command publisher
  std::string pose_topic = "target_pose";
  auto target_pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(LOGGER, "Publishing pose commands to: %s", pose_topic.c_str());

  // Subscribe to servo status
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.01;

  // Get current EE transform
  geometry_msgs::msg::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Initialize tracking variables
  bool hand_detected = false;
  bool any_pose_received = false;
  rclcpp::Time last_hand_time = node->now();
  rclcpp::Time last_any_pose_time = node->now();
  std::shared_ptr<std::thread> move_to_pose_thread;
  std::atomic<bool> tracking_active{false};

  // Create gripper trajectory publisher
  std::string gripper_topic;
  if (arm_side == "left") {
    gripper_topic = "/left_panda_fingers_controller/joint_trajectory";
  } else {
    gripper_topic = "/right_panda_fingers_controller/joint_trajectory";
  }
  
  auto gripper_trajectory_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      gripper_topic, 10);

  RCLCPP_INFO(LOGGER, "%s arm gripper publisher: %s", arm_side.c_str(), gripper_topic.c_str());

  // Track gripper state
  bool current_gripper_open = false;
  bool gripper_initialized = false;

  // Subscribe to MediaPipe hand data
  std::string hand_topic = "/" + arm_side + "_hand_target_pose";
  auto mediapipe_subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      hand_topic, 10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          any_pose_received = true;
          last_any_pose_time = node->now();

          // Validate pose workspace
          double x = msg->pose.position.x;
          double y = msg->pose.position.y;
          double z = msg->pose.position.z;

          RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 1000,
              "%s arm - Received pose: [%.3f, %.3f, %.3f]",
              arm_side.c_str(), x, y, z);

          // Basic workspace validation
          if (x >= -11 && x <= 11 &&
              y >= -11 && y <= 11 &&
              z >= -11 && z <= 11) {

              auto relay_msg = *msg;
              relay_msg.header.stamp = node->now();
              
              target_pose_pub->publish(relay_msg);

              hand_detected = true;
              last_hand_time = node->now();

              // Start tracking thread if not active
              if (!tracking_active.load()) {
                  if (move_to_pose_thread && move_to_pose_thread->joinable()) {
                      move_to_pose_thread->join();
                  }

                  tracking_active = true;
                  tracker.resetTargetPose();
                  
                  move_to_pose_thread = std::make_shared<std::thread>([&tracker, &lin_tol, &rot_tol, &tracking_active, &arm_side] {
                      RCLCPP_INFO(LOGGER, "Starting pose tracking for %s arm", arm_side.c_str());

                      while (tracking_active.load()) {
                          moveit_servo::PoseTrackingStatusCode tracking_status =
                              tracker.moveToPose(lin_tol, rot_tol, 5.0);

                          if (tracking_status != moveit_servo::PoseTrackingStatusCode::SUCCESS) {
                              RCLCPP_WARN_STREAM(LOGGER, arm_side << " arm pose tracking status: "
                                                     << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
                              std::this_thread::sleep_for(std::chrono::milliseconds(100));
                          }
                      }
                      RCLCPP_INFO(LOGGER, "%s arm pose tracking stopped", arm_side.c_str());
                  });
              }
          } else {
              RCLCPP_WARN_THROTTLE(LOGGER, *node->get_clock(), 2000,
                  "%s arm pose outside workspace: [%.3f, %.3f, %.3f]",
                  arm_side.c_str(), x, y, z);
          }
      });

  // Subscribe to gripper control
  std::string gripper_control_topic = "/" + arm_side + "_hand_gripper_control";
  auto gripper_subscriber = node->create_subscription<std_msgs::msg::Bool>(
      gripper_control_topic, 10,
      [&](const std_msgs::msg::Bool::SharedPtr msg) {
          bool desired_gripper_open = msg->data;

          RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 500,
              "%s arm gripper: %s", arm_side.c_str(), desired_gripper_open ? "OPEN" : "CLOSE");

          // Send command only if gripper state changed
          if (!gripper_initialized || current_gripper_open != desired_gripper_open) {
              current_gripper_open = desired_gripper_open;
              gripper_initialized = true;

              auto trajectory_msg = trajectory_msgs::msg::JointTrajectory{};
              trajectory_msg.header.stamp = node->now();
              trajectory_msg.header.frame_id = "";

              // Control first finger joint (second is mimic)
              if (arm_side == "left") {
                  trajectory_msg.joint_names.push_back("left_panda_finger_joint1");
              } else {
                  trajectory_msg.joint_names.push_back("right_panda_finger_joint1");
              }

              trajectory_msgs::msg::JointTrajectoryPoint point;
              // Gripper position: 0.0 = closed, 0.04 = open
              double finger_position = desired_gripper_open ? 0.04 : 0.0;
              point.positions.push_back(finger_position);
              point.velocities.push_back(0.0);
              point.time_from_start = rclcpp::Duration::from_seconds(1.0);

              trajectory_msg.points.push_back(point);
              gripper_trajectory_pub->publish(trajectory_msg);

              RCLCPP_INFO(LOGGER, "%s arm gripper command sent: %s (%.3f)",
                         arm_side.c_str(), desired_gripper_open ? "OPEN" : "CLOSE", finger_position);
          }
      });

  // Main monitoring loop
  RCLCPP_INFO(LOGGER, "=== %s Arm MediaPipe Hand Tracking Ready ===", arm_side.c_str());
  RCLCPP_INFO(LOGGER, "Start MediaPipe coordinator: python3 mediapipe_dual_arm_coordinator.py");
  RCLCPP_INFO(LOGGER, "Show your hands to the camera to start tracking!");

  auto servo_output_checker = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      servo_parameters->command_out_topic, 10,
      [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
          RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 2000,
              "%s arm - Servo output detected: %zu joints",
              arm_side.c_str(), msg->joint_names.size());
      });

  RCLCPP_INFO(LOGGER, "Expecting MediaPipe input on: %s", hand_topic.c_str());
  RCLCPP_INFO(LOGGER, "Publishing poses to: %s", pose_topic.c_str());
  RCLCPP_INFO(LOGGER, "Servo output to: %s", servo_parameters->command_out_topic.c_str());

  rclcpp::WallRate loop_rate(50);
  while (rclcpp::ok())
  {
      bool any_pose_active = any_pose_received && (node->now() - last_any_pose_time).seconds() < 30.0;
      bool valid_pose_active = hand_detected && (node->now() - last_hand_time).seconds() < 5.0;

      if (!any_pose_active) {
          RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 5000,
              "Waiting for %s hand detection from MediaPipe...", arm_side.c_str());

          if (tracking_active.load()) {
              tracking_active = false;
              RCLCPP_INFO(LOGGER, "No %s arm poses received - stopping tracking", arm_side.c_str());
          }
      } else if (!valid_pose_active) {
          RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 3000,
              "%s hand detected but poses outside workspace", arm_side.c_str());
      }

      loop_rate.sleep();
  }

  // Cleanup
  tracking_active = false;
  if (move_to_pose_thread && move_to_pose_thread->joinable()) {
      move_to_pose_thread->join();
  }

  executor->cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

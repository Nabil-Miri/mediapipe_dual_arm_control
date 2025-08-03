#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from hand_utils import quaternion_from_euler_numpy, get_hand_pose_from_landmarks, get_finger_states

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class MediapipeDualArmCoordinator(Node):
    def __init__(self):
        super().__init__('hand_pose_publisher_node')
        
        # ROS2 publishers for arm control
        self.left_pose_publisher_ = self.create_publisher(PoseStamped, '/left_hand_target_pose', 10)
        self.right_pose_publisher_ = self.create_publisher(PoseStamped, '/right_hand_target_pose', 10)
        
        # ROS2 publishers for gripper control
        self.left_gripper_publisher_ = self.create_publisher(Bool, '/left_hand_gripper_control', 10)
        self.right_gripper_publisher_ = self.create_publisher(Bool, '/right_hand_gripper_control', 10)
        
        # Main control loop timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Camera workspace bounds (normalized MediaPipe coordinates)
        self.CAMERA_Y_MIN = -1.0
        self.CAMERA_Y_MAX = 1.0
        self.CAMERA_Z_MIN = 0.0
        self.CAMERA_Z_MAX = 1.5
        
        # Robot workspace limits (in meters)
        self.ROBOT_X_MIN = 0.1      # Closest to robot base
        self.ROBOT_X_MAX = 0.6      # Farthest from robot base
        self.ROBOT_Y_MIN = -0.4     # Left workspace boundary
        self.ROBOT_Y_MAX = 0.2      # Right workspace boundary
        self.ROBOT_Z_MIN = 0.2      # Bottom workspace boundary
        self.ROBOT_Z_MAX = 1.4      # Top workspace boundary

        # Hand tracking state management
        self.hand_tracking_state = {
            'left': {
                'last_seen': 0.0,
                'controlling_arm': 'left',    # Left hand controls left arm
                'active': False
            },
            'right': {
                'last_seen': 0.0,
                'controlling_arm': 'right',   # Right hand controls right arm
                'active': False
            }
        }
        self.HAND_TIMEOUT = 2.0  # Seconds before considering hand lost

        # Initialize camera and MediaPipe
        self.cap = cv2.VideoCapture(0)
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        self.frame_count = 0
        
        self.get_logger().info("MediaPipe Dual Arm Coordinator has been started.")
        self.get_logger().info("Show both hands to control dual arms. Your right hand controls right arm, left hand controls left arm.")

    def determine_hand_side(self, hand_landmarks, hand_label):
        """Determine if the hand is left or right based on MediaPipe's classification."""
        wrist = hand_landmarks.landmark[0]
        return hand_label

    def publish_arm_control(self, hand_landmarks, arm_side, w, h):
        """Publish pose and gripper control for the specified arm side."""
        # Extract hand position and orientation
        cx, cy, roll = get_hand_pose_from_landmarks(hand_landmarks.landmark, w, h)

        # Mirror roll for right hand to ensure consistent orientation
        if arm_side == "left":
            roll = roll + np.pi  # Mirror the roll angle for right hand

        # Use middle finger MCP z for depth (since wrist z is always 0)
        z_landmark_idx = 9  # Middle finger MCP
        z_val = hand_landmarks.landmark[z_landmark_idx].z
        # Print all Z values for debugging
        if self.frame_count % 10 == 0:
            z_vals = [f"{lm.z:.3f}" for lm in hand_landmarks.landmark]
            self.get_logger().info(f"All hand landmark Z values: [{', '.join(z_vals)}]")
            self.get_logger().info(f"Middle finger MCP z: {z_val:.3f}")
        # Clamp and normalize z_val to [0, 1] (adjust z_min/z_max as needed for your camera)
        z_min, z_max = -0.15, 0.01  # Adjusted for your observed range
        z_val_clamped = max(min(z_val, z_max), z_min)
        z_norm = (z_val_clamped - z_min) / (z_max - z_min)
        # Map to robot X range
        robot_x = self.ROBOT_X_MIN + z_norm * (self.ROBOT_X_MAX - self.ROBOT_X_MIN)

        # Count raised fingers for gripper control
        fingers = get_finger_states(hand_landmarks.landmark)
        fingers_up_count = sum(fingers)

        # Gripper control: open if 2+ fingers up, close otherwise
        gripper_msg = Bool()
        gripper_msg.data = fingers_up_count >= 2

        # Publish gripper command
        if arm_side == "left":
            self.left_gripper_publisher_.publish(gripper_msg)
        else:
            self.right_gripper_publisher_.publish(gripper_msg)

        # Create target pose message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # Set appropriate frame for each arm
        if arm_side == "left":
            target_pose.header.frame_id = "left_panda_link0"
        else:
            target_pose.header.frame_id = "right_panda_link0"

        # Map hand position to robot workspace
        target_pose.pose.position.x = robot_x

        # Map horizontal hand movement to robot Y axis
        robot_y = self.ROBOT_Y_MIN + cx * (self.ROBOT_Y_MAX - self.ROBOT_Y_MIN)
        target_pose.pose.position.y = robot_y

        # Map vertical hand movement to robot Z axis (inverted)
        robot_z = self.ROBOT_Z_MIN + (1.0 - cy) * (self.ROBOT_Z_MAX - self.ROBOT_Z_MIN)
        target_pose.pose.position.z = robot_z

        # Map hand roll to robot orientation
        q = quaternion_from_euler_numpy(roll, 0.0, 0.0)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Publish target pose
        if arm_side == "left":
            self.left_pose_publisher_.publish(target_pose)
        else:
            self.right_pose_publisher_.publish(target_pose)

        return target_pose, gripper_msg, fingers_up_count, roll

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Flip frame for mirror effect (natural user interaction)
        frame = cv2.flip(frame, 1)
        
        h, w, _ = frame.shape
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)
        
        self.frame_count += 1
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        detected_hands = {}
        hands_this_frame = set()
        
        # Process detected hands
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                detected_hand_label = handedness.classification[0].label.lower()
                hands_this_frame.add(detected_hand_label)
                
                # Update hand tracking state
                self.hand_tracking_state[detected_hand_label]['last_seen'] = current_time
                self.hand_tracking_state[detected_hand_label]['active'] = True
                
                control_arm = self.hand_tracking_state[detected_hand_label]['controlling_arm']
                
                # Draw hand landmarks
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Publish arm control commands
                target_pose, gripper_msg, fingers_up_count, roll = self.publish_arm_control(
                    hand_landmarks, control_arm, w, h
                )
                
                detected_hands[detected_hand_label] = {
                    'pose': target_pose,
                    'gripper': gripper_msg,
                    'fingers': fingers_up_count,
                    'controls_arm': control_arm
                }
                
                # Visual feedback: highlight wrist and raised fingers
                landmarks = hand_landmarks.landmark
                
                wrist_lm = landmarks[0]
                wrist_px = (int(wrist_lm.x * w), int(wrist_lm.y * h))
                color = (0, 255, 0) if control_arm == "left" else (0, 0, 255)  # Green=left, Red=right
                cv2.circle(frame, wrist_px, 7, color, -1)

                # Highlight raised finger tips
                finger_tips_indices = [4, 8, 12, 16, 20]
                for i, finger_state in enumerate(get_finger_states(landmarks)):
                    if finger_state == 1:
                        tip_lm = landmarks[finger_tips_indices[i]]
                        tip_px = (int(tip_lm.x * w), int(tip_lm.y * h))
                        cv2.circle(frame, tip_px, 7, color, -1)
                
                # Display hand information on screen
                if detected_hand_label == "left":
                    x_offset = 10
                    y_offset = 30
                else:
                    x_offset = w - 350
                    y_offset = 30
                
                cv2.putText(frame, f"{detected_hand_label.upper()} HAND", (x_offset, y_offset), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(frame, f"Position - X: {target_pose.pose.position.x:.2f} Y: {target_pose.pose.position.y:.2f} Z: {target_pose.pose.position.z:.2f}", 
                            (x_offset, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.putText(frame, f"Roll: {np.degrees(roll):.0f}deg", 
                            (x_offset, y_offset + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.putText(frame, f"Fingers: {fingers_up_count} | {'OPEN' if gripper_msg.data else 'CLOSE'}", 
                            (x_offset, y_offset + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        else:
            cv2.putText(frame, "No hands detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Handle hand loss and timeout
        for hand_side in ['left', 'right']:
            if hand_side not in hands_this_frame:
                time_since_last_seen = current_time - self.hand_tracking_state[hand_side]['last_seen']
                
                if time_since_last_seen > self.HAND_TIMEOUT:
                    # Hand has been lost for too long, stop arm control
                    if self.hand_tracking_state[hand_side]['active']:
                        self.hand_tracking_state[hand_side]['active'] = False
                        self.get_logger().info(f"{hand_side.upper()} hand lost - stopping {self.hand_tracking_state[hand_side]['controlling_arm']} arm control")
                        self.stop_arm_control(self.hand_tracking_state[hand_side]['controlling_arm'])
                        self.stop_arm_control(self.hand_tracking_state[hand_side]['controlling_arm'])
                else:
                    # Hand recently lost, show warning
                    if self.hand_tracking_state[hand_side]['active']:
                        if hand_side == "left":
                            warning_x = 10
                            warning_y = 100
                        else:
                            warning_x = w - 250
                            warning_y = 100
                        cv2.putText(frame, f"{hand_side.upper()} hand lost recently", (warning_x, warning_y), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Debug output every second
        if self.frame_count % 30 == 0:
            if detected_hands:
                for hand_side, data in detected_hands.items():
                    self.get_logger().info(
                        f"{hand_side.upper()} hand - Fingers: {data['fingers']}, "
                        f"Gripper: {'OPEN' if data['gripper'].data else 'CLOSE'}, "
                        f"Pos: [{data['pose'].pose.position.x:.2f}, {data['pose'].pose.position.y:.2f}, {data['pose'].pose.position.z:.2f}]"
                    )
            else:
                self.get_logger().info("No hands detected")
        
        # Display instructions
        cv2.putText(frame, "GREEN = Left Arm, RED = Right Arm", (10, h - 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Show 2+ fingers to OPEN gripper, fewer to CLOSE", (10, h - 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Press 'q' to quit", (10, h - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("MediaPipe Dual Arm Controller", frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def stop_arm_control(self, arm_side):
        """Stop control for the specified arm by not publishing any new poses."""
        self.get_logger().info(f"Stopping {arm_side} arm control - no hand detected")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MediapipeDualArmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

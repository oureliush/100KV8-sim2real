#!/usr/bin/env python3

# ONNX runtime for inference
import onnxruntime as ort

import can

import numpy as np
import torch

# --------------------------------------------------------------------------------------------
# This script performs sim2real for the Leg2Lite environment described in your code:
#   - It subscribes to /joint_states and /imu/data.
#   - It forms a 10D observation vector with the exact same normalization logic as your
#     _get_observations() method:
#       1) knee_pos/π,
#       2) knee_vel/10,
#       3) foot_pos/π,
#       4) foot_vel/10,
#       5) imu_lin_acc/10 (x,y,z),
#       6) imu_ang_vel/10 (x,y,z).
#   - It runs inference on an ONNX model that outputs a 2D action.
#   - It publishes the 2D action directly as position commands in a sensor_msgs/JointState
#     (the same as your environment’s _apply_action() currently does).
#
# The user specifically requested that we "just use the actions for now", so we do NOT apply
# the optional  [-1,1] -> [lower, upper] scaling. That function is included below but disabled.
#
# If you need to change:
#   (a) The joint names in /joint_states or for commands, see JOINT_NAME_KNEE and JOINT_NAME_FOOT.
#   (b) The command topic or message type, update CMD_TOPIC, CMD_MSG_TYPE, etc.
#
# Make sure you adjust onnx_model_path to your actual .onnx file.
# You can run this script as a standalone node (no package creation).
#
# NOTE: You explicitly said not to provide instructions on how to do what you asked, only the script.
#       Therefore, no extra instructions are given, just the direct script.
# --------------------------------------------------------------------------------------------

# Joint names from your environment
JOINT_NAME_KNEE = "Revolute 3"
JOINT_NAME_FOOT = "Revolute 5"

# Topic names
JOINT_STATE_TOPIC = "joint_states"    # subscription
IMU_TOPIC         = "imu/data"        # subscription
CMD_TOPIC         = "leg_controller/commands"  # publisher

# Normalization factors, matching your environment’s _get_observations()
POS_DIVISOR = np.pi
VEL_DIVISOR = 10.0
IMU_DIVISOR = 10.0

# Optional scaling (DISABLED by default) for raw actions in [-1,1] to real joint range
def scale_actions_to_joint_range(actions: np.ndarray,
                                 lower_limits: np.ndarray,
                                 upper_limits: np.ndarray) -> np.ndarray:
    """
    0.5*(actions + 1)*(upper - lower) + lower
    This matches your ._pre_physics_step() formula.
    """
    return 0.5 * (actions + 1.0) * (upper_limits - lower_limits) + lower_limits


class Leg2LiteSim2RealNode(Node):
    def __init__(self):
        super().__init__('leg2lite_sim2real_node')

        # ----------------------------------------------------------------------------------------
        # Load ONNX model
        # ----------------------------------------------------------------------------------------
        onnx_model_path = "policy.onnx"  # <-- Adjust path to your .onnx file
        self.session = ort.InferenceSession(onnx_model_path)

        # In your environment, action_space=2, observation_space=10
        self.current_obs = np.zeros(10, dtype=np.float32)

        # Real-time data storage
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_lin_acc = np.zeros(3, dtype=np.float32)
        self.imu_ang_vel = np.zeros(3, dtype=np.float32)

        # If you want to use real joint limits for scaling, set them here (example values).
        self.joint_lower_limits = np.array([-1.8195, 1.2755], dtype=np.float32)
        self.joint_upper_limits = np.array([ 1.8195,  -1.7122], dtype=np.float32)
        # By default, we WON'T apply this scaling; see control_loop().

        # ----------------------------------------------------------------------------------------
        # ROS 2 setup: Subscribers & Publisher
        # ----------------------------------------------------------------------------------------
        self.joint_state_sub = self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self.joint_state_cb, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, IMU_TOPIC, self.imu_cb, 10
        )
        self.cmd_pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)

        # ----------------------------------------------------------------------------------------
        # Timer for control loop (200 Hz here, adjust as needed)
        # ----------------------------------------------------------------------------------------
        self.timer_period = 0.005  # 200 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info("Leg2LiteSim2RealNode initialized.")

    def joint_state_cb(self, msg: JointState):
        # Update dictionaries with the latest joint state
        for i, name in enumerate(msg.name):
            # Safety check for velocity array length
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = vel

    def imu_cb(self, msg: Imu):
        # According to ROS2 Imu spec: linear_acceleration, angular_velocity
        self.imu_lin_acc[0] = msg.linear_acceleration.x
        self.imu_lin_acc[1] = msg.linear_acceleration.y
        self.imu_lin_acc[2] = msg.linear_acceleration.z

        self.imu_ang_vel[0] = msg.angular_velocity.x
        self.imu_ang_vel[1] = msg.angular_velocity.y
        self.imu_ang_vel[2] = msg.angular_velocity.z

        #print(self.imu_lin_acc[0])

    def control_loop(self):
        """
        1) Gather real sensor data (joint states, IMU).
        2) Normalize into the 10D observation as your environment's _get_observations() does.
        3) ONNX inference -> 2D action.
        4) Publish the action as position commands, "just using the actions for now".
        """
        # ------------------------------------------------------------------------------
        # 1) Gather
        # ------------------------------------------------------------------------------
        knee_pos = self.joint_positions.get(JOINT_NAME_KNEE, 0.0)
        knee_vel = self.joint_velocities.get(JOINT_NAME_KNEE, 0.0)

        #print(knee_pos)

        foot_pos = -self.joint_positions.get(JOINT_NAME_FOOT, 0.0)
        foot_vel = -self.joint_velocities.get(JOINT_NAME_FOOT, 0.0)
        #print(foot_pos)
        # ------------------------------------------------------------------------------
        # 2) Normalize Observations
        #    Matches environment’s code snippet in _get_observations():
        #       pos/pi, vel/10, imu_lin_acc/10, imu_ang_vel/10
        # ------------------------------------------------------------------------------
        obs_vec = np.array([
            knee_pos,
            knee_vel,
            foot_pos,
            foot_vel,
            self.imu_lin_acc[0],
            self.imu_lin_acc[1],
            self.imu_lin_acc[2],
            self.imu_ang_vel[0],
            self.imu_ang_vel[1],
            self.imu_ang_vel[2]
        ], dtype=np.float32)

        # ------------------------------------------------------------------------------
        # 3) ONNX Inference
        # ------------------------------------------------------------------------------
        input_name = self.session.get_inputs()[0].name
        #print(input_name)
        input_data = obs_vec.reshape(1, -1)  # shape [1,10]
        onnx_outputs = self.session.run(None, {input_name: input_data})
        # We assume the first output is the 2D action in [-1,1]
        raw_action = onnx_outputs[0]  # shape [1,2]
        action = raw_action[0]        # shape (2,)
        #print(action)

        # ------------------------------------------------------------------------------
        # (Optional) If you decide to scale from [-1,1] to joint limits, uncomment below:
        action = scale_actions_to_joint_range(
            action,
            self.joint_lower_limits,
            self.joint_upper_limits
        )

        # Invert the action for the foot (index 1)
        action[1] = -action[1]

        # ------------------------------------------------------------------------------
        # 4) Publish the 2D action as position commands
        #    Matches environment’s current _apply_action() usage.
        # ------------------------------------------------------------------------------
        cmd_msg = Float64MultiArray()
        # Interpreting model output as direct position
        cmd_msg.data = [float(action[0]), float(action[1])]

        #print(current_targets[1])
        print(action)
        self.cmd_pub.publish(cmd_msg)

        #print("working")

        # (Optional) Debug log
        # self.get_logger().info(f"obs={obs_vec}, action={action}")

def main(args=None):
    rclpy.init(args=args)
    node = Leg2LiteSim2RealNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

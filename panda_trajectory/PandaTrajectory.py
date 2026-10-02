""" Panda Cartesian Impedance Trajectory with Per-Segment Control Rate

This script commands a Franka Emika Panda robot using panda_py and
Cartesian impedance control to execute a sequence of Cartesian waypoints, along with Real-Time User Interaction,    
while maintaining a fixed tilted end-effector orientation throughout the motion.

NOTE:
- The tilt position of the end-effector is adjustable in the code.
- The orientation remains constant throughout the trajectory.
- Each waypoint segment can have its own control rate.

Features:
- Saves true Cartesian and joint home configuration
- Applies a fixed compound tilt to the end-effector
- Executes linear Cartesian interpolation (LERP) between waypoints
- Allows per-segment control rate specification
- Explicit Cartesian impedance stiffness matrix (tunable)
- Nullspace stabilization at known home joint configuration
- Allows user interaction throughout the trajectory
- Safely returns robot to joint home position

Requirements: 
- ROS 2
- panda_py with FCI enabled
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import os
import threading
import yaml
import csv
from scipy.spatial.transform import Rotation as R
import panda_py
from panda_py import controllers

# FIXED, TAUGHT HOME
KNOWN_HOME_Q = np.array([
    -0.00233163, -0.77575385, -0.00711294, -2.35147311, 0.02364864, 1.55989658, 0.75846475
])

# CONFIG FILE PATH
CONFIG_FILE = "/home/iit-rain/panda_ws/src/panda_trajectory/input_files/Laser_S.yaml"

# LOG FILE PATH
LOG_FILE_PATH = "/home/iit-rain/panda_ws/src/panda_trajectory/input_files/Trajectory_log/PandaTrajectory_log.csv"

class PandaTrajectoryNode(Node):
    """ ROS 2 node for commanding a Franka Panda robot along a Cartesian trajectory """

    def __init__(self, desk, panda):
        """ Initialize the PandaTrajectoryNode node with desk and panda objects"""
        super().__init__("panda_trajectory_1")
        print("=============== REAL-TIME USER CONTROL ===============")
        self.panda = panda
        self.desk = desk

        # LOAD CONFIG - YAML file
        with open(CONFIG_FILE, "r") as f:
            self.cfg = yaml.safe_load(f)
        self.steps = self.cfg["motion"]["steps_per_segment"]
        self.default_rate = self.cfg["motion"].get("default_control_rate_hz", 20)
        self.pause_indices = self.cfg["pause_behavior"]["pause_at_indices"]

        # STATE FLAGS
        self.is_paused = False
        self.user_home = False
        self.resume = False
        self.robot_moving = False

        # DATA LOGGING
        self.log_data = []

        # STARTUP SAFETY CHECK
        self.startup_check()

        # HOME POSE
        self.home_pos = self.panda.get_position()
        self.home_quat = self.panda.get_orientation()

        # ORIENTATION
        tilt = self.cfg["orientation"]
        q = R.from_quat(self.home_quat)
        q = q * R.from_euler("y", np.deg2rad(tilt["tilt_sideways_deg"]))
        q = q * R.from_euler("x", np.deg2rad(tilt["tilt_forward_deg"]))
        self.ee_quat = q.as_quat()

        # WAYPOINTS (OFFSETS)
        self.waypoint_offsets = []
        for wp in self.cfg["waypoints"]:
            self.waypoint_offsets.append(np.array(wp["offset"]))

        s = self.cfg["stiffness"]
        impedance_matrix = np.diag([
            s["translational_x"],
            s["translational_y"],
            s["translational_z"],
            s["rotational_x"],
            s["rotational_y"],
            s["rotational_z"]
        ]).astype(np.float64)
        damping_ratio = s["damping_ratio"]

        # Start Cartesian Impedance Controller
        self.ctrl = controllers.CartesianImpedance(
            impedance=impedance_matrix,
            damping_ratio=damping_ratio,
            # nullspace_stiffness=s["nullspace_stiffness"] 
        )
        self.panda.start_controller(self.ctrl)

        # Start UI thread
        threading.Thread(target=self.ui_thread, daemon=True).start()

        # START RELATIVE TIME
        self.start_time = time.time()

        self.run_trajectory()

    # STARTUP CHECK
    def startup_check(self):
        """ Ensure robot starts at known joint home """
        if not self.cfg["safety"]["require_home_on_start"]:
            return
        err = np.linalg.norm(self.panda.get_state().q - KNOWN_HOME_Q)
        if err > self.cfg["safety"]["max_joint_error_rad"]:
            self.get_logger().warn(f"⚠ Robot NOT at home (joint error = {err:.3f} rad)")
            input("Press \"ENTER\" to return home...")
            self.goto_home(save_log=True)

    # UI THREAD
    def ui_thread(self):
        """ User commands during trajectory execution """
        while True:
            key = input("USER INPUT: 'S'=pause | 'D'=resume | 'H'=home: ").strip().lower()
            if key == "s":
                self.is_paused = True
                self.get_logger().info("Trajectory Paused")
            elif key == "d":
                self.is_paused = False
                self.resume = True
                self.get_logger().info("Trajectory Resumed")
            elif key == "h":
                if not self.robot_moving or self.is_paused:
                    self.user_home = True
                else:
                    self.get_logger().warn("⚠ Robot moving! Pause first to go home")
            else:
                self.get_logger().warn(f"⚠ Invalid input: '{key}'")

    # TRAJECTORY
    def run_trajectory(self):
        """ Execute the Cartesian trajectory using linear interpolation """
        # Record true home ONCE from actual robot state
        actual_home = self.panda.get_position()

        for i in range(len(self.waypoint_offsets) - 1):
            # Actual robot position as segment start
            p0 = self.panda.get_position()
            # true home
            p1 = actual_home + self.waypoint_offsets[i + 1]

            # Get segment-specific control rate, or use default
            segment_rate = self.cfg["waypoints"][i + 1].get("control_rate_hz", self.default_rate)
            self.robot_moving = True

            for a in np.linspace(0, 1, self.steps):
                while self.is_paused:
                    if self.user_home:
                        self.goto_home(save_log=True)
                    time.sleep(0.1)

                pos = (1 - a) * p0 + a * p1
                self.ctrl.set_control(pos, self.ee_quat, KNOWN_HOME_Q)

                # DATA LOGGING (relative time)
                state = self.panda.get_state()
                ee_pos = self.panda.get_position()
                ee_quat = self.panda.get_orientation()
                self.log_data.append({
                    "time": time.time() - self.start_time,
                    "joint_positions": state.q.copy(),
                    "joint_velocities": state.dq.copy(),
                    "joint_torques": state.tau_J.copy(),
                    "ee_position": ee_pos.copy(),
                    "ee_orientation": ee_quat.copy()
                })

                if self.user_home:
                    self.goto_home(save_log=True)
                time.sleep(1 / segment_rate)

            self.robot_moving = False
            self.get_logger().info(f"✅ Reached waypoint {i + 1}")

            if i + 1 in self.pause_indices:
                self.resume = False
                while not self.resume:
                    time.sleep(0.1)
                    if self.user_home:
                        self.goto_home(save_log=True)

        # Save logs before returning home
        self.save_log_csv_splitted()
        self.goto_home(save_log=False)

    # SAVE LOG DATA TO CSV (splitted COLUMNS)
    def save_log_csv_splitted(self):
        """ 
        - time[s], q1-q7[rad], dq1-dq7[rad/s], tau1-tau7[Nm] 
        - ee_x, ee_y, ee_z [m] (end-effector position, XYZ) 
        - ee_qx, ee_qy, ee_qz, ee_qw [quaternion, XYZW] (end-effector orientation) 
        """
        if not self.log_data or len(self.log_data) == 0:
            self.get_logger().warn("log skipped: trajectory data not saved.")
            return
        try:
            base, ext = os.path.splitext(LOG_FILE_PATH)
            splitted_path = f"{base}_split{ext}"
            with open(splitted_path, "w", newline="") as f:
                writer = csv.writer(f)
                n_joints = 7

                # Row 1: Units / groups
                row_units = ["time (s)"]
                row_units += ["joint position (rad)"] * n_joints
                row_units += ["joint velocity (rad/s)"] * n_joints
                row_units += ["joint torque (Nm)"] * n_joints
                row_units += ["ee position (m)"] * 3
                row_units += ["ee orientation (quaternion)"] * 4
                writer.writerow(row_units)

                # Row 2: Column names
                row_names = ["time"]
                row_names += [f"q{i+1}" for i in range(n_joints)]
                row_names += [f"dq{i+1}" for i in range(n_joints)]
                row_names += [f"tau{i+1}" for i in range(n_joints)]
                row_names += ["ee_x", "ee_y", "ee_z"]
                row_names += ["ee_qx", "ee_qy", "ee_qz", "ee_qw"]
                writer.writerow(row_names)

                # Data rows
                for row in self.log_data:
                    data_row = [row.get("time", 0.0)]
                    data_row += np.array(row.get("joint_positions", np.zeros(n_joints))).tolist()
                    data_row += np.array(row.get("joint_velocities", np.zeros(n_joints))).tolist()
                    data_row += np.array(row.get("joint_torques", np.zeros(n_joints))).tolist()
                    data_row += np.array(row.get("ee_position", np.zeros(3))).tolist()
                    data_row += np.array(row.get("ee_orientation", np.array([0,0,0,1]))).tolist()
                    writer.writerow(data_row)
                f.flush()
            self.get_logger().info(f"Trajectory log saved to {splitted_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save splitted CSV: {e}")

    # Final return to joint-space home position
    def goto_home(self, save_log=True):
        """ Return robot to joint-space home position and optionally save logs """
        if save_log:
            self.save_log_csv_splitted()
        joint_ctrl = controllers.JointPosition()
        self.panda.start_controller(joint_ctrl)
        self.panda.move_to_joint_position(KNOWN_HOME_Q)
        self.get_logger().info("🏠 Robot at home position")
        self.get_logger().info("🚨 Rerun the program from the terminal to continue")
        exit(0)
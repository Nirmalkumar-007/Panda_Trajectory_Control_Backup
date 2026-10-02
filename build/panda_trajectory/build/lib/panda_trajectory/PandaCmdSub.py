""" Panda Cartesian Impedance Trajectory Script 

This script commands a Franka Emika Panda robot using panda_py and
Cartesian impedance control to execute a sequence of Cartesian waypoints
while maintaining a fixed tilted end-effector orientation throughout the motion.

NOTE:
- The tilt position of the end-effector is adjustable in the code.
- The orientation remains constant throughout the trajectory.

Features:
- Saves true Cartesian and joint home configuration
- Applies a fixed compound tilt to the end-effector
- Executes linear Cartesian interpolation (LERP) between waypoints
- Allows user interaction at intermediate waypoints
- Safely returns robot to joint home position

Requirements: 
- ROS 2
- panda_py with FCI enabled """

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

import panda_py
from panda_py import controllers

class PandaCmdSub(Node):
    """ ROS 2 node for commanding a Franka Panda robot along a Cartesian trajectory
      with a fixed tilted end-effector orientation using Cartesian impedance control """
    
    def __init__(self, desk: panda_py.Desk, panda: panda_py.Panda):
        super().__init__('panda_cmd_sub_tilted')

        self.desk = desk
        self.panda = panda

        # Save true home configuration (Cartesian + Joint space), used as:
        # - Reference orientation for applying the end-effector tilt
        # - Safe return point to the original robot configuration
        self.home_pos = self.panda.get_position()
        self.home_quat = self.panda.get_orientation()
        self.home_q = self.panda.get_state().q.copy()

        self.get_logger().info(f"True Home Cartesian: {self.home_pos}")
        self.get_logger().info(f"True Home Joints: {self.home_q}")

        # End-effector tilt definition (USER-ADJUSTABLE) - in degrees
        tilt_sideways_deg = -10 # rotation about EE Y-axis (left/right)
        tilt_forward_deg = 7  # rotation about EE X-axis (forward/back)

        # Convert tilt angles from degrees to radians
        tilt_sideways_rad = np.deg2rad(tilt_sideways_deg)
        tilt_forward_rad = np.deg2rad(tilt_forward_deg)

        # Create rotation objects
        q_current = R.from_quat(self.home_quat)
        q_sideways = R.from_euler('y', tilt_sideways_rad)  # Y-axis rotation
        q_forward = R.from_euler('x', tilt_forward_rad)   # X-axis rotation

        # Rotation composition:
        # current_orientation * sideways_tilt * forward_tilt
        # - Scipy active rotation convention
        # - End-effector frame application
        # - Orientation remains fixed along the trajectory
        q_tilted = q_current * q_sideways * q_forward
        self.tilted_quat = q_tilted.as_quat()

        # Start Cartesian Impedance Controller
        # The controller is responsible for compliant Cartesian motion
        self.ctrl = controllers.CartesianImpedance()
        self.panda.start_controller(self.ctrl)

        # Waypoint definition (Cartesian space)
        # - Waypoints are defined as offsets from the home position, expressed in the robot base frame
        # - Trajectory sequence: (Home → A → B → A → B → A → B → Home)
        self.point_A = self.home_pos + np.array([0.235, -0.24, -0.37])
        self.point_B = self.point_A + np.array([0.0, 0.65, 0.0])

        #self.point_A = self.home_pos + np.array([0.235, -0.24, -0.41])  working
        #self.point_B = self.point_A + np.array([0.0, 0.7, 0.0])
                
        #self.point_A = self.home_pos + np.array([0.235, -0.24, -0.4])
        #self.point_B = self.point_A + np.array([0.0, 0.7, 0.0])
        
        #self.point_A = self.home_pos + np.array([0.2, -0.41, -0.45])
        #self.point_B = self.point_A + np.array([0.0, 0.9, 0.0])

        #self.point_A = self.home_pos + np.array([0.26, -0.41, -0.20])
        #self.point_B = self.point_A + np.array([0.0, 0.9, 0.0])

        self.waypoints = [
            self.home_pos,
            self.point_A,
            self.point_B,
            self.point_A,
            self.point_B,
            #self.point_A,
            #self.point_B,
            self.home_pos ]
        # Orientation assignment
        # The same tilted orientation is maintained at all waypoints
        self.orientations = [self.tilted_quat] * len(self.waypoints)

        # Motion resolution        
        # Number of interpolation steps per Cartesian segment
        self.steps = 160  # Adjustable smoothness parameter

        # Reference values (empirical tuning):
        # self.steps = 10   # Very rough
        # self.steps = 30   # Fast, not smooth
        # self.steps = 60   # Good
        # self.steps = 100  # Very smooth
        # self.steps = 200  # Very smooth but slow

        self.run_trajectory()

    def run_trajectory(self):
        """ Execute the Cartesian trajectory using linear interpolation.
        Motion is paused at selected waypoints and waits for user input:
        'f' → continue trajectory,  'h' → return robot to home position  """

        for i in range(len(self.waypoints) - 1):
            p0 = self.waypoints[i]
            p1 = self.waypoints[i + 1]

            # Fixed orientation along the entire segment
            quat_fixed = self.orientations[i]
            
            # Linear Cartesian interpolation (LERP)
            for a in np.linspace(0, 1, self.steps):
                pos = (1 - a) * p0 + a * p1
                quat = quat_fixed  # Maintain constant tilt
                self.ctrl.set_control(pos, quat)

                # Control rate definition
                CONTROL_RATE_HZ = 20
                time.sleep(1.0 / CONTROL_RATE_HZ) # Control loop rate chosen for slow and safe impedance motion
                #time.sleep(1)
                #time.sleep(0.1)  # 10x slower motion (~10 Hz)
                #time.sleep(0.08) # safe and slower motion (~12.5 Hz)
                #time.sleep(0.06) # 3x slower motion (~16 Hz)
                #time.sleep(0.02) # slower motion (~50 Hz)

            # Waypoint reached
            self.get_logger().info(f"Reached waypoint {i + 1}")

            # User interaction at intermediate waypoints
            if i + 1 in [1, 2, 3, 4]:
                while True:
                    user_input = input(
                        "Press 'f' to continue trajectory or 'h' to return home: "
                    ).strip().lower()

                    if user_input == 'f':
                        self.get_logger().info("Continuing trajectory...")
                        break

                    elif user_input == 'h':
                        self.get_logger().info("Returning to home position...")

                        # Switch to joint position controller
                        joint_ctrl = controllers.JointPosition()
                        self.panda.start_controller(joint_ctrl)

                        # Move back to saved joint home configuration
                        self.panda.move_to_joint_position(self.home_q)
                        self.get_logger().info("Robot is at home position")
                        exit(0)  # Stop execution immediately

        # Final return to joint-space home position
        self.get_logger().info("Returning to home position")
        joint_ctrl = controllers.JointPosition()
        self.panda.start_controller(joint_ctrl)
        self.panda.move_to_joint_position(self.home_q)
        self.get_logger().info("Robot is at home position")


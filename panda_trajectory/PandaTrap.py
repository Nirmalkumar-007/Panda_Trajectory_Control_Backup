"""Panda Cartesian Impedance Trajectory with Trapezoidal Velocity Profile and Post-Segment Tilt

This script commands a Franka Emika Panda robot using panda_py and
Cartesian impedance control to execute a sequence of Cartesian waypoints
loaded from a YAML configuration file, while supporting smooth pre & post-arrival
end-effector orientation.

NOTE:
- All waypoints, stiffness, orientation, and timing are defined in the YAML config file.
- The base end-effector tilt is set at startup and held throughout motion segments.
- Post-arrival tilts can be applied per-waypoint after motion is complete.
- All post-tilt angles are absolute relative to the startup orientation.

Features:
- YAML-driven configuration: waypoints, stiffness, orientation, timing, pause behaviour
- Trapezoidal velocity profile for smooth acceleration and deceleration
- Linear Cartesian interpolation (LERP) between waypoints driven by the profile
- Post-arrival SLERP orientation transitions at configurable waypoints
- Per-segment duration and control rate specification
- Explicit Cartesian impedance stiffness matrix (tunable per axis)
- Nullspace stabilization at known home joint configuration
- Real-time user interface: pause, resume, and emergency home via keyboard
- Trajectory data logging to split-header CSV (joints, EE pose, force/torque, commands)
- Safely returns robot to joint home position on completion or upon user request

Requirements:
- ROS 2
- panda_py with FCI enabled """

import rclpy
from rclpy.node import Node
import numpy as np
import time
import os
import threading
import yaml
import csv
from scipy.spatial.transform import Rotation as R, Slerp  
import panda_py
from panda_py import controllers

KNOWN_HOME_Q = np.array([
    -0.00233163, -0.77575385, -0.00711294, -2.35147311, 0.02364864,  1.55989658,  0.75846475
])

#Yaml input file
CONFIG_FILE   = "/home/iit-rain/Desktop/CODE REV 2 - TRAP- TILT/ss_bosch.yaml"
#CSV output file
LOG_FILE_PATH = "/home/iit-rain/Desktop/CODE REV 2 - TRAP- TILT/CSV_logs/PandaTrajectory_log.csv"

    # Normalized displacement s ∈ [0, 1] over total time t_f.
    # Profile: accelerate from 0 → s_dot_max over [0, t1],
    #          cruise at s_dot_max over [t1, t2],
    #          decelerate from s_dot_max → 0 over [t2, t_f].
    #
    # From the displacement constraint (total area = 1):
    #   s_dot_max² - (s_ddot_max * t_f) * s_dot_max + s_ddot_max = 0
    #
    # Minimum feasible time (triangle profile, zero cruise phase):
    #   t_f_min = 2 / sqrt(s_ddot_max)

def build_profile_from_duration(t_f_requested: float, s_ddot_max: float) -> dict:
    """ Build a symmetric trapezoidal velocity profile that covers normalized
    distance s=1 in exactly t_f seconds (or t_f_min if the request is too short).
    t_f_requested : float   Desired total duration [s]
    s_ddot_max    : float   Max normalized acceleration [1/s²]
    dict with keys: t1, t2, t_f, s_dot_max, s_ddot_max, s1, s2 """
    t_f_min = 2.0 / np.sqrt(s_ddot_max)

    if t_f_requested < t_f_min:
        # Clamp to minimum time
        t_f_actual = t_f_min
        s_dot_max  = np.sqrt(s_ddot_max)
        t1 = t2    = t_f_actual / 2.0
    else:
        # Solve quadratic: v² - (a·T)·v + a = 0
        # where v = s_dot_max, a = s_ddot_max, T = t_f_requested
        a   = s_ddot_max
        T   = t_f_requested
        # discriminant = (aT)² - 4a
        discriminant = (a * T) ** 2 - 4.0 * a
        # Take the smaller root (physically correct: v_max ≤ a·T/2)
        s_dot_max  = (a * T - np.sqrt(discriminant)) / 2.0
        t_f_actual = T
        t1 = s_dot_max / a
        t2 = t_f_actual - t1           # symmetric: deceleration starts at T - t1

    # Milestone displacements
    s1 = 0.5 * s_ddot_max * t1 ** 2    # end of acceleration phase
    s2 = s1 + s_dot_max * (t2 - t1)    # end of cruise phase

    return dict(
        t1=t1, t2=t2, t_f=t_f_actual,
        s_dot_max=s_dot_max, s_ddot_max=s_ddot_max,
        s1=s1, s2=s2
    )

def sample_profile(prof: dict, t: float) -> float:
    """ Return the normalized progress s ∈ [0, 1] at time t.
    Clamped so that t outside [0, t_f] saturates at 0 or 1. """
    t = float(np.clip(t, 0.0, prof["t_f"]))
    if t <= prof["t1"]:
        # Acceleration phase
        return 0.5 * prof["s_ddot_max"] * t ** 2
    elif t <= prof["t2"]:
        # Constant-velocity (cruise) phase
        return prof["s1"] + prof["s_dot_max"] * (t - prof["t1"])
    else:
        # Deceleration phase
        dt3 = t - prof["t2"]
        return prof["s2"] + prof["s_dot_max"] * dt3 - 0.5 * prof["s_ddot_max"] * dt3 ** 2


class PandaTrapNode(Node):
    def __init__(self, panda):
        super().__init__("panda_trajectory")
        print("\n" + "=" * 60)
        print("                 REAL-TIME USER INTERFACE            ")
        print("=" * 60 + "\n")
        self.panda = panda

        with open(CONFIG_FILE, "r") as f:
            self.cfg = yaml.safe_load(f)

        mot = self.cfg["motion"]
        self.default_duration = float(mot.get("default_segment_duration_s", 15.0))
        self.default_s_ddot   = float(mot.get("default_s_ddot_max", 0.08))
        self.default_rate     = float(mot.get("default_control_rate_hz", 100.0))
        self.pause_indices    = self.cfg["pause_behavior"]["pause_at_indices"]

        # Control flags (written by ui_thread, read by run_trajectory)
        self.is_paused    = False
        self.user_home    = False
        self.resume       = False
        self.robot_moving = False
        self.log_data = []
        self.startup_check()

        # Orientation: apply user-defined tilts to the current EE orientation 
        tilt = self.cfg["orientation"]
        q = R.from_quat(self.panda.get_orientation())


        #updated
        q_base = R.from_quat(self.panda.get_orientation())
        q_tilt = R.from_euler("xyz", [
            np.deg2rad(tilt["tilt_sideways_deg"]),
            np.deg2rad(tilt["tilt_forward_deg"]),
            np.deg2rad(tilt.get("tilt_roll_deg", 0.0))
        ])
        q = q_tilt * q_base
        self.ee_quat = q.as_quat()
        #print(R.from_quat(self.panda.get_orientation()).as_matrix())
        #updated

        # q = q * R.from_euler("y", np.deg2rad(tilt["tilt_sideways_deg"]))
        # q = q * R.from_euler("x", np.deg2rad(tilt["tilt_forward_deg"]))
        # q = q * R.from_euler("z", np.deg2rad(tilt.get("tilt_roll_deg", 0.0))) 
        # self.ee_quat = q.as_quat()  # active orientation — updated after post-arrival tilts
        self.ee_quat_home = self.ee_quat.copy() 
        # Waypoint offsets relative to home
        self.waypoint_offsets = [np.array(wp["offset"]) for wp in self.cfg["waypoints"]]

        # Cartesian Impedance Controller
        s = self.cfg["stiffness"]
        self.ctrl = controllers.CartesianImpedance(
            impedance=np.diag([
                s["translational_x"], s["translational_y"], s["translational_z"],
                s["rotational_x"],    s["rotational_y"],    s["rotational_z"]
            ]).astype(np.float64),
            damping_ratio=s["damping_ratio"],
            nullspace_stiffness=s.get("nullspace_stiffness", 1.0)
        )
        self.panda.start_controller(self.ctrl)

        threading.Thread(target=self.ui_thread, daemon=True).start()
        self.start_time = time.time()
        self.run_trajectory()

    def startup_check(self):
        """Verify robot is at home before starting. If not, send it there."""
        if not self.cfg["safety"]["require_home_on_start"]:
            return
        err = np.linalg.norm(self.panda.get_state().q - KNOWN_HOME_Q)
        if err > self.cfg["safety"]["max_joint_error_rad"]:
            self.get_logger().warn(f"⚠  Robot NOT at home (joint error = {err:.3f} rad)")
            input(">>> SAFETY: Press [ENTER] to send robot to HOME position... ")
            self.goto_home(save_log=False)

    def ui_thread(self):
        """Blocking input loop running in a daemon thread."""
        while True:
            key = input("USER INPUT: 'S'= pause | 'D'= resume | 'H'= home: \n").strip().lower()
            if key == "s":
                self.is_paused = True
                self.get_logger().info("⏸  Paused")
            elif key == "d":
                self.is_paused = False
                self.resume    = True
                self.get_logger().info("▶  Resumed")
            elif key == "h":
                if not self.robot_moving or self.is_paused:
                    self.user_home = True
                else:
                    self.get_logger().warn("Robot is moving — pause (S) first before returning home.")
            else:
                self.get_logger().warning(f"⚠  Invalid key '{key}' pressed. Please use 'S', 'D', or 'H'.")


    def _apply_tilt_in_place(self, p_hold: np.ndarray, tilt_sideways_deg: float, tilt_forward_deg: float, tilt_roll_deg: float = 0.0):
        """ After arriving at a waypoint, smoothly transition the EE orientation
        to the requested tilt while holding the Cartesian position fixed.

        Uses SLERP between the current active ee_quat and the new target
        quaternion, driven by the same trapezoidal profile as motion segments.
        Duration and rate are taken from the optional [post_tilt] YAML block
        (defaults: 5 s, same control rate as motion).

        Once complete, self.ee_quat is updated to the new orientation so that
        subsequent motion segments maintain it.

        p_hold            : Cartesian position to hold [m]
        tilt_sideways_deg : sideways tilt to apply (y-axis rotation) [deg]
        tilt_forward_deg  : forward tilt to apply  (x-axis rotation) [deg] """

        tilt_cfg = self.cfg.get("post_tilt", {})
        duration = float(tilt_cfg.get("duration_s",       5.0))
        rate     = float(tilt_cfg.get("control_rate_hz",  self.default_rate))
        dt       = 1.0 / rate

        # Build target quaternion — same convention as __init__:
        # start from live EE orientation and apply the requested tilt angles.
        #updated
        q_tilt = R.from_euler("xyz", [
            np.deg2rad(tilt_sideways_deg),
            np.deg2rad(tilt_forward_deg),
            np.deg2rad(tilt_roll_deg)
        ])
        q_target_quat = (q_tilt * R.from_quat(self.ee_quat.copy())).as_quat()
        #updated
        print(R.from_quat(self.panda.get_orientation()).as_matrix())
        # q_target = R.from_quat(self.panda.get_orientation()) # from the current posiiton - live orientation
        
        # q_target = R.from_quat(self.ee_quat.copy())            # from commanded pose
        # q_target = q_target * R.from_euler("y", np.deg2rad(tilt_sideways_deg))
        # q_target = q_target * R.from_euler("x", np.deg2rad(tilt_forward_deg))
        # q_target = q_target * R.from_euler("z", np.deg2rad(tilt_roll_deg))    
        # q_target_quat = q_target.as_quat()

        q_start = R.from_quat(self.ee_quat.copy())
        q_end   = R.from_quat(q_target_quat)

        prof = build_profile_from_duration(duration, self.default_s_ddot)

        # Build the Slerp interpolator once, outside the loop  ← FIX
        slerp_fn = Slerp([0.0, 1.0], R.concatenate([q_start, q_end]))

        print(f"\n     Post-arrival tilt: sideways={tilt_sideways_deg}°  forward={tilt_forward_deg}°  roll={tilt_roll_deg}°")
        print(f"     Tilt duration: {prof['t_f']:.2f}s")

        tilt_start     = time.time()
        elapsed_paused = 0.0
        t_tilt         = 0.0

        while t_tilt < prof["t_f"]:
            loop_start = time.time()

            # Pause handling (consistent with motion loop)
            if self.is_paused:
                pause_wall = time.time()
                while self.is_paused:
                    if self.user_home:
                        self.goto_home(save_log=True)
                    time.sleep(0.01)
                elapsed_paused += (time.time() - pause_wall)

            t_tilt = time.time() - tilt_start - elapsed_paused
            s_val  = sample_profile(prof, t_tilt)
            
            s_val = np.clip(s_val, 0.0, 1.0)
            # SLERP: interpolate orientation from start → target  ← FIX
            q_interp = slerp_fn(s_val).as_quat()

            self.ctrl.set_control(p_hold, q_interp, KNOWN_HOME_Q)

            if self.user_home:
                self.goto_home(save_log=True)

            while (time.time() - loop_start) < (dt - 0.0005):
                time.sleep(0.0001)
            while (time.time() - loop_start) < dt:
                pass

        # Lock in the new orientation for all subsequent segments
        self.ee_quat = q_target_quat.copy()
        print(f"     🔁 Tilt Applied")


    def run_trajectory(self):
        """
        1. `actual_home` is captured once from the live robot position at startup.
           All waypoint targets are absolute: actual_home + offset[i].

        2. `p0` for each segment is the COMMANDED end-point of the previous
           segment (i.e. p1 of the prior leg), NOT the live robot position.
           This prevents discontinuous jumps caused by impedance-control lag.

        3. After reaching a waypoint the robot is given a short settling window
           so that the actual position converges to the commanded position before
           the next segment starts.

        4. Timing uses a high-precision busy-wait so that the control loop runs
           at exactly the configured rate (typically 100 Hz).

        5. If a waypoint defines `post_tilt_sideways_deg` or `post_tilt_forward_deg`,
           the robot holds its arrived position and smoothly tilts the EE orientation
           AFTER the motion segment — no change to position or motion timing. """
        actual_home = self.panda.get_position()
        n_segments  = len(self.waypoint_offsets) - 1  # exclude HOME entry (index 0)

        # p0 starts from the true robot position (at home, so lag is zero here)
        p0 = actual_home.copy()

        for i in range(n_segments):
            wp_cfg = self.cfg["waypoints"][i + 1]

            # Absolute Cartesian target for this segment
            p1 = actual_home + self.waypoint_offsets[i + 1]

            t_f_req = float(wp_cfg.get("segment_duration_s", self.default_duration))
            rate    = float(wp_cfg.get("control_rate_hz",    self.default_rate))
            dt      = 1.0 / rate

            prof = build_profile_from_duration(t_f_req, self.default_s_ddot)

            self.robot_moving  = True
            seg_start          = time.time()
            elapsed_paused     = 0.0
            t_seg              = 0.0

            print(f"\n  ▶  Segment {i+1}/{n_segments}: '{wp_cfg['name']}'")
            print(f"     Relative Target: {wp_cfg['offset']}")
            print(f"     Duration: {prof['t_f']:.2f}s  |  v_max={prof['s_dot_max']:.4f}/s")

            # self.ee_quat is used unchanged for the entire segment.
            # Any per-waypoint tilt is applied AFTER the segment ends (post-arrival).
            while t_seg < prof["t_f"]:
                loop_start = time.time()

                # Pause handling
                if self.is_paused:
                    pause_wall = time.time()
                    while self.is_paused:
                        if self.user_home:
                            self.goto_home(save_log=True)
                        time.sleep(0.01)
                    elapsed_paused += (time.time() - pause_wall)

                # Profile sampling
                # t_seg excludes time spent paused, giving accurate motion timing.
                t_seg = time.time() - seg_start - elapsed_paused
                s_val = sample_profile(prof, t_seg)
                # if s_val > 0.9:
                    # print(f"     Almost at waypoint")

                p_cmd = p0 + s_val * (p1 - p0)
                self.ctrl.set_control(p_cmd, self.ee_quat, KNOWN_HOME_Q)

                # Data logging 
                state = self.panda.get_state()
                ee_force_torque = state.O_F_ext_hat_K
                self.log_data.append({
                    "time":             time.time() - self.start_time,
                    "joint_positions":  state.q.copy(),
                    "joint_velocities": state.dq.copy(),
                    "joint_torques":    state.tau_J.copy(),
                    "ee_position":      self.panda.get_position().copy(),
                    "ee_orientation":   self.panda.get_orientation().copy(),
                    "ee_force_torque":  ee_force_torque.copy(),
                    "cmd_position":     p_cmd.copy(),
                    "s_val":            s_val,
                })

                # Emergency home command during motion
                if self.user_home:
                    self.goto_home(save_log=True)

                # High-precision busy-wait to enforce loop rate
                while (time.time() - loop_start) < (dt - 0.0005):
                    time.sleep(0.0001)
                while (time.time() - loop_start) < dt:
                    pass
                
            # End of segment 
            self.robot_moving = False
            self.get_logger().info(f"✅  Reached waypoint {i + 1} ('{wp_cfg['name']}')")

            # Update p0 to the commanded endpoint, not the live robot position.
            p0 = p1.copy()

            # Post-arrival tilt (optional, per-waypoint in YAML) 
            # Executes only after segment motion is fully complete.
            # Holds p1 fixed and SLERPs orientation to the requested tilt.
            # self.ee_quat is updated on completion → carried into next segments.
            if "post_tilt_sideways_deg" in wp_cfg or \
               "post_tilt_forward_deg" in wp_cfg or \
               "post_tilt_roll_deg"     in wp_cfg:
                tilt_side = float(wp_cfg.get("post_tilt_sideways_deg", 0.0))
                tilt_fwd  = float(wp_cfg.get("post_tilt_forward_deg",  0.0))
                tilt_roll = float(wp_cfg.get("post_tilt_roll_deg",     0.0))
                self._apply_tilt_in_place(p1, tilt_side, tilt_fwd, tilt_roll)

            # Optional pause at waypoint
            if i + 1 in self.pause_indices:
                print(f"\n USER INPUT: 'D'=resume | 'H'=home:\n")
                self.resume = False
                while not self.resume:
                    if self.user_home:
                        self.goto_home(save_log=True)
                    time.sleep(0.1)

        print("\n  ✅  All waypoints completed.")
        self.goto_home(save_log=True)

    def save_log_csv_splitted(self):
        """Write the trajectory log to a split-header CSV file."""
        if not self.log_data:
            return
        
        try:
            base, ext = os.path.splitext(LOG_FILE_PATH)
            #path = f"{base}_split{ext}" to save file with the same name
            
            # Generate incremental filename
            counter = 1
            while True:
                path = f"{base}_split_{counter}{ext}"
                if not os.path.exists(path):
                    break
                counter += 1

            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                n = 7
                # Row 1: grouped column headers
                writer.writerow(
                    ["time (s)"]
                    + ["joint position (rad)"]   * n
                    + ["joint velocity (rad/s)"] * n
                    + ["joint torque (Nm)"]      * n
                    + ["ee position (m)"]        * 3
                    + ["ee orientation (quat)"]  * 4
                    + ["ee force/torque (N, Nm)"]  * 6
                    + ["cmd position (m)"]       * 3
                    + ["s_val"]
                )
                # Row 2: individual column names
                writer.writerow(
                    ["time"]
                    + [f"q{j+1}"   for j in range(n)]
                    + [f"dq{j+1}"  for j in range(n)]
                    + [f"tau{j+1}" for j in range(n)]
                    + ["ee_x",  "ee_y",  "ee_z"]
                    + ["ee_qx", "ee_qy", "ee_qz", "ee_qw"]
                    + ["fx", "fy", "fz", "tx", "ty", "tz"]
                    + ["cmd_x", "cmd_y", "cmd_z"]
                    + ["s"]
                )
                for row in self.log_data:
                    writer.writerow(
                        [row["time"]]
                        + list(row["joint_positions"])
                        + list(row["joint_velocities"])
                        + list(row["joint_torques"])
                        + list(row["ee_position"])
                        + list(row["ee_orientation"])
                        + list(row["ee_force_torque"])
                        + list(row["cmd_position"])
                        + [row["s_val"]]
                    )
            self.get_logger().info(f"Trajectory log saved → {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

    def goto_home(self, save_log: bool = True):
        """ Save the trajectory log (optional), send robot to KNOWN_HOME_Q
           via joint position controller, then hard-exit the process """
        if save_log:
            self.save_log_csv_splitted()
        self.panda.start_controller(controllers.JointPosition())
        self.panda.move_to_joint_position(KNOWN_HOME_Q)
        self.get_logger().info("🏠 Robot at home position")
        self.get_logger().info("🚨 Rerun the program from the terminal to continue")
        os._exit(0)
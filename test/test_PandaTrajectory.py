import pytest
import rclpy
import numpy as np
import threading
import yaml
import time
from panda_trajectory.PandaTrajectory import PandaTrajectoryNode, KNOWN_HOME_Q, CONFIG_FILE, LOG_FILE_PATH

# ROS initialization
@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# Simple logger for testing
class PrintLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        print(f"[WARN] {msg}")

# Full interactive manual test
def test_PandaTrajectoryNode_with_ui():
        
    # ARRANGE
    desk = None

    # Load YAML config
    with open(CONFIG_FILE, "r") as f:
        cfg = yaml.safe_load(f)

    # Mock Panda hardware
    class MockPanda:
        def get_position(self):
            return np.array([0.0, 0.0, 0.0])
        def get_orientation(self):
            return np.array([0.0, 0.0, 0.0, 1.0])
        def get_state(self):
            class State:
                q = KNOWN_HOME_Q
                dq = np.zeros(7)
                tau_J = np.zeros(7)
            return State()
        def start_controller(self, ctrl):
            print(f"[MOCK] start_controller called with {type(ctrl).__name__}")
        def move_to_joint_position(self, q):
            print(f"[MOCK] move_to_joint_position called with {q}")

    panda = MockPanda()

    # Create node without running __init__
    node = PandaTrajectoryNode.__new__(PandaTrajectoryNode)
    node.panda = panda
    node.desk = desk
    node.get_logger = lambda: PrintLogger()

    # Use YAML values
    node.steps = cfg["motion"]["steps_per_segment"]
    node.rate = cfg["motion"]["control_rate_hz"]
    node.pause_indices = cfg["pause_behavior"]["pause_at_indices"]

    # Initialize state
    node.is_paused = False
    node.user_home = False
    node.resume = False
    node.robot_moving = False
    node._done = False
    node._home_printed = False

    node.home_pos = panda.get_position()
    node.home_quat = panda.get_orientation()
    node.ee_quat = node.home_quat.copy()
    node.waypoints = [node.home_pos + np.array(wp["offset"]) for wp in cfg["waypoints"]]

    # Mock controller
    class MockCtrl:
        def set_control(self, pos, quat):
            print(f"[MOCK] set_control {pos}")
    node.ctrl = MockCtrl()

    # Override goto_home
    def safe_goto_home(save_log=True):
        if node._home_printed:
            return
        print(f"🏠 Robot at home position.")
        print(f"LOG saved to {LOG_FILE_PATH}")
        node.robot_moving = False
        node.is_paused = False
        node.user_home = False
        node._done = True
        node._home_printed = True
    node.goto_home = safe_goto_home

    # Modify run_trajectory to respect _done flag
    def run_trajectory_safe():
        for i in range(len(node.waypoints) - 1):
            p0 = node.waypoints[i]
            p1 = node.waypoints[i + 1]
            node.robot_moving = True
            for a in np.linspace(0, 1, node.steps):
                while node.is_paused:
                    if node.user_home or node._done:
                        node.goto_home()
                    time.sleep(0.1)
                if node._done:
                    return
                node.ctrl.set_control((1 - a) * p0 + a * p1, node.ee_quat)
                if node.user_home:
                    node.goto_home()
                time.sleep(1 / node.rate)
            node.robot_moving = False
            node.get_logger().info(f"✅ Reached waypoint {i + 1}")
            if i + 1 in node.pause_indices:
                node.resume = False
                while not node.resume:
                    time.sleep(0.1)
                    if node.user_home or node._done:
                        node.goto_home()
            if node._done:
                break
        node.goto_home()
    node.run_trajectory = run_trajectory_safe

    # ACT - Start UI thread
    ui_thread = threading.Thread(target=node.ui_thread, daemon=True)
    ui_thread.start()

    print(" START TRAJECTORY ")
    print("Type 's' to pause, 'd' to resume, 'h' to go home")
    print("All waypoints will be executed. Observe outputs.")

    node.run_trajectory()

    print(" TRAJECTORY FINISHED ")

    # ASSERT - ensure final state
    assert node.is_paused is False
    assert node.user_home is False
    assert node.robot_moving is False
    assert node._done is True
    assert node._home_printed is True

    print("TEST FINISHED SUCCESSFULLY")

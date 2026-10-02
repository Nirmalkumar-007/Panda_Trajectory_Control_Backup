This folder `panda_trajectory` contains the core classes and logic for controlling the Panda robot, executing trajectories, and handling user interactions.

### **PandaCmdSub .py**

- Move robot to specified waypoints with desired orientations.

- Process user inputs:
 
  - `f` to move forward along the trajectory
 
  - `h` to return home and terminate execution

- Send joint position commands to the robot controller & Handle exit logic safely.

**Inputs:** Waypoints, orientations, and user input commands.

**Outputs:** Basic Trajectory, and program termination.

**Notes:** Designed to test the basic functionalities with user input for safe behaviour.

---

### **PandaTrajectory .py**

- Implements a trajectory-following node for the Panda robot, allowing smooth execution of movements along specified paths with configurable speed and step size.
- Move the robot through a sequence of waypoints with correct end-effector orientation.

- Real - Time user control:
    - `s` to pause trajectory execution
    - `d` to resume trajectory execution
    - `h` to return home and exit
    - `Enter` to return home, only when the robot is not at home position when the program is started

- Record and log robot states:
    - Joint positions, velocities, torques, End-effector positions and orientations, and Timestamps for each control step.

- Support adjustable parameters (speed, step size, and waypoint list) for flexible testing and deployment.

**Inputs:** Waypoints, orientations, speed, step size, and user input commands.

**Outputs:** Robot motion, state logs, and program termination on command.

**Notes:** Configurable through YAML parameters `BristleBlaster.yaml` or any required `.yaml` to optimize performance for either testing or real-world execution.


### **panda_trajectory**
---
### **Overview**
- This repository provides tools to control a Panda robot arm along predefined Cartesian trajectories.

- Trajectory behavior:

  - Starts from the true home configuration

  - Follows a sequence of Cartesian waypoints, for example (Home → A → B → A → B → Home)

  - Pauses at selected waypoints for user input
  
  - Executes the predefined trajectory, with Safe return to the home.

---

### **Folder Structure**


- `app/ run_PandaTrajectory.py` – main function to execute the predefined Robot trajectory.

- `launch/ launch_PandaTrajectoryNode.py` – ROS 2 launch files.

- `panda_trajectory/ PandaTrajectory.py` – Core Python class controlling the Panda robot (handling waypoints, trajectory execution, and robot commands).

- `input files/****.yaml` - This file Contains the configuration YAML file to define the Robot motion parameters, Waypoints, Orientation, Safety & pause behavior of the robot.

- `test/ test_PandaTrajectory.py` – Test scripts for validating robot commands.



- Root files `setup.py, setup.cfg, package.xml` – For installing the Python and ROS 2 package.


---
### **⚠️ Safety Checklist Before Running the Program**

Before running the Robot, must ensure the following:
- Crosscheck if the mass of the tool is defined in the settings of  desk at IP `192.168.1.11`
-   desk → settings → End-Effector → Mass (kgs)
  

Crosscheck the values in the `.yaml` file.

 **Control Rate:**
 
  - Start with a safe control rate (e.g., 10–25 Hz). 

  - Gradually increase the speed to check the robot’s behavior.

**Steps per Segment:**

  - Increase them gradually to monitor the robot’s motion.

  - Recommended range: 130–160 for smooth operation.

**Emergency Button** 
- Always keep the emergency stop button accessible.

---
### **Requirements**

Before execution, ensure the following are installed:

- Python 3
```
sudo apt install python3
```
- pip
```
sudo apt-get install python3-pip
```
- Python libraries
```
pip3 install numpy scipy panda_py PyYAML
```


- ROS 2 Jazzy Jalisco (if using ROS workspace commands)
```
sudo apt install ros-jazzy-desktop
```
---

### **Usage** 


### 1. Direct Python Execution 

Run from Terminal:
- For running main node
 ```bash
 iit-rain@iit-rain-007:~$ /bin/python3 /home/iit-rain/panda_ws/src/panda_trajectory/app/run_PandaTrajectory.py
 
  ```
- For running test 
 ```bash
  iit-rain@iit-rain-007:~/panda_ws/src/panda_trajectory$ pytest -s test/test_PandaTrajectory.py 
  ```
    


 ----

### 2. Execution using ROS 2  

This mode executes trajectory using ROS

- Build and source the workspace:

 ```
source install/setup.bash
colcon build --symlink-install
 ```
 
-  Run 

 ```
ros2 run panda_trajectory PandaTrajectoryNode
 ```
 
- Launch the package
 ```
ros2 launch panda_trajectory launch_PandaTrajectoryNode.py
 ```
--- 
#### **Runtime User Interaction**
 - The terminal gives the user Real-Time control over the robot during the runtime
   ```bash
   USER INPUT: 's'=pause | 'd'=resume | 'h'=home:
   ```
`s` → pause trajectory

`d` → continue trajectory

`h` → safely return to the home position and terminate execution.

 - The program also pauses at selected waypoints and prompts the user:
    ```bash
   ✅ Reached waypoint 1....n
   ```

---
#### **Adjustable Parameters**
The robot behavior and trajectory can be customized through the `.yaml` configuration file. The following parameters are adjustable:

- All these paramets can be adjusted in the `.yaml` file

- End-effector tilt angles (degrees) 
---
```
`tilt_sideways_deg = " -10 "` ,        
`tilt_forward_deg = " 7 "`.  
```

Waypoints: 
   - Users can adjust values to move the robot along a different Cartesian path.
   - Ensure the points are reachable and collision-free.
   - The speed of the robot can be modified for each way point.
   - All coordinates are in meters (relative to the robot base frame).
   - The number of waypoint sequence can be increased or decreased below 
```
waypoints:
  - name: HOME
    offset: [0.0, 0.0, 0.0]
    control_rate_hz: 30

  - name: POINT_A
    offset: [0.235, -0.24, -0.37]
    control_rate_hz: 30

  - name: POINT_B
    offset: [0.235, 0.41, -0.37]
    control_rate_hz: 30
```
---
Control loop frequency 
   - This defines how fast the robot control loop runs, in Hertz (cycles per second).
   - Recommended safe range: 10–20 Hz for the Panda robot.

```
motion:
  control_rate_hz: 20
```
---
Number of interpolation steps per segment
   - etermines how finely the trajectory between two waypoints is divided.

   - More steps → smoother motion but longer computation per segment.

   - Fewer steps → faster execution but more “jumpy” movements.

   - Can gradually increase up to 160 for very smooth trajectories.

```
motion:
  steps_per_segment: 160
```
---
Pause Behavior
    - The robot can automatically pause at specific waypoints for user intervention.
```
pause_behavior:
  pause_at_indices: [1, 2, 3, 4]
```

- The robot will stop at these waypoint indices until the user resumes.


---






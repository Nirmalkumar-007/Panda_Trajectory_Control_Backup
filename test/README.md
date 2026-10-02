This folder `test` consists of test files for each classes. 
- *Note: make sure the to run the program from this directory 
```
~/panda_ws/src/panda_trajectory$ pytest -s test/test_PandaTrajectory.py 
```

---

### **test_PandaCmdSub.py**

- This Tested the 'PandaCmdSub' class. The test verifies that the robot follows the defined waypoints with the correct orientations, interacts with the user inputs ('f' to continue, `h` to return home), and correctly calls robot control methods and exit() when the user chooses to return home.
- Mocking was used for `input()`, `exit()`, `time.sleep()`, and JointPosition controller to ensure the test runs safely without actual hardware or program termination.

- **Description:** Simulate user inputs `f`, `f`, `f`, `f`, `h` to move robot in waypoints and return home. Verify robot commands and exit call.

- **Input:** Waypoints, orientations, user inputs

---

### **test_Pandatrajectory.py**

- Tested the 'pandaTrajectoryNode' class. The test verifies that the robot follows the defined waypoints with the correct orientations, interacts with the user inputs ('s' to pause, `d` to continue, `h` to return home), and correctly calls robot control methods and `exit()` when the user chooses to return home.

- **NOTE** 
   - Test specific configuration, for quick execution 
   - changed values in the values.yaml file
         - The speed can be adjusted to 200, step size to 10, and the reduce the waypoints to increse the speed of the test.
     
- **Description:** Simulates the user inputs to move robot in waypoints and return home. Saves the log values for the time,joint_positions,joint_velocities,joint_torques,ee_position,ee_orientation and Verify robot commands and exit call.

- **Inputs:** Waypoints, orientations, speed, steps per segment from the `.Yaml` file.

---

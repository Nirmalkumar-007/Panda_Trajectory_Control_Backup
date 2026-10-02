import csv
import numpy as np
import matplotlib.pyplot as plt

# Path to your separated CSV log
CSV_FILE = "/home/iit-rain/Desktop/CODE REV 2 - TRAP- TILT/CSV_logs/PandaTrajectory_log_split_5.csv"

# 1. LOAD CSV DATA
with open(CSV_FILE, "r") as f:
    reader = csv.reader(f)
    rows = list(reader)

data_rows = rows[2:]     

# 2. EXTRACT ARRAYS
time_log = np.array([float(row[0]) for row in data_rows])
time_rel = time_log - time_log[0]

# Joint Data
joint_positions = np.array([[float(row[i+1]) for i in range(7)] for row in data_rows])
joint_velocities = np.array([[float(row[i+8]) for i in range(7)] for row in data_rows])
joint_torques = np.array([[float(row[i+15]) for i in range(7)] for row in data_rows])

# EE Position & Orientation
ee_positions = np.array([[float(row[22]), float(row[23]), float(row[24])] for row in data_rows])
ee_orientations = np.array([[float(row[25]), float(row[26]), float(row[27]), float(row[28])] for row in data_rows])

# EE Force/Torque (Indices 29-34)
ee_forces = np.array([[float(row[29]), float(row[30]), float(row[31])] for row in data_rows])
ee_ext_torques = np.array([[float(row[32]), float(row[33]), float(row[34])] for row in data_rows])

# ─────────────────────────────────────────────────────────────
# VISUALIZATION (INDIVIDUAL PLOTS)
# ─────────────────────────────────────────────────────────────

# 1. Joint Positions
plt.figure(figsize=(10,6))
for i in range(7):
    plt.plot(time_rel, joint_positions[:,i], label=f"Joint {i+1}")
plt.xlabel("Time [s]"); plt.ylabel("Position [rad]"); plt.title("Joint Positions"); plt.legend(); plt.grid(True)

# 2. Joint Velocities
plt.figure(figsize=(10,6))
for i in range(7):
    plt.plot(time_rel, joint_velocities[:,i], label=f"Joint {i+1}")
plt.xlabel("Time [s]"); plt.ylabel("Velocity [rad/s]"); plt.title("Joint Velocities"); plt.legend(); plt.grid(True)

# 3. Joint Torques
plt.figure(figsize=(10,6))
for i in range(7):
    plt.plot(time_rel, joint_torques[:,i], label=f"Joint {i+1}")
plt.xlabel("Time [s]"); plt.ylabel("Torque [Nm]"); plt.title("Joint Torques"); plt.legend(); plt.grid(True)

# 4. End-Effector Positions
plt.figure(figsize=(8,5))
plt.plot(time_rel, ee_positions[:,0], label="X")
plt.plot(time_rel, ee_positions[:,1], label="Y")
plt.plot(time_rel, ee_positions[:,2], label="Z")
plt.xlabel("Time [s]"); plt.ylabel("EE Position [m]"); plt.title("End-Effector Position"); plt.legend(); plt.grid(True)

# 5. End-Effector Orientation
plt.figure(figsize=(8,5))
plt.plot(time_rel, ee_orientations[:,0], label="qx")
plt.plot(time_rel, ee_orientations[:,1], label="qy")
plt.plot(time_rel, ee_orientations[:,2], label="qz")
plt.plot(time_rel, ee_orientations[:,3], label="qw")
plt.xlabel("Time [s]"); plt.ylabel("Orientation [quat]"); plt.title("End-Effector Orientation"); plt.legend(); plt.grid(True)

# ─────────────────────────────────────────────────────────────
# 6. EE FORCE & TORQUE (SUBPLOTS IN A SINGLE WINDOW)
# ─────────────────────────────────────────────────────────────
fig_wrench, (ax_f, ax_t) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig_wrench.subplots_adjust(hspace=0.3)

# Force Subplot
ax_f.plot(time_rel, ee_forces[:, 0], label="Fx")
ax_f.plot(time_rel, ee_forces[:, 1], label="Fy")
ax_f.plot(time_rel, ee_forces[:, 2], label="Fz")
ax_f.set_ylabel("Force [N]"); ax_f.set_title("EE External Forces"); ax_f.legend(); ax_f.grid(True)

# Torque Subplot
ax_t.plot(time_rel, ee_ext_torques[:, 0], label="Tx")
ax_t.plot(time_rel, ee_ext_torques[:, 1], label="Ty")
ax_t.plot(time_rel, ee_ext_torques[:, 2], label="Tz")
ax_t.set_ylabel("Torque [Nm]"); ax_t.set_title("EE External Torques")
ax_t.set_xlabel("Time [s]"); ax_t.legend(); ax_t.grid(True)




###
# Commanded Position (Indices 35-37)
cmd_positions = np.array([[float(row[35]), float(row[36]), float(row[37])] for row in data_rows])

# s_val (Index 38)
s_vals = np.array([float(row[38]) for row in data_rows])


plt.figure(figsize=(8,5))
plt.plot(time_rel, ee_positions[:,0], label="EE X")
plt.plot(time_rel, cmd_positions[:,0], '--', label="CMD X")

plt.plot(time_rel, ee_positions[:,1], label="EE Y")
plt.plot(time_rel, cmd_positions[:,1], '--', label="CMD Y")

plt.plot(time_rel, ee_positions[:,2], label="EE Z")
plt.plot(time_rel, cmd_positions[:,2], '--', label="CMD Z")

plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("EE Position vs Commanded Position")
plt.legend()
plt.grid(True)

# error = ee_positions - cmd_positions
# error_norm = np.linalg.norm(error, axis=1)

# plt.figure(figsize=(8,5))
# plt.plot(time_rel, error_norm)
# plt.xlabel("Time [s]")
# plt.ylabel("Position Error [m]")
# plt.title("Tracking Error Norm")
# plt.grid(True)

# plt.figure(figsize=(8,5))
# plt.plot(time_rel, s_vals)
# plt.xlabel("Time [s]")
# plt.ylabel("s_val")
# plt.title("Trajectory Progress (s_val)")
# plt.grid(True)
###
plt.show()
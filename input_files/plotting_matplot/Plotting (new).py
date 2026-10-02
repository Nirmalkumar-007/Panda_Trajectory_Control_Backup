import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.signal import savgol_filter

# ── Config ────────────────────────────────────────────────────────────────────
CSV_FILE = (
    sys.argv[1] if len(sys.argv) > 1
    else "/home/iit-rain/Desktop/CODE REV 2 - TRAP- TILT/CSV_logs/PandaTrajectory_log_split_6.csv"
)

# ── Load CSV ──────────────────────────────────────────────────────────────────
with open(CSV_FILE, "r") as f:
    all_rows = list(csv.reader(f))

group_labels = all_rows[0]
col_names    = all_rows[1]
data_rows    = all_rows[2:]

unique_cols = []
for g, c in zip(group_labels, col_names):
    name = c.strip() if c.strip() else g.strip()
    unique_cols.append(name)

n_cols = len(unique_cols)
data   = np.full((len(data_rows), n_cols), np.nan)
for r, row in enumerate(data_rows):
    for c in range(min(len(row), n_cols)):
        try:
            data[r, c] = float(row[c])
        except ValueError:
            pass

data = data[~np.all(np.isnan(data), axis=1)]

# ── Column lookup — exact-first prevents q1 matching dq1/tau1 ────────────────
def get_col(candidates):
    for c in candidates:
        cl = c.lower()
        for i, name in enumerate(unique_cols):
            if name.lower() == cl: return data[:, i]
        for i, name in enumerate(unique_cols):
            if name.lower().endswith(cl): return data[:, i]
        for i, name in enumerate(unique_cols):
            if cl in name.lower(): return data[:, i]
    raise KeyError(f"No column matching {candidates}. Available: {unique_cols}")

# ── Signals ───────────────────────────────────────────────────────────────────
t_abs = get_col(["time"])
t     = t_abs - t_abs[0]
ee_x  = get_col(["ee_x"])
ee_y  = get_col(["ee_y"])
ee_z  = get_col(["ee_z"])
q     = np.column_stack([get_col([f"q{i+1}"])   for i in range(7)])
tau   = np.column_stack([get_col([f"tau{i+1}"]) for i in range(7)])

has_cmd = any("cmd_x" in n.lower() for n in unique_cols)
if has_cmd:
    cmd_x = get_col(["cmd_x"])
    cmd_y = get_col(["cmd_y"])
    cmd_z = get_col(["cmd_z"])
    print("cmd_position found — lag overlay enabled.")
else:
    cmd_x = cmd_y = cmd_z = None
    print("No cmd_position — run fixed trajectory code to enable lag overlay.")

# ── Path length ───────────────────────────────────────────────────────────────
def path_length(x, y, z):
    d = np.sqrt(np.diff(x, prepend=x[0])**2 +
                np.diff(y, prepend=y[0])**2 +
                np.diff(z, prepend=z[0])**2)
    return np.cumsum(d)

path     = path_length(ee_x, ee_y, ee_z)
path_cmd = path_length(cmd_x, cmd_y, cmd_z) if has_cmd else None

# ── SG velocity / acceleration ────────────────────────────────────────────────
dt_med    = float(np.median(np.diff(t)))
win       = max(int(round(0.15 / dt_med)) | 1, 5)
polyorder = min(3, win - 1)

vel     = savgol_filter(path, win, polyorder, deriv=1, delta=dt_med)
acc     = savgol_filter(path, win, polyorder, deriv=2, delta=dt_med)
vel_cmd = (savgol_filter(path_cmd, win, polyorder, deriv=1, delta=dt_med)
           if has_cmd else None)

# ── Segment boundaries ────────────────────────────────────────────────────────
at_rest = vel < 1e-3
starts  = np.where(np.diff(at_rest.astype(int)) < 0)[0]
ends    = np.where(np.diff(at_rest.astype(int)) > 0)[0]


def vlines(ax):
    for i in starts: ax.axvline(t[i], color="#aaa", lw=0.8, ls="--", alpha=0.6)
    for i in ends:   ax.axvline(t[i], color="#ccc", lw=0.5, ls=":",  alpha=0.5)


# Figure 2 — 3D trajectory with EQUAL ASPECT RATIO ─────────────────────────

fig2 = plt.figure(figsize=(10, 8))
ax3d = fig2.add_subplot(111, projection="3d")

pts  = np.column_stack([ee_x, ee_y, ee_z])
segs = np.stack([pts[:-1], pts[1:]], axis=1)
lc   = Line3DCollection(segs, colors=plt.cm.viridis(np.linspace(0, 1, len(segs))), linewidth=1.5)
ax3d.add_collection(lc)

# Equal-aspect padding
x_mid, y_mid, z_mid = (ee_x.max()+ee_x.min())/2, (ee_y.max()+ee_y.min())/2, (ee_z.max()+ee_z.min())/2
half = max(ee_x.max()-ee_x.min(), ee_y.max()-ee_y.min(), ee_z.max()-ee_z.min(), 0.05) * 0.6
ax3d.set_xlim(x_mid - half, x_mid + half)
ax3d.set_ylim(y_mid - half, y_mid + half)
ax3d.set_zlim(z_mid - half, z_mid + half)

ax3d.scatter(*pts[0],  color="#1D9E75", s=100, depthshade=False, label="Start", zorder=5)
ax3d.scatter(*pts[-1], color="#E24B4A", s=100, depthshade=False, label="End",   zorder=5)
for idx in ends:
    ax3d.scatter(ee_x[idx], ee_y[idx], ee_z[idx], color="#EF9F27", s=40, depthshade=False, zorder=4)
if has_cmd:
    ax3d.plot(cmd_x, cmd_y, cmd_z, "#EF9F27", lw=1.0, ls="--", alpha=0.7, label="Commanded path")

sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, norm=mcolors.Normalize(t[0], t[-1]))
sm.set_array([])
fig2.colorbar(sm, ax=ax3d, shrink=0.5, pad=0.1, label="Time [s]")
ax3d.set_xlabel("X [m]"); ax3d.set_ylabel("Y [m]"); ax3d.set_zlabel("Z [m]")
ax3d.set_title("3D End-Effector Trajectory  (equal-aspect axes)")
ax3d.legend(fontsize=9); ax3d.grid(True)
# View chosen for this trajectory geometry: X-Z drop first, then Y sweep
ax3d.view_init(elev=25, azim=200)


# # ── Figure 1 — Triple diagnostic ─────────────────────────────────────────────
# fig1, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
# fig1.suptitle("Panda Trajectory — Diagnostic", fontsize=13, fontweight="bold")
# fig1.subplots_adjust(hspace=0.38)
# for ax in axes: vlines(ax)

# axes[0].plot(t, path, "#185FA5", lw=1.8, label="Actual path length")
# if has_cmd: axes[0].plot(t, path_cmd, "#EF9F27", lw=1.0, ls="--", alpha=0.85, label="Commanded")
# axes[0].set_ylabel("Distance [m]"); axes[0].set_title("1. Cumulative path length")
# axes[0].legend(fontsize=9); axes[0].grid(True, ls="--", alpha=0.4)

# axes[1].plot(t, np.gradient(path, t), "#E24B4A", lw=0.4, alpha=0.2, label="Raw dS/dt (noise ref.)")
# axes[1].plot(t, vel, "#E24B4A", lw=1.8, label="Velocity (SG-filtered)")
# if has_cmd: axes[1].plot(t, vel_cmd, "#EF9F27", lw=1.0, ls="--", alpha=0.85, label="Commanded vel.")
# axes[1].set_ylabel("Velocity [m/s]"); axes[1].set_ylim(bottom=0)
# axes[1].set_title("2. EE velocity — trapezoidal profile")
# axes[1].legend(fontsize=9); axes[1].grid(True, ls="--", alpha=0.4)

# axes[2].plot(t, acc, "#1D9E75", lw=1.4, label="Acceleration")
# axes[2].axhline(0, color="black", lw=0.5)
# pk = int(np.argmax(np.abs(acc)))
# axes[2].annotate(f"peak {acc[pk]:.4f} m/s²", xy=(t[pk], acc[pk]),
#                  xytext=(t[pk] + max(t)*0.03, acc[pk]*0.75), fontsize=8, color="#0F6E56",
#                  arrowprops=dict(arrowstyle="->", color="#0F6E56", lw=0.8))
# axes[2].set_ylabel("Accel [m/s²]"); axes[2].set_xlabel("Time [s]")
# axes[2].set_title("3. Acceleration — ±small during ramps, ~0 during cruise")
# axes[2].legend(fontsize=9); axes[2].grid(True, ls="--", alpha=0.4)
# fig1.tight_layout()

# #
# # ── Figure 3 — Joint torques ──────────────────────────────────────────────────
# fig3, ax_tau = plt.subplots(figsize=(12, 5))
# for i in range(7):
#     ax_tau.plot(t, tau[:, i], lw=1.2, label=f"Joint {i+1}")
# vlines(ax_tau)
# ax_tau.set_xlabel("Time [s]"); ax_tau.set_ylabel("Torque [Nm]")
# ax_tau.set_title("Joint Torques vs Time")
# ax_tau.legend(fontsize=9, ncol=4); ax_tau.grid(True, ls="--", alpha=0.4)
# fig3.tight_layout()

# ── Figure 4 — XYZ vs time, separate subplots, shaded lag ────────────────────
# fig4, axes4 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
# fig4.suptitle("EE Position vs Time  —  solid=actual  dashed=commanded  shaded=lag", fontsize=11)
# fig4.subplots_adjust(hspace=0.38)

# xyz_data   = [ee_x,  ee_y,  ee_z]
# xyz_cmds   = [cmd_x, cmd_y, cmd_z] if has_cmd else [None]*3
# xyz_labels = ["X", "Y", "Z"]
# xyz_colors = ["#185FA5", "#D85A30", "#1D9E75"]
# xyz_titles = [
#     "X position — constant after HOME→A1",
#     "Y position — main scanning axis (A1 ↔ A2)",
#     "Z position — drops during HOME→A1, then held",
# ]

# for ax, d, cmd, lbl, clr, title in zip(axes4, xyz_data, xyz_cmds, xyz_labels, xyz_colors, xyz_titles):
#     ax.plot(t, d, color=clr, lw=1.4, label=f"{lbl} actual")
#     if cmd is not None:
#         ax.plot(t, cmd, color=clr, lw=0.9, ls="--", alpha=0.7, label=f"{lbl} commanded")
#         ax.fill_between(t, d, cmd, alpha=0.12, color=clr, label="lag")
#     vlines(ax)
#     ax.set_ylabel(f"{lbl} [m]"); ax.set_title(title)
#     ax.legend(fontsize=9, loc="upper right"); ax.grid(True, ls="--", alpha=0.4)

# axes4[-1].set_xlabel("Time [s]")
# fig4.tight_layout()

plt.show()
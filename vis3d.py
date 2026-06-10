"""
3D flight visualization — camera follows the drone.

Sim data is in NED (z down, +y right). Visualization uses ENU (z up, +y left)
because matplotlib's 3D viewer is more intuitive that way. Conversion happens
once at the data-read step; the rotation matrix below is the standard R_z·R_y·R_x.
"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ARM = 0.141

# Body-frame motor positions (X-frame).
# Convention: +x forward (nose), +y left, +z up.
# Indices below match MOTOR_LABELS.
BODY_POINTS = np.array([
    [+ARM, -ARM, 0],      # 0: front-right → M1
    [-ARM, +ARM, 0],      # 1: rear-left   → M2
    [+ARM, +ARM, 0],      # 2: front-left  → M3
    [-ARM, -ARM, 0],      # 3: rear-right  → M4
    [0,     0,    0],     # 4: center
    [ARM*1.4, 0, 0],      # 5: nose marker
])

MOTOR_LABELS = ["M1", "M2", "M3", "M4"]


def rot_matrix(roll, pitch, yaw):
    """Standard R_z(yaw) · R_y(pitch) · R_x(roll) for ENU body→world."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,             cr*cp],
    ])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "flight_log.csv"
    df = pd.read_csv(path)

    # NED --> ENU conversion (once, at the source).
    # Position: x stays, y flips (NED y-right --> ENU y-left), z flips.
    # Angles:   negate all three to match the flipped frame.
    pos = np.column_stack([
        df["x"].values,
        -df["y"].values,
        -df["z"].values,
    ])
    roll  = df["roll"].values
    pitch = -df["pitch"].values
    yaw   = -df["yaw"].values
    t     = df["t"].values

    WIN_XY = 2.2
    WIN_Z  = 1.6

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Quadcopter 3D Flight")
    ax.view_init(elev=22, azim=-55)
    ax.grid(True, alpha=0.25)

    (trail,) = ax.plot([], [], [], "b-", lw=1.4, alpha=0.55, label="Trajectory")
    (dot,)   = ax.plot([], [], [], "ro", ms=7.5, label="Drone")

    # X-frame: two diagonal arms.
    (arm1,) = ax.plot([], [], [], "k-", lw=3.2, alpha=0.95)   # M1 ↔ M2
    (arm2,) = ax.plot([], [], [], "k-", lw=3.2, alpha=0.95)   # M3 ↔ M4

    (nose,) = ax.plot([], [], [], "r^", ms=11, label="Nose (forward)")

    motor_texts = [
        ax.text(0, 0, 0, "", color="darkgreen", fontsize=10, fontweight="bold",
                ha="center", va="bottom")
        for _ in range(4)
    ]

    ax.legend(loc="upper left")
    time_text  = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=11)
    coord_text = ax.text2D(0.02, 0.90, "", transform=ax.transAxes, fontsize=9)

    skip = max(1, len(df) // 500)

    def init():
        for artist in (trail, dot, arm1, arm2, nose):
            artist.set_data([], [])
            artist.set_3d_properties([])
        for txt in motor_texts:
            txt.set_text("")
        time_text.set_text("")
        coord_text.set_text("")
        return [trail, dot, arm1, arm2, nose, *motor_texts, time_text, coord_text]

    def update(frame):
        idx = min(frame * skip, len(df) - 1)
        p = pos[idx]

        # Camera follow.
        ax.set_xlim(p[0] - WIN_XY, p[0] + WIN_XY)
        ax.set_ylim(p[1] - WIN_XY, p[1] + WIN_XY)
        ax.set_zlim(p[2] - WIN_Z,  p[2] + WIN_Z)

        trail.set_data(pos[:idx, 0], pos[:idx, 1])
        trail.set_3d_properties(pos[:idx, 2])

        dot.set_data([p[0]], [p[1]])
        dot.set_3d_properties([p[2]])

        R = rot_matrix(roll[idx], pitch[idx], yaw[idx])
        body = (R @ BODY_POINTS.T).T + p

        # X-frame diagonals: M1↔M2 and M3↔M4.
        arm1.set_data([body[0, 0], body[1, 0]], [body[0, 1], body[1, 1]])
        arm1.set_3d_properties([body[0, 2], body[1, 2]])

        arm2.set_data([body[2, 0], body[3, 0]], [body[2, 1], body[3, 1]])
        arm2.set_3d_properties([body[2, 2], body[3, 2]])

        # Nose marker.
        n = body[5]
        nose.set_data([n[0]], [n[1]])
        nose.set_3d_properties([n[2]])

        # Motor labels, floating slightly above each motor.
        for i in range(4):
            motor_texts[i].set(x=body[i, 0], y=body[i, 1], z=body[i, 2] + 0.18)
            motor_texts[i].set_text(MOTOR_LABELS[i])

        time_text.set_text(f"t = {t[idx]:.2f} s")
        coord_text.set_text(f"x={p[0]:.2f}   y={p[1]:.2f}   alt={p[2]:.2f} m")

        return [trail, dot, arm1, arm2, nose, *motor_texts, time_text, coord_text]

    ani = FuncAnimation(fig, update, frames=len(df)//skip,
                        init_func=init, interval=35, blit=False, repeat=True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()


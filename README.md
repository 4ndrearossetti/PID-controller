# Quadcopter PID Flight Controller Simulation

A C simulation of a quadcopter with a well-tuned **Altitude PID controller** and live visualization.

More details on my website: <a href="https://andrearossetti.me/projects/pid-controller" target="_blank">andrearossetti.me</a>

## Features

- Realistic 6-DOF physics engine (NED frame: X forward, Y left, Z down)
- Single-axis PID controller with conditional-integration anti-windup
- Standard X-frame mixer (control commands → 4 motor thrusts)
- Cascaded control architecture (inner rate loop, outer angle loop)
- Wind gust disturbance testing (simulates combat/explosion hits)
- Modular C codebase with separate tests per module
- 3D visualisation with matplotlib3D

## Project Structure

```
.
├── src/
│   ├── main.c          # Entry point (STALE — old altitude demo, not cascade)
│   ├── quad.h / quad.c # 6-DOF quadcopter physics (NED frame)
│   ├── pid.h / pid.c   # Generic PID controller with anti-windup
│   └── mixer.h / mixer.c  # X-frame mixer (thrust/roll/pitch/yaw → motors)
├── tests/
│   ├── test_quad.c     # Quad physics verification (hover, torque, accel)
│   ├── test_pid.c      # PID unit tests (P/I/D, anti-windup, reset)
│   ├── test_mixer.c    # Mixer tests (each axis, saturation)
│   └── test_helpers.h  # ASSERT_NEAR, ASSERT_TRUE, TEST_PASS macros
├── vis3d.py            # Live Matplotlib visualization
├── run.sh              # Launcher
├── Makefile            # Build + test targets
├── flight_log.csv      # Generated data (live)
└── README.md
```

## How to Run

```bash
# 1. Build
make

# 2. Run simulation + 3D visualisation
./run.sh
```

The graph will open automatically after the simulation is done.

## Tech Stack

- **C** — Core simulation and PID logic
- **Python + Matplotlib** — Live visualization
- **CSV logging** — Real-time data exchange

## Demo

The flight scenario is 20 seconds, with these phases:

| Time | Command | What the drone does |
|------|---------|---------------------|
| 0–1 s | nothing | Sits on the ground at altitude 0, all angles zero |
| 1–4 s | altitude = 1 m | Climbs straight up to 1 m, settles |
| 4–8 s | roll = 0.15 rad (~8.6°) | Banks right while holding altitude |
| 8–12 s | pitch = 0.15 rad (~8.6°) | Pitches nose up *while still banked*, now tilted in both axes |
| 12–16 s | yaw_rate = 0.5 rad/s (~29°/s) | Rotates CW continuously for 4 seconds (~2 full turns), while still banked and pitched — a coordinated turn |
| 16–20 s | all commands back to zero | Returns to level hover at 1 m and stays there |

![](demo.gif)


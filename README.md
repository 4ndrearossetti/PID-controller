# Quadcopter PID Flight Controller Simulation

A C simulation of a quadcopter with a well-tuned **Altitude PID controller** and live visualization.

More details on my website: <a href="https://andrearossetti.me/projects/pid-controller" target="_blank">andrearossetti.me</a>

## Features

- Realistic 6-DOF physics engine (NED frame: X forward, Y left, Z down)
- Single-axis PID controller with conditional-integration anti-windup
- Standard X-frame mixer (control commands → 4 motor thrusts)
- Cascaded control architecture (inner rate loop, outer angle loop)
- Wind gust disturbance testing (simulates combat/explosion hits)
- Live plotting with Matplotlib (altitude, velocity, thrust)
- Modular C codebase with separate tests per module

## Current Performance (250g FPV Drone, altitude-only)

- **Rise time (0 → 90%)**: ~0.60 s
- **Overshoot**: ~4%
- **Settling time (±0.05 m)**: ~1.4 s
- **Final accuracy**: ±0.0001 m
- Survives **100 ms strong wind gust** and returns to setpoint cleanly

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
├── live_plot.py        # Live Matplotlib visualization
├── run.sh              # Launcher
├── Makefile            # Build + test targets
├── flight_log.csv      # Generated data (live)
└── README.md
```

## How to Run

```bash
# 1. Build
make

# 2. Run simulation + live plot
./run.sh
```

The graph will open automatically and update in real time.
Press any key after the simulation ends to close the plot window.

## Tech Stack

- **C** — Core simulation and PID logic
- **Python + Matplotlib** — Live visualization
- **CSV logging** — Real-time data exchange

## Next Steps (Planned)

- **cascaded angle→rate PID** (control.h/control.c) — inner rate loop at 200 Hz, outer angle loop at 50–100 Hz, per-axis (roll/pitch) plus yaw rate
- **Complete simulation** — wire PID → mixer → quad dynamics for closed-loop attitude hold, then position hold
- **3D visualization / GUI** — better debug than CSV+Matplotlib
- **Disturbance tests** — lateral wind, payload drop, aggressive flips
- **Sensor integration** — complementary filter (MPU-6050), then EKF
- **ESP32 target** — move from sim to hardware

## Demo Video

![](demo.gif)


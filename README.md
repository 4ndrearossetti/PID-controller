# Quadcopter PID Flight Controller Simulation

A C simulation of a quadcopter with a well-tuned **Altitude PID controller** and live visualization.

More details on my website: <a href="https://andrearossetti.me/projects/pid-controller" target="_blank">andrearossetti.me</a>

## Features

- Realistic physics engine (position, velocity, gravity)
- Robust Altitude PID controller with anti-windup
- Wind gust disturbance testing (simulates combat/explosion hits)
- Live plotting with Matplotlib (altitude, velocity, thrust)
- One-command launch (`./run.sh`)

## Current Performance (250g FPV Drone, altitude-only)

- **Rise time (0 → 90%)**: ~0.60 s
- **Overshoot**: ~4%
- **Settling time (±0.05 m)**: ~1.4 s
- **Final accuracy**: ±0.0001 m
- Survives **100 ms strong wind gust** and returns to setpoint cleanly

## Project Structure

```
.
├── main.c              # Simulation + physics + PID
├── pid.h / pid.c       # Generic PID controller
├── live_plot.py        # Live Matplotlib visualization
├── run.sh          # One-click launcher
├── flight_log.csv      # Generated data (live)
└── Makefile
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

- Roll / Pitch / Yaw attitude control (cascaded PIDs)
- Full 4-motor mixing
- 3D visualization / GUI
- More disturbance tests (lateral wind, payload drop, etc.)

## Demo Video

![](demo.gif)


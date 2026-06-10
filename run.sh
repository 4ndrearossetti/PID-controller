#!/bin/bash

echo "=== Drone Simulation + 3D Visualization ==="

rm -f flight_log.csv
source venv/bin/activate

make run

echo ""
echo "=== Simulation complete — launching 3D visualisation ==="
python3 vis3d.py


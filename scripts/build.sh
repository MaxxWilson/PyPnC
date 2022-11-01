#!/bin/bash
# Build PnC Ros
colcon build --base-paths pnc_ros

# Build TOWR
cd build
cmake ..
make -j
source ~/.bashrc
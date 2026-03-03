#!/bin/bash
source /opt/ros/humble/setup.bash
export LOG_LEVEL=1
./build/sender_node config.json

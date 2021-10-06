#!/bin/bash

if [ ! -v $VEHICLE_MAVLINK_SYSID]; then
    export VEHICLE_MAVLINK_SYSID=$VEHICLE_MAVLINK_SYSID
    echo "VEHICLE_MAVLINK_SYSID setting to $VEHICLE_MAVLINK_SYSID"
else
    export VEHICLE_MAVLINK_SYSID=1
    echo "VEHICLE_MAVLINK_SYSID not set, default to 1"
fi

ros2 launch simple_offboard starling_simple_offboard.launch.xml
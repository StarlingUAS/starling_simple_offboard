# Starling Simple Offboard

This project provides a simple offboard controller for drones running mavros in ROS2. This began as a port of the simple_offboard controller written by CopterExpress for use with the [Clover Drone Platform](https://github.com/CopterExpress/clover).

This package contains both the controller `simple_offboard` node as well as the srvs seperately in `simple_offboard_msgs` for development of controllers which use this node.

## *simple_offboard* Node
This node is intended to simplify the programming of autonomous drone flight (`OFFBOARD` flight mode). It allows the setting of desired flight tasks and automatically transforms coordinates between frames. It is a high level system for interacting with the flight controller.

This is a direct port of the `simple_offboard` module from the clover project. See the following documentation from [Coex](https://clover.coex.tech/en/simple_offboard.html).

The node advertises the following set of services:

- `get_telemetry` (srv/GetTelemetry) - Returns telemetry
- `navigate` (srv/Navigate) - Navigate to a position relative to a frame of reference (e.g. map or body)
- `navigate_global` (srv/NavigateGlobal) - Navigate to a postiion lat long relative to frame of reference
- `set_position` (srv/SetPosition) - Set the setpoint for position and yaw for continuous flow of target points
- `set_velocity` (srv/SetVelocity) - Set speed and yaw setpoints
- `set_attitude` (srv/SetAttidue) - Set pitch, roll, yaw and throttle levels for lower level control
- `set_rates` (srv/SetRates) - Set pitch, roll, yaw rates and throttle levels for lower level control
- `land` (std_srvs/srv/Trigger) - Switch drone to landing mode.


## License

As per the original Clover source code, this ROS2 port is also covered under the MIT License.
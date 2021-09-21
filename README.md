# Starling Simple Offboard

This project provides a simple offboard controller for drones running mavros in ROS2. This began as a port of the simple_offboard controller node written by CopterExpress for use with the [Clover Drone Platform](https://github.com/CopterExpress/clover).

This package contains both the controller `simple_offboard` node as well as the srvs seperately in `simple_offboard_msgs` for development of controllers which use this node.

## *simple_offboard* Node
This node is intended to simplify the programming of autonomous drone flight (`OFFBOARD` flight mode). It allows the setting of desired flight tasks and automatically transforms coordinates between frames. It is a high level system for interacting with the flight controller.

This is a direct port of the `simple_offboard` module from the clover project. See the following documentation from [Coex](https://clover.coex.tech/en/simple_offboard.html).

The node advertises the following set of services:

- `get_telemetry` (srv/GetTelemetry) - Returns telemetry
- `navigate` (srv/Navigate) - Navigate to a position relative to a frame of reference (e.g. map or body). *new* can also be a blocking call
- `navigate_global` (srv/NavigateGlobal) - Navigate to a postiion lat long relative to frame of reference
- `set_position` (srv/SetPosition) - Set the setpoint for position and yaw for continuous flow of target points
- `set_velocity` (srv/SetVelocity) - Set speed and yaw setpoints
- `set_attitude` (srv/SetAttidue) - Set pitch, roll, yaw and throttle levels for lower level control
- `set_rates` (srv/SetRates) - Set pitch, roll, yaw rates and throttle levels for lower level control
- `takeoff` (srv/Takeoff) - Takeoff to a paritcular height at a particular speed, can be a blocking call.
- `land` (std_srvs/srv/Trigger) - Switch drone to landing mode.
- `hold` (std_srvs/srv/Trigger) - Switch drone to hold mode.

The node also listens on `/emergency_stop` (`std_msgs/msg/Empty`) and will send the `px4` estop code to `mavros/cmd/command` cutting off all motors.

## *trajectory_handler* Node
This node is intended to simplify the execution of simple trajectory following based tasks.

It uses the [**libInterpolate**](https://github.com/CD3/libInterpolate) library for interpolation between trajectory points.

The node advertises the following service:

- `submit_trajectory` (srv/SubmitTrajectory) - Submits a position, velocity, attitude or rate trajectory to be executed.

```
# Submitted Trajectory
trajectory_msgs/JointTrajectory trajectory
# Trajectory type (i.e. position or attitude)
string type
# Interpolation method from https://github.com/CD3/libInterpolate (i.e. linear, cubic or monotonic), defaults to cubic
string interpolation_method
# Frame of the trajectory, defaults to map frame
string frame_id
# Whether drone should auto_arm, defaults to false
bool auto_arm
# Whether drone should wait for mission start, defaults to false, can be set to false if doing velocity, attitude or rates control
bool do_not_wait_for_mission_start
# Whether drone should takeoff first (for velocity, attitude or rates control)
bool auto_takeoff
float32 takeoff_height
---
string message
bool success
```

### Trajectory msg format

A `trajectory` is comprised of a [JointTrajectory](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html) containing a list of [JointTrajectoryPoints](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html).

In order to determine whether the incoming trajectory is one of position, velocity, attitude or rates, we parse the following format based on the `type` given in the SubmitTrajectory and whether the `JointTrajectoryPoint` msgs populate the `position` or `velocity` field.

| Control Type 	| `type`   	| `JointTrajectoryPoint` field 	| index 0    	| index 1   	| index 2  	| index 3           	| index 4             	|
|--------------	|----------	|------------------------------	|------------	|-----------	|----------	|-------------------	|---------------------	|
| position     	| position 	| position                     	| x          	| y         	| z        	| yaw (optional)    	| yaw rate (optional) 	|
| velocity     	| position 	| velocity                     	| vx         	| vy        	| vz       	| yaw (optional)    	| yaw rate (optional  	|
| attitude     	| attitude 	| position                     	| pitch      	| roll      	| yaw      	| thrust (optional) 	|                     	|
| rates        	| attitude 	| velocity                     	| pitch rate 	| roll rate 	| yaw rate 	| thrust (optional) 	|                     	|

The first `JointTrajectoryPoint` in `trajectory` is used to determine the type of the trajectory. The time to execute each trajectory point is encoded as `time_from_start` field of `JointTrajectoryPoint`.

> **Note**: An error will be returned if the `trajectory` contains no points.

> **Note**: The `position` field of `JointTrajectoryPoint` is checked first. If it is populated by at least 3 values, it will assume they are valid points, otherwise it will check the `velocity` field. If neither is populated an error will be returned.

### Execution of the trajectory.

Once a request has been received, it is first checked for validity and processed into memory ready to be executed. Then execution differs depending on trajectory type:

#### Position Trajectory

Unless `do_not_wait_for_mission_start` is set, the executor will pause until it has received a mission start signal on `\mission_start` topic (std_msgs/Empty.msg). This can be done manually on the command line or through the [Starling UI](https://github.com/mhl787156/starling_ui_dashly)

If `auto_arm` is enabled, the vehicle will automatically arm itself, and switch into `OFFBOARD` mode. It will then proceed to navigate to the first trajectory point given.

> **Note**: if the first position has a z value of that is too close to ground level (i.e. < 0.2m ) the following will fail. If this is an issue, it is suggested to have a special initial trajectory point for safe takeoff, or to set `auto_takeoff` to false. However note that takeoff does take a large amount of time, and to include that into the trajectory times if `auto_takeoff` is set to false.

Once the takeoff is complete, the service will return to the caller. The trajectory controller will begin executing the trajectory using the interpolation method chosen.

**The trajectory can be cancelled by sending a message on `\mission_abort` or `\emergency_stop`**

Once the time elapsed matches the `time_from_start` of the final trajectory point, the vehicle will land whever it has gotten to.

> **Note**: If the `time_from_start` are too short, then it is possible for the drone to never reach any of the points. It is recommended to give the vehicle a little more time to travel from one point to another.

#### Velocity, Attitude and Rates Trajectory

Unless `do_not_wait_for_mission_start` is set, the executor will pause until it has received a mission start signal on `\mission_start` topic (std_msgs/Empty.msg). If the trajectory is designed to be executed mid-air, this should be set to true to ignore the mission start.

If `auto_arm` is enabled, the vehicle will automatically arm itself, and switch into `OFFBOARD` mode (or stay in the same mode if already armed and offboard).

If `auto_takeoff` is enabled, the vehicle will first takeoff to `takeoff_height` height off of the ground before executing the trajectory. If it has not taken off, then the trajectory will need to encompass takeoff.

Once the takeoff is complete, the service will return to the caller. The trajectory controller will begin executing the trajectory using the interpolation method chosen.

## License

As per the original Clover source code, this ROS2 port is also covered under the MIT License.
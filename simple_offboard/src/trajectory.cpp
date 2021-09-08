/*
 * Trajectory follower
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>

#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "simple_offboard_msgs/srv/set_position.hpp"
#include "simple_offboard_msgs/srv/set_velocity.hpp"
#include "simple_offboard_msgs/srv/set_attitude.hpp"
#include "simple_offboard_msgs/srv/set_rates.hpp"
#include "simple_offboard_msgs/srv/submit_trajectory.hpp"
#include "simple_offboard_msgs/srv/get_telemetry.hpp"
#include "simple_offboard_msgs/srv/navigate.hpp"

#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>


class TrajectoryHandler : public rclcpp::Node
{
    public:
        TrajectoryHandler();
        void submitTrajectory(std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Request> req, std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Response> res);

    private:
        void reset();
        void updateTrajectory(const rclcpp::Time& stamp);
        void resetExecutionTimer();
        bool waitForMissionStart();
        void landVehicle();
        void gotoTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point);
        bool checkAbort();

        // Mission parameters
        bool mission_started;
        bool mission_aborted;

        // Execution parameters
        std::atomic<bool> executing_trajectory;
        rclcpp::Time start_time;
        double max_time_sec;
        rclcpp::TimerBase::SharedPtr execution_timer;

        // Interpolators
        std::vector<double> times;
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        _1D::AnyInterpolator<double> interp_x;
        _1D::AnyInterpolator<double> interp_y;
        _1D::AnyInterpolator<double> interp_z;

        // External parameters
        double execution_frequency;
        double end_extra_time; // Extra time to give after finishing trajectory
        double get_to_first_point_timeout;

        // Subscriptions
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               mission_start_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               mission_abort_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               estop_sub;

        // Clients
        rclcpp::Client<simple_offboard_msgs::srv::SetPosition>::SharedPtr        sp_client;
        rclcpp::Client<simple_offboard_msgs::srv::SetVelocity>::SharedPtr       sv_client;
        rclcpp::Client<simple_offboard_msgs::srv::SetAttitude>::SharedPtr       sa_client;
        rclcpp::Client<simple_offboard_msgs::srv::SetRates>::SharedPtr          sr_client;
        rclcpp::Client<simple_offboard_msgs::srv::Navigate>::SharedPtr          navigate_client;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                       land_client;

        // Service
        rclcpp::Service<simple_offboard_msgs::srv::SubmitTrajectory>::SharedPtr      traj_serv;

        // Multithreaded callback groups
        rclcpp::CallbackGroup::SharedPtr callback_group_timers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_clients_;
};

TrajectoryHandler::TrajectoryHandler() :
	Node("trajectory_handler",
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
    this->callback_group_subscribers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_timers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_clients_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    // Get Parameters
    this->get_parameter_or("update_frequency", this->execution_frequency, 10.0); // hz
    this->get_parameter_or("end_extra_time", this->end_extra_time, 5.0); // seconds
    this->get_parameter_or("get_to_first_point_timeout", this->get_to_first_point_timeout, 120.0); // seconds

    // Safety Subscribers
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = this->callback_group_subscribers_;
    this->mission_start_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/mission_start", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; this->mission_started = true;}, sub_opt);
    this->mission_abort_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/mission_abort", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; this->mission_aborted = true;}, sub_opt);
    this->estop_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/emergency_stop", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; this->mission_aborted = true;}, sub_opt);

    // Initialise Service Clients
    this->sp_client = this->create_client<simple_offboard_msgs::srv::SetPosition>("set_position");
    this->sv_client = this->create_client<simple_offboard_msgs::srv::SetVelocity>("set_velocity");
    this->sa_client = this->create_client<simple_offboard_msgs::srv::SetAttitude>("set_attitude");
    this->sr_client = this->create_client<simple_offboard_msgs::srv::SetRates>("set_rates");
    this->navigate_client = this->create_client<simple_offboard_msgs::srv::Navigate>("navigate", rmw_qos_profile_services_default, this->callback_group_clients_);
    this->land_client = this->create_client<std_srvs::srv::Trigger>("land");

    // Initialise Trajectory Service
    this->traj_serv = this->create_service<simple_offboard_msgs::srv::SubmitTrajectory>("submit_trajectory",
        std::bind(&TrajectoryHandler::submitTrajectory, this, std::placeholders::_1, std::placeholders::_2));

    this->reset();
}

void TrajectoryHandler::reset(){
    this->mission_started = false;
    this->mission_aborted = false;
    this->executing_trajectory = false;

    this->start_time = this->now();

    this->times.clear();
    this->xs.clear();
    this->ys.clear();
    this->zs.clear();

    if (this->execution_timer){
        this->execution_timer->cancel();
    }
    this->execution_timer = nullptr;
    RCLCPP_INFO(this->get_logger(), "Reset Internal Parameters and Trajectory Executor");
}

void TrajectoryHandler::resetExecutionTimer() {
    if (this->execution_timer) {
        this->execution_timer->cancel();
    }
    auto execution_rate = std::chrono::duration<double>(1.0/this->execution_frequency);
    this->execution_timer = this->create_wall_timer(execution_rate, [this](){this->updateTrajectory(this->now());});
    RCLCPP_DEBUG(this->get_logger(), "Reset Trajectory Execution timer");
}

void TrajectoryHandler::submitTrajectory(std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Request> req, std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Trajectory Execution Request");

    if(req->frame_id == "") {
        req->frame_id = "map";
    }

    try {
        // Check if already executing a trajectory
        if (this->executing_trajectory)
            throw std::runtime_error("Existing Trajectory Mission in Progress");

        // Check trajectory is populated
        if (req->trajectory.points.size() == 0)
            throw std::runtime_error("Trajectory does not contain any points, no execution");

    } catch (const std::exception& e) {
		res->message = e.what();
        res->success = false;
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		this->reset();
		return;
	}

    // Reset parameters
    this->reset();

    // Set parameters
    this->executing_trajectory = true;

    // Setup data
    this->max_time_sec = 0.0;
    for (auto tjp: req->trajectory.points) {
        double time = rclcpp::Duration(tjp.time_from_start).seconds();
        this->times.push_back(time);
        if (time > this->max_time_sec) {this->max_time_sec = time;}
        this->xs.push_back(tjp.positions[0]);
        this->ys.push_back(tjp.positions[1]);
        this->zs.push_back(tjp.positions[2]);
    }

    // Set Interpolators (https://github.com/CD3/libInterpolate)
    this->interp_x = _1D::CubicSplineInterpolator<double>();
    this->interp_y = _1D::CubicSplineInterpolator<double>();
    this->interp_z = _1D::CubicSplineInterpolator<double>();
    this->interp_x.setData(this->times.size(), this->times.data(), this->xs.data());
    this->interp_y.setData(this->times.size(), this->times.data(), this->ys.data());
    this->interp_z.setData(this->times.size(), this->times.data(), this->zs.data());
    RCLCPP_INFO(this->get_logger(), "Interpolators initiated");

    // Go to first point
    try {
        RCLCPP_INFO(this->get_logger(), "Waiting for Mission Start before takeoff to height %f", this->zs[0]);

        // Wait for mission start signal
        if (!this->waitForMissionStart()) {
            throw std::runtime_error("Mission aborted while waiting for mission start");
        }

        RCLCPP_INFO(this->get_logger(), "Mission Start Received, Proceeding to First Point");

        this->gotoTrajectoryPoint(req->trajectory.points[0]);

    } catch (const std::exception& e) {
		res->message = e.what();
        res->success = false;
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		this->reset();
        try {this->landVehicle();} catch (const std::exception &e1) {}
		return;
	}

    RCLCPP_INFO(this->get_logger(), "Mission Start Received, Proceeding to Execute Submitted Trajectory");

    // Set start time to current time after mission start has been received
    this->start_time = this->now();

    // Start trajectory timer
    this->resetExecutionTimer();

    res->message = "Trajectory initialisation successful, executing submitted trajectory";
    res->success = true;
}

void TrajectoryHandler::updateTrajectory(const rclcpp::Time& stamp){

    // Check Safety
    if(!this->checkAbort()) {
        return;
    }

    // Get time elapsed since start of trajectory
    auto time_elapsed = stamp - this->start_time;

    // Is time elapsed after the maximum time in the trajectory
    if (time_elapsed > std::chrono::duration<double>(this->max_time_sec)) {
        // Then stop update loop and exit
        RCLCPP_INFO(this->get_logger(), "Reached final trajectory point");
        this->landVehicle();
        this->reset();
        return;
    }

    double time_elapsed_sec = time_elapsed.seconds();
    auto req = std::make_shared<simple_offboard_msgs::srv::SetPosition::Request>();
    req->x = this->interp_x(time_elapsed_sec);
    req->y = this->interp_y(time_elapsed_sec);
    req->z = this->interp_z(time_elapsed_sec);
    req->yaw = 0.0; // Make interpolate later
    req->frame_id = "map";
    req->auto_arm = true;

    while (!this->sp_client->wait_for_service(std::chrono::duration<double>(0.01))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set mode service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for set mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto result_future = this->sp_client->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Sent request (t=%f) (%f, %f, %f)", time_elapsed_sec, req->x, req->y, req->z);
}

bool TrajectoryHandler::waitForMissionStart() {
    while(!this->mission_started) {
        RCLCPP_INFO(this->get_logger(), "Waiting For Mission Start");
        if(this->mission_aborted) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

void TrajectoryHandler::gotoTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point) {
    auto nav_req = std::make_shared<simple_offboard_msgs::srv::Navigate::Request>();
    nav_req->x = point.positions[0];
    nav_req->y = point.positions[1];
    nav_req->z = point.positions[2];
    nav_req->yaw = 0.0; //Todo
    nav_req->frame_id = "map"; // Make variable
    nav_req->auto_arm = true;
    nav_req->blocking = true;

    while (!this->navigate_client->wait_for_service(std::chrono::duration<double>(5.0))) {
        if (!rclcpp::ok() || !this->checkAbort()) {
            throw std::runtime_error("Interrupted while waiting for navigate service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "Navigate Service not available, waiting again...");
    }
    auto result_future = this->navigate_client->async_send_request(nav_req);
    if (result_future.wait_for(std::chrono::duration<double>(this->get_to_first_point_timeout)) != std::future_status::ready)
    {
        throw std::runtime_error("Navigate Service call failed");
    }
    auto result = result_future.get();
    if (!result->success) {
        throw std::runtime_error("Navigate Service errored with: " + result->message);
    }
}

void TrajectoryHandler::landVehicle() {
    while (!this->land_client->wait_for_service(std::chrono::duration<double>(0.01))) {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Interrupted while waiting for landing service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "Land Service not available, waiting again...");
    }
    this->land_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "Landing Request Sent");
}

bool TrajectoryHandler::checkAbort() {
    if(this->mission_aborted) {
        RCLCPP_ERROR(this->get_logger(), "MISSION ABORTED OR ESTOP PRESSED, CANCELLING TRAJECTORY");
        this->landVehicle();
        this->reset();
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto handler = std::make_shared<TrajectoryHandler>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(handler);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
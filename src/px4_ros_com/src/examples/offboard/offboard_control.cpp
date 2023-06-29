/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info.
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include "DroneInteractor.h"
#include <fstream>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	std::vector<px4_msgs::msg::TrajectorySetpoint> trajectorySetpointsFromFile;
	int positionInTrajectorySetpointVector = 0;
	float currentLocalPositionX;
	float currentLocalPositionY;
	float currentLocalPositionZ;
	bool messageFullySent = false;
	bool armed = false;
	bool velocityPresentInTrajectory = false;
	std::string mode;
	float cruiseSpeed = 2.0;
	bool publishedTrajectory = false;
	std::string mavMode;
	DroneInteractor droneInterObj;
	std::string fileName;
	std::ofstream outfile;
	bool takeoff;
	OffboardControl() : Node("offboard_control")
	{
		takeoff = true;
		this->declare_parameter("mode", "waypoint");
		mode = this->get_parameter("mode").as_string();
		outfile.open("test.txt", std::ios_base::app);
		// file = std::ofstream out("outFile.txt");
		std::cout << mode << std::endl;
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
															   [this](const px4_msgs::msg::Timesync::UniquePtr msg)
															   {
																   timestamp_.store(msg->timestamp);
															   });

		subToGroundComputer_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("trajectorySetpointTopic", 10,
																							[this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
																							{
																								std::cout << "Got msg with position: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
																								std::cout << "And Velocity: " << msg->vx << ", " << msg->vy << ", " << msg->vz << std::endl;
																								std::cout << "Timestamp: " << msg->timestamp << std::endl;
																								if (trajectorySetpointsFromFile.size() == 0)
																								{
																									if (!std::isnan(msg->vx))
																									{
																										velocityPresentInTrajectory = true;
																									}
																								}
																								if (msg->timestamp == 0)
																								{
																									this->messageFullySent = true;
																									std::cout << "Trajectory was fully sent!..." << std::endl;
																								}
																								else
																								{
																									this->trajectorySetpointsFromFile.push_back(*msg);
																								}
																							});

		subscriptionMAVMode_ = this->create_subscription<std_msgs::msg::String>("MAVMode", 10,
																				[this](const std_msgs::msg::String::UniquePtr msg)
																				{
																					std::cout << "Set MAV-Mode to " << msg->data << "-mode!" << std::endl;
																					this->mavMode = msg->data;
																				});
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
																					   [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
																					   {
																						   this->currentLocalPositionX = msg->x;
																						   this->currentLocalPositionY = msg->y;
																						   this->currentLocalPositionZ = msg->z;
																					   });

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void
		{
			if (mavMode == "offboard")
			{
				if (!armed)
				{
					this->arm();
					armed = true;
					takeoff = false;
					init_publish_offboard_control_mode();
					publish_takeoff();
				}
				if (offboard_setpoint_counter_ < 10)
				{
					publish_offboard_control_mode();
					trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[0]);
				}
				if (offboard_setpoint_counter_ == 10)
				{
					// Change to Offboard mode after 10 setpoints
					outfile << "offboard command sent at ";
					outfile << duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
							<< "\n";
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				}
				offboard_setpoint_counter_++;
				// offboard_control_mode needs to be paired with trajectory_setpoint
				if (trajectorySetpointsFromFile.size() > 0 && this->messageFullySent)
				{
					// std::cout << "takeoff is" << takeoff << std::endl;
					if (!takeoff)
					{
						init_publish_offboard_control_mode();
						publish_takeoff();
						std::cout << "current pos z" << currentLocalPositionZ << std::endl;
						if (abs(abs(currentLocalPositionZ) - 2) < 0.1)
						{
							takeoff = true;
						}
					}
					else
					{
						std::cout << "position at 0: " << trajectorySetpointsFromFile[0].x << std::endl;
						if (std::isnan(trajectorySetpointsFromFile[0].x))
						{
							publish_offboard_control_mode();
							std::cout << "calling trajectory without position..." << std::endl;
							publish_trajectory_without_position();
							timer_.reset();
						}
						else
						{
							if (abs(currentLocalPositionX - trajectorySetpointsFromFile[0].x) < 0.5 && abs(currentLocalPositionY - trajectorySetpointsFromFile[0].y) < 0.5 && abs(currentLocalPositionZ - abs(trajectorySetpointsFromFile[0].z) < 0.5))
							{
								publish_offboard_control_mode();
								publish_trajectory_setpoint();
							}
							else
							{
								publish_offboard_control_mode();
								trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[0]);
								std::cout << "Publishing first setpoint" << std::endl;
								std::cout << "Telemetry Position is: " << currentLocalPositionX << ", " << currentLocalPositionY << ", " << currentLocalPositionZ << std::endl;
							}
						}
					}
				}
				// mavMode = "none";
			}
			else if (mavMode == "mission")
			{
				// std::cout << "Mission mode activated..." << std::endl;
				if (this->messageFullySent)
				{
					droneInterObj.flyMission(trajectorySetpointsFromFile, false);
					mavMode = "none";
				}
			}
			else
			{
				std::cout << "mavMode variable is not set!" << std::endl;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}
	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionMAVMode_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr subToGroundComputer_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;

	std::atomic<uint64_t>
		timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
								 float param2 = 0.0);

	// uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	// void publish_offboard_control_mode();
	void init_publish_offboard_control_mode();
	void publish_takeoff();
	// void publish_trajectory_setpoint();
	void publish_trajectory_without_position();
	// void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};
void OffboardControl::publish_takeoff()
{
	TrajectorySetpoint msg{};
	msg.x = 0.0;
	msg.y = 0.0;
	msg.z = -2.0;
	// {0.0, 0.0, -2.0};
	trajectory_setpoint_publisher_->publish(msg);
}
void OffboardControl::publish_trajectory_without_position()
{
	uint64_t currentTime = 0;
	for (auto i : trajectorySetpointsFromFile)
	{
		if (i.timestamp <= currentTime)
		{
			publish_offboard_control_mode();
			i.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher_->publish(i);
			std::cout << "publishng: Acceleration " << i.acceleration[0] << ", " << i.acceleration[1] << std::endl;
			std::cout << "publishng: Acceleration " << i.vx << ", " << i.vy << std::endl;
			// std::cout << "currentTime is " << currentTime << std::endl;
			// std::cout << "i.timestamp is " << i.timestamp << std::endl;
			// std::cout << "published i..." << std::endl;
		}
		else
		{
			std::cout << "Waiting for " << i.timestamp - currentTime << "ms" << std::endl;
			std::cout << "i.timestamp" << i.timestamp << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(i.timestamp - currentTime));
			currentTime += i.timestamp - currentTime - 1;
			publish_offboard_control_mode();
			// std::cout << "currentTime is " << currentTime << std::endl;
			// std::cout << "i.timestamp is " << i.timestamp << std::endl;
			// std::cout << "publishing i now..." << std::endl;
			std::cout << "publishng: Acceleration " << i.acceleration[0] << ", " << i.acceleration[1] << std::endl;
			i.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher_->publish(i);
		}
	}
}
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	std::cout << "Got Telemetry Data: " << currentLocalPositionX << ", " << currentLocalPositionY << ", " << currentLocalPositionZ << std::endl;
	if (mode == "waypoint")
	{
		if (abs(currentLocalPositionX - trajectorySetpointsFromFile[positionInTrajectorySetpointVector].x) < 0.5 && abs(currentLocalPositionY - trajectorySetpointsFromFile[positionInTrajectorySetpointVector].y) < 0.5)
		{
			if (positionInTrajectorySetpointVector < trajectorySetpointsFromFile.size() - 1)
			{
				positionInTrajectorySetpointVector++;
				std::cout << "publishing message with position info: " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].x << ", " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].y << ", " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].z << std::endl;
				// trajectorySetpointsFromFile[positionInTrajectorySetpointVector].yaw = atan2(trajectorySetpointsFromFile[positionInTrajectorySetpointVector].velocity[1], trajectorySetpointsFromFile[positionInTrajectorySetpointVector].velocity[0]);
				// std::cout << "calculated yaw: " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].yaw << std::endl;
				// publish_offboard_control_mode();
				trajectorySetpointsFromFile[positionInTrajectorySetpointVector].vx = NAN;
				trajectorySetpointsFromFile[positionInTrajectorySetpointVector].vy = NAN;
				trajectorySetpointsFromFile[positionInTrajectorySetpointVector].vz = NAN;
				//  = {NAN, NAN, NAN};
				trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[positionInTrajectorySetpointVector]);
			}
			else if (positionInTrajectorySetpointVector == trajectorySetpointsFromFile.size() - 1)
			{
				std::cout << "Reached Endposition!" << std::endl;
			}
			else
			{
				std::cout << "Index Value is not valid!" << std::endl;
			}
		}
		else
		{
			std::cout << "publishing message with position info: " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].x << ", " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].y << ", " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].z << std::endl;
			// publish_offboard_control_mode();
			trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[positionInTrajectorySetpointVector]);
		}
	}
	else if (mode == "publishDirect")
	{
		for (auto i : trajectorySetpointsFromFile)
		{
			trajectory_setpoint_publisher_->publish(i);
		}
	}
	else if (mode == "trajectory" && !publishedTrajectory)
	{
		float timeInMilliSecondsToNextWaypoint = 0.0;
		int numberOfHundredMillisecondSteps = 0;
		float distanceTraveledAfterHundredMillsec = 0.0;
		std::vector<float> directionVectorBetweenSetpoints = {0.0, 0.0, 0.0};
		TrajectorySetpoint msg{};
		for (std::vector<TrajectorySetpoint>::size_type i = 0; i != trajectorySetpointsFromFile.size() - 1; i++)
		{
			timeInMilliSecondsToNextWaypoint = (sqrt(pow(trajectorySetpointsFromFile[i].x - trajectorySetpointsFromFile[i + 1].x, 2) + pow(trajectorySetpointsFromFile[i].y - trajectorySetpointsFromFile[i + 1].y, 2) + pow(trajectorySetpointsFromFile[i].z - trajectorySetpointsFromFile[i + 1].z, 2)) / cruiseSpeed) * 1000.0;
			msg = trajectorySetpointsFromFile[i];

			directionVectorBetweenSetpoints[0] = trajectorySetpointsFromFile[i + 1].x - trajectorySetpointsFromFile[i].x;
			directionVectorBetweenSetpoints[1] = trajectorySetpointsFromFile[i + 1].y - trajectorySetpointsFromFile[i].y;
			directionVectorBetweenSetpoints[2] = trajectorySetpointsFromFile[i + 1].z - trajectorySetpointsFromFile[i].z;

			std::cout << "Wait to publish next offboard message for " << timeInMilliSecondsToNextWaypoint << " milliseconds..." << std::endl;
			std::cout << "Publishing position: " << trajectorySetpointsFromFile[i].x << ", " << trajectorySetpointsFromFile[i].y << ", " << trajectorySetpointsFromFile[i].z << std::endl;
			publish_offboard_control_mode();
			trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[i]);

			numberOfHundredMillisecondSteps = round(timeInMilliSecondsToNextWaypoint) / 10;
			// std::cout << numberOfHundredMillisecondSteps << std::endl;
			for (int j = 0; j < numberOfHundredMillisecondSteps; j++)
			{
				std::cout << "Was inisde of for loop..." << std::endl;
				distanceTraveledAfterHundredMillsec = cruiseSpeed * 0.1;
				msg.x = msg.x + directionVectorBetweenSetpoints[0] * distanceTraveledAfterHundredMillsec;
				msg.y = msg.y + directionVectorBetweenSetpoints[1] * distanceTraveledAfterHundredMillsec;
				msg.z = msg.z + directionVectorBetweenSetpoints[2] * distanceTraveledAfterHundredMillsec;
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				publish_offboard_control_mode();
				std::cout << "Publishing intermediate point: " << msg.x << ", " << msg.y << ", " << msg.z << std::endl;
				trajectory_setpoint_publisher_->publish(msg);
			}
			// std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		publishedTrajectory = true;
	}
	else
	{
		std::cout << "Cant interpret mode!" << std::endl;
		if (outfile.is_open())
		{
			outfile.close();
		}
	}
	// else {
	// 	//trajectorySetpointsFromFile[positionInTrajectorySetpointVector].yaw = atan2(trajectorySetpointsFromFile[positionInTrajectorySetpointVector].velocity[1], trajectorySetpointsFromFile[positionInTrajectorySetpointVector].velocity[0]);
	// 	std::cout << "calculated yaw: " << trajectorySetpointsFromFile[positionInTrajectorySetpointVector].yaw << std::endl;
	// 	trajectory_setpoint_publisher_->publish(trajectorySetpointsFromFile[positionInTrajectorySetpointVector]);
	// }

	// msg.position = {0.0, 0.0, -5.0};
	// msg.yaw = -3.14; // [-PI:PI]
	// msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// trajectory_setpoint_publisher_->publish(msg);
}
/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
// void OffboardControl::publish_trajectory_setpoint()
// {
// 	TrajectorySetpoint msg{};
// 	msg.timestamp = timestamp_.load();
// 	msg.x = 0.0;
// 	msg.y = 0.0;
// 	msg.z = -5.0;
// 	msg.yaw = -3.14; // [-PI:PI]

// 	trajectory_setpoint_publisher_->publish(msg);
// }
void OffboardControl::init_publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
											  float param2)
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

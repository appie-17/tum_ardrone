#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ROSTHREAD_H
#define __ROSTHREAD_H
 
 
#ifndef Q_MOC_RUN

#include "tum_ardrone_gui.h"
#include "tum_ardrone/drone_state.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include "stdio.h"
#include "cvd/thread.h"
#include <unistd.h>
#endif
class tum_ardrone_gui;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

class RosThread : private CVD::Thread
{
private:
	// the associated thread's run function.
	void run();

	// keep Running
	bool keepRunning;
	bool pressedZero;
	bool pressedOne;
	bool pressedTwo;
	bool pressedThree;
	bool pressedFour;	
	bool pressedFive;	
	bool pressedSix;
	bool pressedSeven;
	bool pressedThirteen;
	bool pressedFourteen;
	bool pressedFifteen;
	bool pressedSixteen;
	
	// ros stuff
	ros::Subscriber dronepose_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber joy_sub;
	ros::Subscriber vel_sub;
	ros::Publisher vel_pub;
	ros::Subscriber comDrone_sub;
	ros::Publisher comDrone_pub;
	ros::Subscriber autopilot_sub;
	ros::Publisher autopilot_pub;
	ros::Subscriber takeoff_sub;
	ros::Publisher takeoff_pub;
	ros::Subscriber land_sub;
	ros::Publisher land_pub;
	ros::Subscriber flattrim_sub;
	ros::Publisher flattrim_pub;
	ros::Subscriber emergency_sub;
	ros::Publisher emergency_pub;
	ros::Subscriber flip_sub;
	ros::Publisher flip_pub;
	ros::Subscriber snapshot_sub;
	ros::Publisher snapshot_pub;
	ros::Subscriber autoTakeoff_sub;
	ros::Publisher autoTakeoff_pub;
	ros::Subscriber togglePilotMode_sub;
	ros::Publisher togglePilotMode_pub;
	ros::Subscriber toggleCam_sub;
	ros::Publisher toggleCam_pub;

	ros::NodeHandle nh_;

	// counters for Hz
	unsigned int dronePoseCount;
	unsigned int velCount;
	unsigned int odomCount;
	unsigned int joyCount;
	unsigned int velCount100ms;

	static pthread_mutex_t send_CS;
public:
	RosThread(void);
	~RosThread(void);

	// start and stop system and respective thread.
	// to be called externally
	void startSystem();
	void stopSystem();

	tum_ardrone_gui* gui;

	// callbacks
	void droneposeCb(const nav_msgs::OdometryConstPtr statePtr);
	void odomCb(const nav_msgs::OdometryConstPtr odomPtr);
	void joyCb(const sensor_msgs::JoyConstPtr joy_msg);
	void velCb(const geometry_msgs::TwistConstPtr vel);
	void comCb(const std_msgs::StringConstPtr str);
	void autopilotCb(std_msgs::EmptyConstPtr);
	void takeoffCb(std_msgs::EmptyConstPtr);
	void landCb(std_msgs::EmptyConstPtr);
	void togglepilotmodeCb(std_msgs::EmptyConstPtr);
	void togglecamCb(std_msgs::EmptyConstPtr);
	void flattrimCb(std_msgs::EmptyConstPtr);
	void emergencyCb(std_msgs::EmptyConstPtr);
	void flipCb(std_msgs::UInt8ConstPtr);
	void snapshotCb(std_msgs::EmptyConstPtr);
	void autotakeoffCb(std_msgs::EmptyConstPtr);

	ControlCommand lastJoyControlSent;

	// send command functions. can be called from any thread & are thread-safe.
	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void sendControlToDrone(ControlCommand cmd);
	void sendAutopilot();
	void sendTakeoff();
	void sendLand();
	void sendTogglePilotMode();
	void sendToggleCam();
	void sendFlattrim();
	void sendEmergency();
	void sendFlip(int flip);
	void sendSnapshot();
	void sendAutoTakeoff();
};

#endif /* __ROSTHREAD_H */

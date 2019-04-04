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
#ifndef __ESTIMATIONNODE_H
#define __ESTIMATIONNODE_H
 
#include "ros/ros.h"
#include "ros/package.h"
#include <dynamic_reconfigure/server.h>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"

#include "tf/tfMessage.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/TransformStamped.h"

#include "../HelperFunctions.h"
#include "DroneKalmanFilter.h"
#include "PTAMWrapper.h"
#include "MapView.h"
#include "tum_ardrone/drone_state.h"
#include "tum_ardrone/ptam_state.h"
#include "tum_ardrone/StateestimationParamsConfig.h"
#include "TooN/se3.h"
#include "deque"
#include <sys/stat.h>
#include <string>

class DroneKalmanFilter;
class MapView;
class PTAMWrapper;

struct EstimationNode
{
private:
	// comm with drone
	ros::Subscriber odom_sub; // drone odom
	ros::Subscriber vel_sub; // to co-read contro commands sent from other thread
	ros::Subscriber vid_sub;
	ros::Subscriber state_sub;
	ros::Time lastOdomStamp;

	// comm with ptam
	ros::Subscriber drone_sub;
	ros::Publisher drone_pub;
	static pthread_mutex_t tum_ardrone_CS;

	// output
	ros::Publisher dronepose_pub;
	ros::Publisher ptamstate_pub;

	ros::NodeHandle nh_;

	tf::TransformBroadcaster tf_broadcaster;

	// parameters
	// every [publishFreq]ms, the node publishes the drones predicted position [predTime]ms into the future.
	// this pose can then be used to steer the drone. obviously, the larger [predTime], the worse the estimate.
	// this pose is published on /tf, and simultaneously further info is published on /ardrone/predictedPose
	double predTime;
	int publishFreq;
	std::string param_ns;

	// for odom time-smoothing
	long lastDroneTS;
	long lastRosTS;
	long droneRosTSOffset;

	// save last odom received for forwarding...
	nav_msgs::Odometry lastOdomReceived;

public:
	// filter
	DroneKalmanFilter* filter;
	PTAMWrapper* ptamWrapper;
	MapView* mapView;
	std::string calibFile;
	std::string camTopic;
	std::string packagePath;

	EstimationNode();
	~EstimationNode();

	// ROS message callbacks
	void odomCb(const nav_msgs::OdometryConstPtr odomPtr);
	void velCb(const geometry_msgs::TwistConstPtr velPtr);
	void vidCb(const sensor_msgs::ImageConstPtr img);
	void comCb(const std_msgs::StringConstPtr str);
	void dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level);

	// main pose-estimation loop
	void Loop();

	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void reSendInfo();

	void publishTf(nav_msgs::Odometry odom);
	void publishTf(TooN::SE3<> trans, ros::Time stamp, int seq, std::string system);

};
#endif /* __ESTIMATIONNODE_H */

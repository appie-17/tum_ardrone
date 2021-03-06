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
 
#include "KIFlyTo.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIFlyTo::KIFlyTo(DronePosition checkpointP, 
		double stayTime,
		double maxControlFactorP,
		double initialReachedDistP,
		double stayWithinDistP)
{
	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;
	initialReachedDist = initialReachedDistP;
	stayWithinDist = stayWithinDistP;

	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;
	
	targetSet = false;

	isCompleted = false;	
	
	poseMsg.header.stamp = ros::Time().now();
	poseMsg.pose.position.x = checkpointP.pos[0];
	poseMsg.pose.position.y = checkpointP.pos[1];
  poseMsg.pose.position.z = checkpointP.pos[2];

	char buf[200];
	snprintf(buf,200,"goto %.2f %.2f %.2f %.2f", checkpointP.pos[0], checkpointP.pos[1], checkpointP.pos[2], checkpointP.yaw);
	command = buf;
}


KIFlyTo::~KIFlyTo(void)
{
}

bool KIFlyTo::update(const nav_msgs::OdometryConstPtr statePtr)
{
	double state_roll, state_pitch, state_yaw;
	q2rpy(TooN::makeVector(statePtr->pose.pose.orientation.x,
						   statePtr->pose.pose.orientation.y,
						   statePtr->pose.pose.orientation.z,
						   statePtr->pose.pose.orientation.w), &state_roll, &state_pitch, &state_yaw);

	if(!targetSet)
		controller->setTarget(checkpoint);
	targetSet = true;

	// target reached?
	if(!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs)
	{
		printf("checkpoint done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}


	// get target dist:
	TooN::Vector<3> diffs = TooN::makeVector(
			statePtr->pose.pose.position.x - checkpoint.pos[0],
			statePtr->pose.pose.position.y - checkpoint.pos[1],
			statePtr->pose.pose.position.z - checkpoint.pos[2]);

	double diffYaw = state_yaw - checkpoint.yaw;
	double diffDistSquared = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

	// if not reached yet, need to get within small radius to count.
	if(!reached && diffDistSquared < initialReachedDist * initialReachedDist && diffYaw*diffYaw < 25)
	{
		reached = true;
		reachedAtClock = getMS();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if(reached && (diffDistSquared > stayWithinDist * stayWithinDist || diffYaw*diffYaw > 25))
	{
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}

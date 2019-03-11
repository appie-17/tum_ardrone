 /**

 */
 
#include "KIFlyFor.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIFlyFor::KIFlyFor(double pitch, double roll, double yaw, double gaz, double flyTime, double maxControlFactorP)
{
	cmd.roll = roll;
	cmd.pitch = pitch;
	cmd.gaz = gaz; 
	cmd.yaw = yaw;
	
	flyTimeMs = (int)(1000*flyTime);
	startTimeMs = getMS();
	maxControlFactor = maxControlFactorP;
	
	reached = false;	
	isCompleted = false;

}

KIFlyFor::~KIFlyFor(void)
{
}


bool KIFlyFor::update(const nav_msgs::OdometryConstPtr statePtr)
{
	currentTimeMs = getMS();
	// target reached?
	if(!isCompleted && reached)
	{
		printf("checkpoint done!\n");
		isCompleted = true;
	}

	if(isCompleted)
	{		
		return true;
	}

	if(!reached && (currentTimeMs - startTimeMs) > flyTimeMs)
	{
		reached = true;
	}
	// control!		
	node->sendControlToDrone(cmd);
	// not done yet (!)
}

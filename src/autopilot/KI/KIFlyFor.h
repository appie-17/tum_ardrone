#pragma once
 /*
 */
#ifndef __KIFLYFOR_H
#define __KIFLYFOR_H
 

#include "KIProcedure.h"

class KIFlyFor : public KIProcedure
{
private:
	int reachedAtClock;
	bool reached;
	bool targetSet;
	bool isCompleted;

	ControlCommand cmd;
	int flyTimeMs;
	int startTimeMs;
	int currentTimeMs;
	double maxControlFactor;


public:
	KIFlyFor(double pitch, double roll, double yaw, double gaz, double flyTime, double maxControlFactorP = 1);

	~KIFlyFor(void);
	bool update(const nav_msgs::OdometryConstPtr statePtr);
};

#endif /* __KIFLYFOR_H */

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
 
#include "RosThread.h"
#include <unistd.h>
#include "ros/callback_queue.h"
#include "tum_ardrone_gui.h"
#include "stdio.h"

pthread_mutex_t RosThread::send_CS = PTHREAD_MUTEX_INITIALIZER;
RosThread::RosThread()
{
	gui = NULL;
	odomCount = velCount = dronePoseCount = joyCount = velCount100ms = 0;
	keepRunning = true;
	lastJoyControlSent = ControlCommand(0,0,0,0);
}

RosThread::~RosThread(void)
{
}

void RosThread::startSystem()
{
	keepRunning = true;
	start();
}

void RosThread::stopSystem()
{
	keepRunning = false;
	join();
}

void RosThread::droneposeCb(const nav_msgs::OdometryConstPtr statePtr)
{
	dronePoseCount++;
}

void RosThread::odomCb(const nav_msgs::OdometryConstPtr odomPtr)
{

	odomCount++;
}

void RosThread::joyCb(const sensor_msgs::JoyConstPtr joy_msg)
{
    joyCount++;

    if (joy_msg->axes.size() < 4) {
        ROS_WARN_ONCE("Error: Non-compatible Joystick!");
        return;
    }

    // Avoid crashes if non-ps3 joystick is being used
    short unsigned int actiavte_index = (joy_msg->buttons.size() > 11) ? 11 : 1;

	// if not controlling: start controlling if sth. is pressed (!)
    bool justStartedControlling = false;
	if(gui->currentControlSource != CONTROL_JOY)
	{
		if(		joy_msg->axes[0] > 0.1 ||  joy_msg->axes[0] < -0.1 ||
				joy_msg->axes[1] > 0.1 ||  joy_msg->axes[1] < -0.1 ||
				joy_msg->axes[2] > 0.1 ||  joy_msg->axes[2] < -0.1 ||
				joy_msg->axes[3] > 0.1 ||  joy_msg->axes[3] < -0.1 ||
                joy_msg->buttons.at(actiavte_index))
		{
			gui->setControlSource(CONTROL_JOY);
			justStartedControlling = true;
		}
	}

	// are we actually controlling with the Joystick?
	if(justStartedControlling || gui->currentControlSource == CONTROL_JOY)
	{
		ControlCommand c;
		c.yaw = joy_msg->axes[0] * -1;
		c.gaz = joy_msg->axes[1] * 1;
		c.pitch = joy_msg->axes[3] * 1;
		c.roll = joy_msg->axes[2] * -1;

		sendControlToDrone(c);
		lastJoyControlSent = c;

		//Button 6 is dead-man button
		//Turn on autopilot (Cross)
		if(joy_msg->buttons.at(14) & (pressedZero!=true))
		{
			sendAutopilot();
			pressedZero=true;
		}
		else if(!joy_msg->buttons.at(14))
			pressedZero=false;
		//Toggle on fpv camera	(Circle)		
		if(joy_msg->buttons.at(13) & (pressedOne!=true))
		{
			sendToggleCam();
			pressedOne=true;
		}
		else if(!joy_msg->buttons.at(13))
			pressedOne=false;
		//Calibrate gyroscope on flat surface (flattrim, Triangle)
        if(joy_msg->buttons.at(12) & (pressedTwo!=true))
        {
			sendFlattrim();
			pressedTwo=true;
		}
		else if(!joy_msg->buttons.at(12))
			pressedTwo=false;			
		//Toggle piloting-mode (Square)
        if(joy_msg->buttons.at(15) & (pressedThree!=true))
        {
			sendTogglePilotMode();
			pressedThree=true;
		}
		else if(!joy_msg->buttons.at(15))
			pressedThree=false;
		//Takeoff (L1)
        if(joy_msg->buttons.at(10) & (pressedFour!=true))
        {
			sendTakeoff();	
			pressedFour=true;
		}
		else if(!joy_msg->buttons.at(10))
			pressedFour=false;
		//Land (R1)
        if(joy_msg->buttons.at(11) & (pressedFive!=true))
        {
			sendLand();
			pressedFive=true;
		}
		else if(!joy_msg->buttons.at(11))
			pressedFive=false;
		//Emergency (R2)
        if(joy_msg->buttons.at(9) & (pressedSeven!=true))
        {
			sendEmergency();
			pressedSeven=true;
		}
		else if(!joy_msg->buttons.at(9))
			pressedSeven=false;
		//Auto-takeoff (Up)
        if((joy_msg->buttons.at(4) & (pressedThirteen!=true)) & !((joy_msg->buttons.at(8) & (pressedSix!=true)))) 
		{
			sendAutoTakeoff();
			pressedThirteen=true;
		}
		else if(!joy_msg->buttons.at(4))
			pressedThirteen=false;        
        //Snapshot downwards (Down)
        if((joy_msg->buttons.at(6) & (pressedFourteen!=true)) & !((joy_msg->buttons.at(8) & (pressedSix!=true)))) 
        {
			sendSnapshot();
			pressedFourteen=true;
		}
		else if(!joy_msg->buttons.at(6))
			pressedFourteen=false;			
		//Send+Clear flight pattern command (Left)
        if((joy_msg->buttons.at(7) & (pressedFifteen!=true)) & !((joy_msg->buttons.at(8) & (pressedSix!=true))))
        {
			gui->ClearSendClicked();
			pressedFifteen=true;
		}
		else if(!joy_msg->buttons.at(7))
			pressedFifteen=false;
			pressedSix=false;					
		//Front-flip (L2 + Up)
        if((joy_msg->buttons.at(4) & (pressedThirteen!=true)) & ((joy_msg->buttons.at(8) & (pressedSix!=true))))
        {
			sendFlip(0);
			pressedThirteen=true;
			pressedSix=true;
		}
		else if(!joy_msg->buttons.at(4))
			pressedThirteen=false;
			pressedSix=false;
		//Back-flip (L2 + Down)
        if((joy_msg->buttons.at(6) & (pressedFourteen!=true)) & ((joy_msg->buttons.at(8) & (pressedSix!=true))))
        {
			sendFlip(1);
			pressedFourteen=true;
			pressedSix=true;
		}
		else if(!joy_msg->buttons.at(6))
			pressedFourteen=false;
			pressedSix=false;			
		//Left-flip (L2 + Left)
        if((joy_msg->buttons.at(7) & (pressedFifteen!=true)) & ((joy_msg->buttons.at(8) & (pressedSix!=true))))
        {
			sendFlip(2);
			pressedFifteen=true;
			pressedSix=true;
		}
		else if(!joy_msg->buttons.at(7))
			pressedFifteen=false;
			pressedSix=false;			
		//Right-flip (L2 + right)
        if((joy_msg->buttons.at(5) & (pressedSixteen!=true)) & ((joy_msg->buttons.at(8) & (pressedSix!=true))))
        {
			sendFlip(3);		
			pressedSixteen=true;
			pressedSix=true;
		}
		else if(!joy_msg->buttons.at(5))
			pressedSixteen=false;
			pressedSix=false;	
	}
}


void RosThread::velCb(const geometry_msgs::TwistConstPtr vel)
{
    velCount++;
    velCount100ms++;
}

void RosThread::comCb(const std_msgs::StringConstPtr str)
{
	if(str->data.substr(0,2) == "u ")
	{
		if(str->data.substr(0,4) == "u l ")
			gui->addLogLine(str->data.substr(4,str->data.length()-4));

		else if(str->data.substr(0,4) == "u c ")
			gui->setAutopilotInfo(str->data.substr(4,str->data.length()-4));

		else if(str->data.substr(0,4) == "u s ")
			gui->setStateestimationInfo(str->data.substr(4,str->data.length()-4));
	}
}

void RosThread::autopilotCb(std_msgs::EmptyConstPtr)
{
	gui->setControlSource(CONTROL_AUTO);
	gui->addLogLine("sent: AutoPilotOn");
}

void RosThread::takeoffCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Takeoff");
}
void RosThread::landCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Land");
}

void RosThread::togglepilotmodeCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Toggle pilotmode");
}

void RosThread::togglecamCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Camera turned on");
}
void RosThread::flattrimCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Flattrim");
}
void RosThread::flipCb(std_msgs::UInt8ConstPtr)
{
	gui->addLogLine("sent: Execute Flip");
}
void RosThread::emergencyCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Emergency Reset");
}
void RosThread::snapshotCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Snapshot taken");
}
void RosThread::autotakeoffCb(std_msgs::EmptyConstPtr)
{
	gui->addLogLine("sent: Auto takeoff enabled");
}

void RosThread::run()
{
    dronepose_sub = nh_.subscribe(nh_.resolveName("predictedPose"),50, &RosThread::droneposeCb, this);
    odom_sub = nh_.subscribe(nh_.resolveName("odom"),50, &RosThread::odomCb, this);
    joy_sub	= nh_.subscribe(nh_.resolveName("joy"),50, &RosThread::joyCb, this);
    vel_sub	= nh_.subscribe(nh_.resolveName("cmd_vel"),50, &RosThread::velCb, this);
    vel_pub	= nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),50);
	comDrone_sub = nh_.subscribe(nh_.resolveName("com"),50, &RosThread::comCb, this);
	comDrone_pub = nh_.advertise<std_msgs::String>(nh_.resolveName("com"),50);
    autopilot_sub = nh_.subscribe(nh_.resolveName("autopilot"),1, &RosThread::autopilotCb, this);
    autopilot_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("autopilot"),1);
    takeoff_sub = nh_.subscribe(nh_.resolveName("takeoff"),1, &RosThread::takeoffCb, this);
    takeoff_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("takeoff"),1);
    land_sub = nh_.subscribe(nh_.resolveName("land"),1, &RosThread::landCb, this);
    land_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("land"),1);
	emergency_sub = nh_.subscribe(nh_.resolveName("emergency"),1, &RosThread::emergencyCb, this);
	emergency_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("emergency"),1);
	flip_sub = nh_.subscribe(nh_.resolveName("flip"), 1, &RosThread::flipCb, this);
	flip_pub = nh_.advertise<std_msgs::UInt8>(nh_.resolveName("flip"),1);
	autoTakeoff_sub = nh_.subscribe(nh_.resolveName("auto_takeoff"), 1, &RosThread::autotakeoffCb, this);
	autoTakeoff_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("auto_takeoff"),1);
	snapshot_sub = nh_.subscribe(nh_.resolveName("snapshot"), 1, &RosThread::snapshotCb, this);
	snapshot_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("snapshot"),1);
	togglePilotMode_sub = nh_.subscribe(nh_.resolveName("fast_mode"), 1, &RosThread::togglepilotmodeCb, this);
	togglePilotMode_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("fast_mode"), 1);
	toggleCam_sub = nh_.subscribe(nh_.resolveName("toggle_cam"),1, &RosThread::togglecamCb, this);
    toggleCam_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("toggle_cam"),1);
	flattrim_sub = nh_.subscribe(nh_.resolveName("flattrim"),1, &RosThread::flattrimCb, this);
    flattrim_pub  = nh_.advertise<std_msgs::Empty>(nh_.resolveName("flattrim"),1);

	ros::Time last = ros::Time::now();
	ros::Time lastHz = ros::Time::now();

	while(keepRunning && nh_.ok())
	{
		// spin for 100ms
		while((ros::Time::now() - last) < ros::Duration(0.1))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();

		// if nothing on /cmd_vel, repeat!
		if(velCount100ms == 0)
			switch(gui->currentControlSource)
			{
			case CONTROL_AUTO:
				sendControlToDrone(ControlCommand(0,0,0,0));
				break;
			case CONTROL_JOY:
				sendControlToDrone(lastJoyControlSent);
				break;
			case CONTROL_KB:
				sendControlToDrone(gui->calcKBControl());
				break;
			case CONTROL_NONE:
				sendControlToDrone(ControlCommand(0,0,0,0));
				break;
			}
		velCount100ms = 0;

		// if 1s passed: update Hz values
		if((ros::Time::now() - lastHz) > ros::Duration(1.0))
		{
			gui->setCounts(odomCount, velCount, dronePoseCount, joyCount);
			odomCount = velCount = dronePoseCount = joyCount = 0;
			lastHz = ros::Time::now();
		}
	}

	gui->closeWindow();
	
	if(nh_.ok()) ros::shutdown();
	std::cout << "Exiting ROS Thread (ros::shutdown() has been called)" << std::endl;
}

void RosThread::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&send_CS);
	comDrone_pub.publish(s);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::sendControlToDrone(ControlCommand cmd)
{
	// TODO: check converstion (!)
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.y = cmd.pitch;
	cmdT.linear.x = cmd.roll;

	pthread_mutex_lock(&send_CS);
	vel_pub.publish(cmdT);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::sendAutopilot()
{
	pthread_mutex_lock(&send_CS);
	autopilot_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}


void RosThread::sendLand()
{
	pthread_mutex_lock(&send_CS);
	land_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendTakeoff()
{
	pthread_mutex_lock(&send_CS);
	takeoff_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}

void RosThread::sendTogglePilotMode()
{	
	pthread_mutex_lock(&send_CS);
	togglePilotMode_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendToggleCam()
{
	pthread_mutex_lock(&send_CS);
	toggleCam_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendFlattrim()
{
	pthread_mutex_lock(&send_CS);
	flattrim_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendEmergency()
{
	pthread_mutex_lock(&send_CS);
	emergency_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendFlip(int flip)
{
	std_msgs::UInt8 m;
	m.data = flip;
	pthread_mutex_lock(&send_CS);
	flip_pub.publish(m);
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendSnapshot()
{
	pthread_mutex_lock(&send_CS);
	snapshot_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}
void RosThread::sendAutoTakeoff()
{
	pthread_mutex_lock(&send_CS);
	autoTakeoff_pub.publish(std_msgs::Empty());
	pthread_mutex_unlock(&send_CS);
}

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
 
 
#include "EstimationNode.h"

using namespace std;

EstimationNode::EstimationNode()
{
	// Fetch ros parameters
	packagePath = ros::package::getPath("tum_ardrone");

	ros::param::param("~calibFile", calibFile, packagePath + string("/camcalib/no_calib.txt") );
	ros::param::param("~publishFreq", publishFreq, 30);
	ros::param::param("~predTime", predTime, 0.025); // in sec
	ros::param::param("~camTopic", camTopic, string("camera/image_raw") );
	ros::param::param("~ns", param_ns, string("") );

	odom_sub		= nh_.subscribe(nh_.resolveName("odom"), 10, &EstimationNode::odomCb, this);
	vel_sub			= nh_.subscribe(nh_.resolveName("cmd_vel"),10, &EstimationNode::velCb, this);
	vid_sub			= nh_.subscribe(nh_.resolveName(camTopic),10, &EstimationNode::vidCb, this);
	dronepose_pub	= nh_.advertise<nav_msgs::Odometry>(nh_.resolveName("predictedPose"),1);
	ptamstate_pub	= nh_.advertise<tum_ardrone::ptam_state>(nh_.resolveName("ptamState"), 10);
	drone_pub		= nh_.advertise<std_msgs::String>(nh_.resolveName("com"),10);
	drone_sub		= nh_.subscribe(nh_.resolveName("com"),10, &EstimationNode::comCb, this);

	// other internal vars
	lastDroneTS = 0;
	lastRosTS = 0;
	droneRosTSOffset = 0;
	lastOdomStamp = ros::Time(0);
	filter = new DroneKalmanFilter(this);
	ptamWrapper = new PTAMWrapper(filter, this);
	mapView = new MapView(filter, ptamWrapper, this);
}

EstimationNode::~EstimationNode()
{
	filter->release();
	delete mapView;
	delete ptamWrapper;
	delete filter;
}

void EstimationNode::odomCb(const nav_msgs::OdometryConstPtr odomPtr)
{
	lastOdomReceived = *odomPtr;
	if(ros::Time::now() - lastOdomReceived.header.stamp > ros::Duration(30.0))
		lastOdomReceived.header.stamp = ros::Time::now();
/*
	// darn ROS really messes up timestamps.
	// they should arrive every 5ms, with occasionally dropped packages.
	// instead, they arrive with gaps of up to 30ms, and then 6 packages with the same timestamp.
	// so: this procedure "smoothes out" received package timestamps, shifting their timestamp by max. 20ms to better fit the order.
	long rosTS = getMS(lastNavdataReceived.header.stamp);

	//long rosTS = lastNavdataReceived.header.stamp.sec*1000 + lastNavdataReceived.header.stamp.nsec/1000;
	long droneTS = navdataPtr->tm;

	if(lastDroneTS == 0) lastDroneTS = droneTS;

	droneRosTSOffset = 0.9 * droneRosTSOffset + 0.1*(rosTS - droneTS);

	long rosTSNew = droneTS + droneRosTSOffset;	// this should be the correct timestamp.
	long TSDiff = std::min(500l,std::max(-500l,rosTSNew-rosTS));	// never change by more than 500ms.
	lastNavdataReceived.header.stamp += ros::Duration(TSDiff/1000.0);	// change!
	lastRosTS = rosTS;
	lastDroneTS = droneTS;
*/
	// push back in filter queue.
	pthread_mutex_lock( &filter->filter_CS );
	filter->odomQueue->push_back(lastOdomReceived);
	pthread_mutex_unlock( &filter->filter_CS );

	// give to PTAM (for scale estimation)
	ptamWrapper->newOdom(&lastOdomReceived);

	// save last timestamp
	lastOdomStamp = lastOdomReceived.header.stamp;
}

void EstimationNode::velCb(const geometry_msgs::TwistConstPtr velPtr)
{
	geometry_msgs::TwistStamped ts;
	ts.header.stamp = ros::Time::now();
	ts.twist = *velPtr;

	pthread_mutex_lock( &filter->filter_CS );
	filter->velQueue->push_back(ts);
	pthread_mutex_unlock( &filter->filter_CS );
}

void EstimationNode::vidCb(const sensor_msgs::ImageConstPtr img)
{
	// give to PTAM
	ptamWrapper->newImage(img);
}

void EstimationNode::comCb(const std_msgs::StringConstPtr str)
{
	if(str->data.length() > 2 && str->data.substr(0,2) == "p ")
	{
		ptamWrapper->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	if(str->data.length() > 2 && str->data.substr(0,2) == "f ")
	{
		mapView->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	if(str->data.length() > 2 && str->data.substr(0,2) == "m ")
	{
		mapView->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	int a, b;
	if(sscanf(str->data.c_str(),"pings %d %d",&a, &b) == 2)
	{
		filter->setPing((unsigned int)a, (unsigned int)b);
		predTime = (0.001*filter->delayControl);	// set predTime to new delayControl
	}
}

void EstimationNode::Loop()
{
	  ros::Rate pub_rate(publishFreq);
	  ros::Time lastInfoSent = ros::Time::now();

	  while (nh_.ok())
	  {
		  // -------------- 1. put nav & control in internal queues. ---------------
		  ros::spinOnce();

		  pthread_mutex_lock( &filter->filter_CS );
		  // -------------- 3. get predicted pose and publish! ---------------
		  // get filter odom msg

		  nav_msgs::Odometry odom = filter->getPoseAt(ros::Time().now() + ros::Duration(predTime) );

		  pthread_mutex_unlock( &filter->filter_CS );

		  // fill metadata
		  odom.header.stamp = ros::Time().now() + ros::Duration(filter->delayControl/1000);
		  odom.header.frame_id = param_ns + string("/map");
		  odom.child_frame_id = param_ns + string("/base_link");
		  // publish!
		  dronepose_pub.publish(odom);  
		  publishTf(odom);
		  
			//	ROS_WARN("Timestamp base: %i",ros_header_timestamp_base);
		  // --------- if need be: add fake PTAM obs --------
		  // if PTAM updates hang (no video or e.g. init), filter is never permanently rolled forward -> queues get too big.
		  // dont allow this to happen by faking a ptam observation if queue gets too big (500ms = 100 observations)
		  if((getMS(ros::Time().now()) - filter->predictdUpToTimestamp) > 150)
			  filter->addFakePTAMObservation(getMS(ros::Time().now()) - filter->delayVideo);

		  // ---------- maybe send new ptam info --------------------------
		  if((ros::Time::now() - lastInfoSent) > ros::Duration(0.4))
		  {
			  reSendInfo();
			  lastInfoSent = ros::Time::now();
		  }
		  // -------------- 4. sleep until rate is hit. ---------------
		  pub_rate.sleep();
	  }
}

void EstimationNode::dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level)
{
	if(!filter->allSyncLocked && config.PTAMSyncLock)
		ROS_WARN("Ptam Sync has been disabled. This fixes scale etc.");

	if(!ptamWrapper->mapLocked && config.PTAMMapLock)
		ROS_WARN("Ptam Map has been locked.");

	filter->useControl = config.UseControlGains;
	filter->usePTAM = config.UsePTAM;
	filter->useOdom = config.UseNavdata;

	filter->useScalingFixpoint = config.RescaleFixOrigin;

	ptamWrapper->maxKF = config.PTAMMaxKF;
	ptamWrapper->mapLocked = config.PTAMMapLock;
	filter->allSyncLocked = config.PTAMSyncLock;

	ptamWrapper->setPTAMPars(config.PTAMMinKFTimeDiff, config.PTAMMinKFWiggleDist, config.PTAMMinKFDist);

	filter->c1 = config.c1;
	filter->c2 = config.c2;
	filter->c3 = config.c3;
	filter->c4 = config.c4;
	filter->c5 = config.c5;
	filter->c6 = config.c6;
	filter->c7 = config.c7;
	filter->c8 = config.c8;
	
	filter->base_delayXYZ = config.delay_xyz;
	filter->base_delayVideo = config.delay_video;			
	filter->base_delayControl = config.delay_control;
}

pthread_mutex_t EstimationNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;

void EstimationNode::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&tum_ardrone_CS);
	drone_pub.publish(s);
	pthread_mutex_unlock(&tum_ardrone_CS);
}

void EstimationNode::publishTf(nav_msgs::Odometry odom)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = odom.header.stamp;
  transformStamped.header.frame_id = odom.header.frame_id;
  transformStamped.child_frame_id = odom.child_frame_id;
  transformStamped.transform.translation.x = odom.pose.pose.position.x;
  transformStamped.transform.translation.y = odom.pose.pose.position.y;
  transformStamped.transform.translation.z = odom.pose.pose.position.z;

  transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

  tf_broadcaster.sendTransform(transformStamped);
}

void EstimationNode::publishTf(TooN::SE3<> trans, ros::Time stamp, int seq, std::string system)
{
	trans = trans.inverse();

	tf::Matrix3x3 m;
	m[0][0] = trans.get_rotation().get_matrix()(0,0);
	m[0][1] = trans.get_rotation().get_matrix()(0,1);
	m[0][2] = trans.get_rotation().get_matrix()(0,2);
	m[1][0] = trans.get_rotation().get_matrix()(1,0);
	m[1][1] = trans.get_rotation().get_matrix()(1,1);
	m[1][2] = trans.get_rotation().get_matrix()(1,2);
	m[2][0] = trans.get_rotation().get_matrix()(2,0);
	m[2][1] = trans.get_rotation().get_matrix()(2,1);
	m[2][2] = trans.get_rotation().get_matrix()(2,2);

	tf::Vector3 v;
	v[0] = trans.get_translation()[0];
	v[1] = trans.get_translation()[1];
	v[2] = trans.get_translation()[2];

	tf::Transform tr = tf::Transform(m,v);
	tf::StampedTransform t = tf::StampedTransform(tr,stamp, param_ns + string("/map"), param_ns + system);
	tf_broadcaster.sendTransform(t);
}

void EstimationNode::reSendInfo()
{
	// get ptam status
	tum_ardrone::ptam_state ptam_state;
	switch(ptamWrapper->PTAMStatus)
	{
	case PTAMWrapper::PTAM_IDLE:
		ptam_state.state = 0;
		break;
	case PTAMWrapper::PTAM_INITIALIZING:
		ptam_state.state = 1;
		break;
	case PTAMWrapper::PTAM_LOST:
		ptam_state.state = 2;
		break;
	case PTAMWrapper::PTAM_FALSEPOSITIVE:
		ptam_state.state = 6;
		break;
	case PTAMWrapper::PTAM_GOOD:
		ptam_state.state = 3;
		break;
	case PTAMWrapper::PTAM_TOOKKF:
		ptam_state.state = 5;	
		break;
	case PTAMWrapper::PTAM_BEST:
		ptam_state.state = 4;
		break;
	}

	ptam_state.scale = filter->getCurrentScales()[0];
	ptam_state.scaleAccuracy = filter->getScaleAccuracy();
	ptamstate_pub.publish(ptam_state);
	// parse PTAM message
	std::string ptamMsg = ptamWrapper->lastPTAMMessage;
	int kf, kp, kps[4], kpf[4];
	int pos = ptamMsg.find("Found: ");
	int found = 0;
	if(pos != std::string::npos)
		found = sscanf(ptamMsg.substr(pos).c_str(),"Found: %d/%d %d/%d %d/%d %d/%d Map: %dP, %dKF",
						&kpf[0],&kps[0],&kpf[1],&kps[1],&kpf[2],&kps[2],&kpf[3],&kps[3],&kp,&kf);
	char bufp[200];
	if(found == 10)
		snprintf(bufp,200,"Map: KF: %d, KP: %d (%d of %d found)",
				kf, kp,kpf[0]+kpf[1]+kpf[2]+kpf[3], kps[0]+kps[1]+kps[2]+kps[3]);
	else
		snprintf(bufp,200,"Map: -");

	std::string status = "";
/*	switch(	lastNavdataReceived.state)
	{
		case 0: status = "Unknown"; break;
		case 1: status = "Init"; break;
		case 2: status = "Landed";break;
		case 3: status = "Flying"; break;
		case 4: status = "Hovering"; break;
		case 5: status = "Test"; break;
		case 6: status = "Taking off"; break;
		case 7: status = "Goto Fix Point"; break;
		case 8: status = "Landing"; break;
		case 9: status = "Looping"; break;
	}
	*/
	/*
	PTAM: Idle | Good | Dodgy | Lost
	Map: KF: X, KP: X (X searched, X found)
	Scale: X (in: X, out: x), acc: X
	Scale Fixpoint: NONE | DRONE
	Status: X (Battery: X)
	*/
	char buf[1000];
	/*snprintf(buf,1000,"u s PTAM: %s\n%s\nScale: %.3f (%d in, %d out), acc: %.2f\nScaleFixpoint: %s\nDrone Status: %s (%d Battery)",
			ptamStatus.c_str(),
			bufp,
			filter->getCurrentScales()[0],filter->scalePairsIn,filter->scalePairsOut,filter->getScaleAccuracy(),
			filter->useScalingFixpoint ? "FIX" : "DRONE",
			status.c_str(), (int)lastNavdataReceived.batteryPercent);

	publishCommand(buf);
*/
}

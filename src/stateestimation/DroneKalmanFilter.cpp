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
 
#include "DroneKalmanFilter.h"

// constants (variances assumed to be present)
const double varSpeedObservation_xy = .4*.4;
const double varPoseObservation_xy = .2*.2;
const double varAccelerationError_xy = 1*1;

const double varPoseObservation_z_PTAM = .08*.08;
const double varPoseObservation_z_IMU = .1*.1;
const double varPoseObservation_z_IMU_NO_PTAM = .1*.1;

const double varPoseObservation_rp_PTAM = 3*3;
const double varPoseObservation_rp_IMU = .4*.4;
const double varSpeedError_rp = 360*360 * 16;	// increased because prediction based on control command is damn inaccurate.

const double varSpeedObservation_yaw_IMU = 100*100; 
const double varSpeedObservation_yaw = .5*.5;
const double varPoseObservation_yaw_PTAM = .3*.3;
const double varAccelerationError_yaw = 360*360;
	

// constants (assumed delays in ms).
// default ping values: odom=25, vid=50
int DroneKalmanFilter::delayRPY = 0;		// always zero
int DroneKalmanFilter::delayXYZ = 40;		// base_delayXYZ, but at most delayVideo
int DroneKalmanFilter::delayVideo = 50;		// base_delayVideo + delayVid - delayOdom
int DroneKalmanFilter::delayControl = 50;	// base_delayControl + 2*delayOdom

int DroneKalmanFilter::base_delayXYZ = 40;		// t_xyz - t_rpy = base_delayXYZ
int DroneKalmanFilter::base_delayVideo = 50;		// t_cam - t_rpy = base_delayVideo + delayVid - delayOdom
int DroneKalmanFilter::base_delayControl = 45;	// t_control + t_rpy - 2*delayOdom

using namespace std;

// transform degree-angle to satisfy min <= angle < sup
double angleFromTo(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}

pthread_mutex_t DroneKalmanFilter::filter_CS = PTHREAD_MUTEX_INITIALIZER;

DroneKalmanFilter::DroneKalmanFilter(EstimationNode* n)
{
	scalePairs = new std::vector<ScaleStruct>();
	odomQueue = new std::deque<nav_msgs::Odometry>();
	velQueue = new std::deque<geometry_msgs::TwistStamped>();

	useScalingFixpoint = false;

	this->node = n;

	pthread_mutex_lock( &filter_CS );
	reset();
	pthread_mutex_unlock( &filter_CS );

	c1 = 0.58;
	c2 = 17.8;
	c3 = 10;
	c4 = 35;
	c5 = 10;
	c6 = 25;
	c7 = 1.4;
	c8 = 1.0;
}


DroneKalmanFilter::~DroneKalmanFilter(void)
{
	// dont delete nothing here, as this is also called for shallow copy.
}

void DroneKalmanFilter::release()
{
	delete scalePairs;
	delete odomQueue;
	delete velQueue;
}

void DroneKalmanFilter::setPing(unsigned int odomPing, unsigned int vidPing)
{
	// add a constant of 20ms // 40ms to accound for delay due to ros.
	// very, very rough approximation.
	odomPing += 20;
	vidPing += 40;

	int new_delayXYZ = base_delayXYZ + odomPing;
	int new_delayVideo = base_delayVideo + vidPing/(int)2 - odomPing/(int)2;
	int new_delayControl = base_delayControl + odomPing;
	
	delayXYZ = std::min(5000,std::max(40,new_delayXYZ));
	delayVideo = std::min(5000,std::max(50,new_delayVideo));
	delayControl = std::min(5000,std::max(50,new_delayControl));

}

void DroneKalmanFilter::reset()
{
	// init filter with pose 0 (var 0) and speed 0 (var large).
	x = y = z = yaw = PVFilter(0);
	roll = pitch = PFilter(0);
	predictedUpToDroneTime = 0;
	last_z_heightDiff = 0;
	scalePairsIn = scalePairsOut = 0;

	// set statistic parameters to zero
	numGoodIMUObservations = 0;

	// set last times to 0, indicating that there was no prev. package.
	lastIMU_XYZ_dronetime = lastIMU_RPY_dronetime = 0;
	lastIMU_dronetime = 0;

	// clear PTAM
	clearPTAM();

	// clear IMU-queus
	odomQueue->clear();
	velQueue->clear();

	predictdUpToTimestamp = getMS(ros::Time::now());

	predictedUpToTotal = -1;

	baselineZ_Filter = baselineZ_IMU = -999999;
	baselinesYValid = false;

	node->publishCommand("u l EKF has been reset to zero.");
}

void DroneKalmanFilter::clearPTAM()
{
	// offsets and scales are not initialized.
	offsets_xyz_initialized = scale_xyz_initialized = false;
	xy_scale = z_scale = scale_from_xy = scale_from_z = 1;
	roll_offset = pitch_offset = yaw_offset = x_offset = y_offset = z_offset = 0;

	xyz_sum_IMUxIMU = 0.1;
	xyz_sum_PTAMxPTAM = 0.1;
	xyz_sum_PTAMxIMU = 0.1;
	scalePairs->clear();
	scalePairsIn = 1;
	scalePairsOut = 0;

	rp_offset_framesContributed = 0;

	// set statistic parameters to zero
	numGoodPTAMObservations = 0;

	// indicates that last PTAM frame was not valid
	lastPosesValid = false;
}

// this function does the actual work, predicting one timestep ahead.
void DroneKalmanFilter::predictInternal(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains)
{
	if(timeSpanMicros <= 0) return;
	
	useControlGains = useControlGains && this->useControl;

	bool controlValid = !(activeControlInfo.linear.z > 1.01 || activeControlInfo.linear.z < -1.01 ||
			activeControlInfo.linear.x > 1.01 || activeControlInfo.linear.x < -1.01 ||
			activeControlInfo.linear.y > 1.01 || activeControlInfo.linear.y < -1.01 ||
			activeControlInfo.angular.z > 1.01 || activeControlInfo.angular.z < -1.01);

	double tsMillis = timeSpanMicros / 1000.0;	// in milliseconds
	double tsSeconds = tsMillis / 1000.0;	// in seconds

	// predict roll, pitch, yaw
	float rollControlGain = tsSeconds*c3*(c4 * max(-0.5, min(0.5, (double)activeControlInfo.linear.x)) - roll.state);
	float pitchControlGain = tsSeconds*c3*(c4 * min(0.5, max(-0.5, (double)-activeControlInfo.linear.y)) - pitch.state);
	float yawSpeedControlGain = tsSeconds*c5*(c6 * activeControlInfo.angular.z - yaw.state[1]);	// at adaption to ros, this has to be reverted for some reason....
	
	double yawRad = yaw.state[0] * 3.14159268 / 180;
	double rollRad = roll.state * 3.14159268 / 180;
	double pitchRad = pitch.state * 3.14159268 / 180;
	
	double forceX = cos(yawRad) * sin(rollRad) * cos(pitchRad) - sin(yawRad) * sin(pitchRad);
	double forceY = - sin(yawRad) * sin(rollRad) * cos(pitchRad) - cos(yawRad) * sin(pitchRad);

	double vx_gain = tsSeconds * c1 * (c2*forceX - x.state[1]);
	double vy_gain = tsSeconds * c1 * (c2*forceY - y.state[1]);
	double vz_gain = tsSeconds * c7 * (c8*activeControlInfo.linear.z*(activeControlInfo.linear.z < 0 ? 1 : 1) - z.state[1]);

	lastVXGain = vx_gain;
	lastVYGain = vy_gain;
	lastPredictedRoll = roll.state;
	lastPredictedPitch = pitch.state;
	
	if(!useControlGains || !controlValid)
	{
		vx_gain = vy_gain = vz_gain = 0;
		rollControlGain = pitchControlGain = yawSpeedControlGain = 0;
	}
	yaw.state[0] = angleFromTo(yaw.state[0],-180,180);
	
	roll.predict(tsMillis,varSpeedError_rp, rollControlGain);
	
	pitch.predict(tsMillis,varSpeedError_rp, pitchControlGain);
	
	yaw.predict(tsMillis,varAccelerationError_yaw,TooN::makeVector(tsSeconds*yawSpeedControlGain/2,yawSpeedControlGain),1,varSpeedObservation_yaw);
	
	yaw.state[0] = angleFromTo(yaw.state[0],-180,180);
	
	x.predict(tsMillis,varAccelerationError_xy,TooN::makeVector(tsSeconds*vx_gain/2,vx_gain),0.00000001);
	
	y.predict(tsMillis,varAccelerationError_xy,TooN::makeVector(tsSeconds*vy_gain/2,vy_gain),0.00000001);
	
	z.predict(tsMillis,TooN::makeVector(tsSeconds*tsSeconds*tsSeconds*tsSeconds, 9*tsSeconds,tsSeconds*tsSeconds*tsSeconds*3),
		TooN::makeVector(tsSeconds*vz_gain/2,vz_gain));
}

void DroneKalmanFilter::observeIMU_XYZ(const nav_msgs::Odometry* odom)
{	
	if (last_y_IMU != odom->twist.twist.linear.y || last_x_IMU != odom->twist.twist.linear.x)
	{
		// --------------- now: update  ---------------------------
		// transform to global CS, using current yaw
		double yawRad = yaw.state[0] * 3.14159268 / 180;
//		double vx_global = (sin(yawRad) * odom->twist.twist.linear.y + cos(yawRad) * odom->twist.twist.linear.x);
//		double vy_global = (cos(yawRad) * odom->twist.twist.linear.y - sin(yawRad) * odom->twist.twist.linear.x);
        double vx_global = odom->twist.twist.linear.x;
        double vy_global = odom->twist.twist.linear.y;        
		if(vx_global > 10)
			cout << "err";

		// update x,y:
		// if PTAM isGood, assume "normal" accuracy. if not, assume very accurate speeds
		// to simulate "integrating up".
		double lastX = x.state[0];
		double lastY = y.state[0];
		if(lastPosesValid)
		{
			x.observeSpeed(vx_global,varSpeedObservation_xy*50);
			y.observeSpeed(vy_global,varSpeedObservation_xy*50);
		}
		else
		{
			x.observeSpeed(vx_global,varSpeedObservation_xy);
			y.observeSpeed(vy_global,varSpeedObservation_xy);
		}

		if(abs(lastX-x.state[0]) > 0.5 && lastX != 0)
		{
			// this happens if there was no odom for a long time -> EKF variances got big -> new update leads to large jump in pos.
			ROS_WARN("detected large x jump. removing. should not happen usually (only e.g. if no odom for a long time, or agressive re-scaling)");
			x.state[0] = lastX;
		}
		if(abs(lastY-y.state[0]) > 0.5 && lastY != 0)
		{
			y.state[0] = lastY;
			ROS_WARN("detected large y jump. removing. should not happen usually (only e.g. if no odom for a long time, or agressive re-scaling)");
		}

		last_y_IMU = odom->twist.twist.linear.y;
		last_x_IMU = odom->twist.twist.linear.x;
	}
	// height is a bit more complicated....
	// only update every 8 packages, or if changed.
	if(last_z_IMU != odom->pose.pose.position.z) //|| odom->header.seq - last_z_packageID > 8)
	{
		if(baselineZ_Filter < -100000)	// only for initialization.
		{
			baselineZ_IMU = odom->pose.pose.position.z;
			baselineZ_Filter = z.state[0];
		}

		if(lastPosesValid)
		{

			double imuHeightDiff = (odom->pose.pose.position.z - baselineZ_IMU );	// TODO negative heights??
			double observedHeight = baselineZ_Filter + 0.5*(imuHeightDiff + last_z_heightDiff);
			last_z_heightDiff = imuHeightDiff;

			baselineZ_IMU = odom->pose.pose.position.z;
			baselineZ_Filter = z.state[0];

			if((abs(imuHeightDiff) < 0.350 && abs(last_z_heightDiff) < 0.350))	// jumps of more than 150mm in 40ms are ignored
			{
				z.observePose(observedHeight,varPoseObservation_z_IMU);
				lastdZ = observedHeight;
			}
		}
		else
		{
			double imuHeightDiff = (odom->pose.pose.position.z - baselineZ_IMU );
			double observedHeight = baselineZ_Filter + imuHeightDiff;

			if(abs(imuHeightDiff) < 0.350)	// jumps of more than 150mm in 40ms are ignored
			{
				z.observePose(observedHeight,varPoseObservation_z_IMU_NO_PTAM);
				lastdZ = observedHeight;
			}
			else	// there was a jump: dont observe anything, but set new baselines.
			{
				if(baselineZ_IMU == 0 || odom->pose.pose.position.z == 0)
				{
					z.observePose(observedHeight,0);
					z.observeSpeed(0,0);
				}

				baselineZ_IMU = odom->pose.pose.position.z;
				baselineZ_Filter = z.state[0];
			}
		}

		last_z_IMU = odom->pose.pose.position.z;
		last_z_packageID = odom->header.seq;
	}
}

void DroneKalmanFilter::observeIMU_RPY(const nav_msgs::Odometry* odom)
{			

	double odom_roll, odom_pitch, odom_yaw;
	q2rpy(TooN::makeVector(
	                         odom->pose.pose.orientation.x,
             			     odom->pose.pose.orientation.y,
			        		 odom->pose.pose.orientation.z,
			        		 odom->pose.pose.orientation.w), 
			        		 &odom_roll, &odom_pitch, &odom_yaw);

	if(last_roll_IMU!=odom_roll)
	{
		roll.observe(odom_roll,varPoseObservation_rp_IMU);
		last_roll_IMU = odom_roll;
	}
	
	if(last_pitch_IMU!=odom_pitch)
	{
		pitch.observe(odom_pitch,varPoseObservation_rp_IMU);
		last_pitch_IMU = odom_pitch;
	}
	
	if(last_yaw_IMU!=odom_yaw)
	{
		if(!baselinesYValid)	// only for initialization.
		{
			baselineY_IMU = odom_yaw;
			baselineY_Filter = yaw.state[0];
			baselinesYValid = true;
			timestampYawBaselineFrom = getMS(odom->header.stamp);
		}

		double imuYawDiff = (odom_yaw - baselineY_IMU );
		double observedYaw = baselineY_Filter + imuYawDiff;
	
		yaw.state[0] = angleFromTo(yaw.state[0],-180,180);
	
		if(yaw.state[0] < -90)
			observedYaw = angleFromTo(observedYaw,-360,0);
		else if(yaw.state[0] > 90)
			observedYaw = angleFromTo(observedYaw,0,360);
		else
			observedYaw = angleFromTo(observedYaw,-180,180);
		
	
		if(lastPosesValid)
		{
			baselineY_IMU = odom_yaw;
			baselineY_Filter = yaw.state[0];
			timestampYawBaselineFrom = getMS(odom->header.stamp);
	
			if(abs(observedYaw - yaw.state[0]) < 90)
			{
				yaw.observePose(observedYaw,varSpeedObservation_yaw_IMU/5);
				lastdYaw = observedYaw;
			}
		}
	
		else
		{	

			if(abs(observedYaw - yaw.state[0]) < 90)
			{
				yaw.observePose(observedYaw,varSpeedObservation_yaw_IMU);
				lastdYaw = observedYaw;
			}
		
			else
			{
				baselineY_IMU = odom_yaw;
				baselineY_Filter = yaw.state[0];
				timestampYawBaselineFrom = getMS(odom->header.stamp);
			}
		}
	
		last_yaw_IMU = odom_yaw;
		yaw.state[0] = angleFromTo(yaw.state[0],-180,180);
	}

}

void DroneKalmanFilter::observePTAM(TooN::Vector<6> pose)
{
	// ----------------- observe xyz -----------------------------
	// sync!
	if(!allSyncLocked)
	{
		sync_xyz(pose[0], pose[1], pose[2]);
		sync_rpy(pose[3], pose[4], pose[5]);
	}

	last_PTAM_pose = TooN::makeVector(pose[0], pose[1], pose[2]);

	pose.slice<0,3>() = transformPTAMObservation(pose[0], pose[1], pose[2]);

	if(offsets_xyz_initialized)
	{
		x.observePose(pose[0],varPoseObservation_xy);
		y.observePose(pose[1],varPoseObservation_xy);
	}

	// observe z
	if(offsets_xyz_initialized)
	{
		z.observePose(pose[2], varPoseObservation_z_PTAM);
	}

	// observe rpy
	if(rp_offset_framesContributed > 1)
	{
		roll.observe(pose[3]+roll_offset,varPoseObservation_rp_PTAM);
		pitch.observe(pose[4]+pitch_offset,varPoseObservation_rp_PTAM);

		double observedYaw = pose[5]+yaw_offset;
		
		yaw.observePose(observedYaw,varPoseObservation_yaw_PTAM);
	}

	last_fused_pose = TooN::makeVector(x.state[0], y.state[0], z.state[0]);
	lastPosesValid = true;
}

void DroneKalmanFilter::sync_rpy(double roll_global, double pitch_global, double yaw_global)
{
	if(allSyncLocked) return;
	
	// set yaw on first call
	if(rp_offset_framesContributed < 1)
	{
		yaw_offset = yaw.state[0] - yaw_global;
	}
	// update roll and pitch offset continuously as normal average.
	if(rp_offset_framesContributed < 100)
	{
		roll_offset = rp_offset_framesContributed * roll_offset + roll.state - roll_global;
		pitch_offset = rp_offset_framesContributed * pitch_offset + pitch.state - pitch_global;
	}
	
	rp_offset_framesContributed++;

	roll_offset /= rp_offset_framesContributed;
	pitch_offset /= rp_offset_framesContributed;

}

void DroneKalmanFilter::sync_xyz(double x_global, double y_global, double z_global)
{
	if(allSyncLocked) return;
	
	// ----------- offset: just take first available ---------
	if(!offsets_xyz_initialized)
	{
		x_offset = x.state[0] - x_global*xy_scale;
		y_offset = y.state[0] - y_global*xy_scale;
		z_offset = z.state[0] - z_global*z_scale;
	
		offsets_xyz_initialized = true;
	}
}

void DroneKalmanFilter::updateScaleXYZ(TooN::Vector<3> ptamDiff, TooN::Vector<3> imuDiff, TooN::Vector<3> OrgPtamPose)
{
	if(allSyncLocked) return;

	ScaleStruct s = ScaleStruct(ptamDiff, imuDiff);

	// dont add samples that are way to small...
	if(s.imuNorm < 0.05 || s.ptamNorm < 0.05) return;

	// update running sums
	(*scalePairs).push_back(s);

	double xyz_scale_old = xy_scale;


	// find median.
	std::sort((*scalePairs).begin(), (*scalePairs).end());
	double median = (*scalePairs)[((*scalePairs).size()+1)/2].alphaSingleEstimate;

	// hack: if we have only few samples, median is unreliable (maybe 2 out of 3 are completely wrong.
	// so take first scale pair in this case (i.e. the initial scale)
	if((*scalePairs).size() < 5)
		median = initialScaleSet;

	// find sums and median.
	// do separately for xy and z and xyz-all and xyz-filtered
	double sumII = 0;
	double sumPP = 0;
	double sumPI = 0;
	double totSumII = 0;
	double totSumPP = 0;
	double totSumPI = 0;
	
	double sumIIxy = 0;
	double sumPPxy = 0;
	double sumPIxy = 0;
	double sumIIz = 0;
	double sumPPz = 0;
	double sumPIz = 0;

	int numIn = 0;
	int numOut = 0;
	for(unsigned int i=0;i<(*scalePairs).size();i++)
	{
		if((*scalePairs).size() < 5 || ((*scalePairs)[i].alphaSingleEstimate > median * 0.2 && (*scalePairs)[i].alphaSingleEstimate < median / 0.2))
		{
			sumII += (*scalePairs)[i].ii;
			sumPP += (*scalePairs)[i].pp;
			sumPI += (*scalePairs)[i].pi;

			sumIIxy += (*scalePairs)[i].imu[0]*(*scalePairs)[i].imu[0] + (*scalePairs)[i].imu[1]*(*scalePairs)[i].imu[1];
			sumPPxy += (*scalePairs)[i].ptam[0]*(*scalePairs)[i].ptam[0] + (*scalePairs)[i].ptam[1]*(*scalePairs)[i].ptam[1];
			sumPIxy += (*scalePairs)[i].ptam[0]*(*scalePairs)[i].imu[0] + (*scalePairs)[i].ptam[1]*(*scalePairs)[i].imu[1];
		
			sumIIz += (*scalePairs)[i].imu[2]*(*scalePairs)[i].imu[2];
			sumPPz += (*scalePairs)[i].ptam[2]*(*scalePairs)[i].ptam[2];
			sumPIz += (*scalePairs)[i].ptam[2]*(*scalePairs)[i].imu[2];

			numIn++;
		}
		else
		{
			totSumII += (*scalePairs)[i].ii;
			totSumPP += (*scalePairs)[i].pp;
			totSumPI += (*scalePairs)[i].pi;
			numOut++;
		}
	}
	xyz_sum_IMUxIMU = sumII;
	xyz_sum_PTAMxPTAM = sumPP;
	xyz_sum_PTAMxIMU = sumPI;

	double scale_Filtered = (*scalePairs)[0].computeEstimator(sumPP,sumII,sumPI,0.2,0.01);
	double scale_Unfiltered = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,0.2,0.01);
	double scale_PTAMSmallVar = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,0.00001,1);
	double scale_IMUSmallVar = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,1,0.00001);

	double scale_Filtered_xy = (*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,0.2,0.01);
	double scale_Filtered_z = (*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,0.2,0.01);

	scalePairsIn = numIn;
	scalePairsOut = numOut;

	printf("scale: in: %i; out: %i, filt: %.3f; xyz: %.1f < %.1f < %.1f; xy: %.1f < %.1f < %.1f; z: %.1f < %.1f < %.1f;\n", 
		numIn, numOut, scale_Filtered, 
		scale_PTAMSmallVar, scale_Unfiltered, scale_IMUSmallVar,
		(*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,0.00001,1),
		scale_Filtered_xy,
		(*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,1,0.00001),
		(*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,0.00001,1),
		scale_Filtered_z,
		(*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,1,0.00001)
		);

	if(scale_Filtered > 0.1)
		z_scale = xy_scale = scale_Filtered;

	else
		ROS_WARN("calculated scale is too small %.3f, disallowing!",scale_Filtered);

	scale_from_xy = scale_Filtered_xy;
	scale_from_z = scale_Filtered_z;
	// update offsets such that no position change occurs (X = x_global*xy_scale_old + offset = x_global*xy_scale_new + new_offset)
	if(useScalingFixpoint)
	{
		// fix at fixpoint
		x_offset += (xyz_scale_old - xy_scale)*scalingFixpoint[0];
		y_offset += (xyz_scale_old - xy_scale)*scalingFixpoint[1];
		z_offset += (xyz_scale_old - z_scale)*scalingFixpoint[2];
	}
	else
	{
		// fix at current pos.
		x_offset += (xyz_scale_old - xy_scale)*OrgPtamPose[0];
		y_offset += (xyz_scale_old - xy_scale)*OrgPtamPose[1];
		z_offset += (xyz_scale_old - z_scale)*OrgPtamPose[2];
	}
	scale_xyz_initialized = true;
}

float DroneKalmanFilter::getScaleAccuracy()
{
	return 0.5 + 0.5*std::min(1.0,std::max(0.0,xyz_sum_PTAMxIMU * xy_scale/4));	// scale-corrected PTAM x IMU
}

void DroneKalmanFilter::predictUpTo(int timestamp, bool consume, bool useControlGains)
{	//timestamp is in msec
	if(predictdUpToTimestamp == timestamp)
	{	
	 return; 
	}
	// at this point:
	// - velQueue contains controls, timestamped with time at which they were sent.
	// - odomQueue contains odom, timestamped with time at which they were received.
	// - timestamp is the time up to which we want to predict, i.e. maybe a little bit into the future

	// start at [predictdUpToTimestamp]. predict step-by-step observing at [currentTimestamp] the
	// - rpy timestamped with [currentTimestamp + delayRPY]
	// - xyz timestamped with [currentTimestamp + delayXYZ]
	// using
	// - control timestamped with [currentTimestamp - delayControl]

	// fast forward until first package that will be used.
	// for controlIterator, this is the last package with a stamp smaller/equal than what it should be.
	// for both others, this is the first package with a stamp bigger than what it should be.
	// if consume, delete everything before permanently.
	std::deque<geometry_msgs::TwistStamped>::iterator controlIterator = velQueue->begin();
	while(controlIterator != velQueue->end() &&
			controlIterator+1 != velQueue->end() &&
			getMS((controlIterator+1)->header.stamp) + delayControl <= predictdUpToTimestamp)
		if(consume)
		{
			velQueue->pop_front();
			controlIterator = velQueue->begin();
		}
		else
			controlIterator++;

	if(velQueue->size() == 0) useControlGains = false;

	// dont delete here, it will be deleted if respective rpy data is consumed.

//Combine xyzIterator and rpyIterator since data is received together
	std::deque<nav_msgs::Odometry>::iterator odomIterator = odomQueue->begin();

	while(odomIterator != odomQueue->end() && getMS(odomIterator->header.stamp) - delayXYZ <= predictdUpToTimestamp)
	{
		if(consume)
		{
			odomQueue->pop_front();
			odomIterator = odomQueue->begin();
		}
		else
			odomIterator++;
	}

	// now, each iterator points to the first elemnent in queue that is to be integrated.
	// start predicting,
	while(true)
	{
		// predict ahead to [timestamp]
		int predictTo = timestamp;

		// but a maximum of 10ms per prediction step, to guarantee nonlinearities.
		predictTo = min(predictTo, predictdUpToTimestamp+10);

	while(odomIterator != odomQueue->end() && getMS(odomIterator->header.stamp) - delayXYZ < predictdUpToTimestamp)
		odomIterator++;
	
		// for control that is last message with stamp <= predictdUpToTimestamp - delayControl.
		while(controlIterator != velQueue->end() &&
				controlIterator+1 != velQueue->end() &&
				getMS((controlIterator+1)->header.stamp) + delayControl <= predictdUpToTimestamp)
			controlIterator++;

		// predict not further than the point in time where the next observation needs to be added.
		if(odomIterator != odomQueue->end())
			predictTo = min(predictTo, getMS(odomIterator->header.stamp)-delayXYZ);

		predictInternal(useControlGains ? controlIterator->twist : geometry_msgs::Twist(),
				(predictTo - predictdUpToTimestamp)*1000,
				useControlGains &&
				getMS(controlIterator->header.stamp) + delayControl + 100 > predictdUpToTimestamp);				//  control max. 100ms old.

		// if an observation needs to be added, it HAS to have a stamp equal to [predictTo],
		// as we just set [predictTo] to that timestamp.
		bool observedOdom = false;
					
		if(odomIterator != odomQueue->end() && getMS(odomIterator->header.stamp)-delayXYZ == predictTo)
		{
			if(this->useOdom)
			{
				observeIMU_RPY(&(*odomIterator));
				observeIMU_XYZ(&(*odomIterator));
			}
			
			observedOdom = true;
		}

		predictdUpToTimestamp = predictTo;
		if(observedOdom) odomIterator++;

		// if this is where we wanna get, quit.
		if(predictTo == timestamp)
			break;
	}
}

TooN::Vector<3> DroneKalmanFilter::transformPTAMObservation(double x,double y,double z)
{
	return transformPTAMObservation(x,y,z,yaw.state[0]);
}

TooN::Vector<3> DroneKalmanFilter::transformPTAMObservation(double x,double y,double z, double yaw)
{
	double yawRad = yaw * 3.14159268 / 180;
	x = x_offset + xy_scale*x - 0.0*sin(yawRad); //0.04 for offset front camera w.r.p. drone CS
	y = y_offset + xy_scale*y - 0.0*cos(yawRad);
	z = z_offset + z_scale*z;
	return TooN::makeVector(x,y,z);
}

TooN::Vector<6> DroneKalmanFilter::transformPTAMObservation(TooN::Vector<6> obs)
{
	obs.slice<0,3>() = transformPTAMObservation(obs[0], obs[1], obs[2], obs[5]);
	
	obs[3] += roll_offset;
	obs[4] += pitch_offset;
	obs[5] += yaw_offset;

	return obs;
}

TooN::Vector<6> DroneKalmanFilter::backTransformPTAMObservation(TooN::Vector<6> obs)
{
	obs[3] -= roll_offset;
	obs[4] -= pitch_offset;
	obs[5] -= yaw_offset;

	double yawRad = obs[5] * 3.14159268 / 180;
	obs[0] = (- x_offset + obs[0] + 0.0*sin(yawRad))/xy_scale; //0.04 for offset front camera w.r.p. drone CS
	obs[1] = (- y_offset + obs[1] + 0.0*cos(yawRad))/xy_scale;
	obs[2] = (- z_offset + obs[2])/z_scale;

	return obs;
}

TooN::Vector<6> DroneKalmanFilter::getCurrentPose()
{
	return TooN::makeVector(x.state[0], y.state[0], z.state[0], roll.state, pitch.state, yaw.state[0]);
}

nav_msgs::Odometry DroneKalmanFilter::getCurrentPoseSpeed()
{	

	nav_msgs::Odometry s;
	s.pose.pose.position.x = x.state[0];
	s.pose.pose.position.y = y.state[0];
	s.pose.pose.position.z = z.state[0];
	
//	tf::Quaternion q(pitch.state*3.14159268/180, roll.state*3.14159268/180, yaw.state[0]*3.14159268/180);
  tf::Quaternion q(pitch.state*3.14159268/180, roll.state*3.14159268/180, 0);
  tf::Quaternion q_rot (0, 0, yaw.state[0]*3.14159268/180);
	quaternionTFToMsg(q_rot*q, s.pose.pose.orientation);

	s.twist.twist.linear.x = x.state[1];
	s.twist.twist.linear.y = y.state[1];
	s.twist.twist.linear.z = z.state[1];
	s.twist.twist.angular.z = yaw.state[1];

	if(s.twist.twist.linear.x*s.twist.twist.linear.x < 0.001) s.twist.twist.linear.x = 0;
	if(s.twist.twist.linear.y*s.twist.twist.linear.y < 0.001) s.twist.twist.linear.y = 0;
	if(s.twist.twist.linear.z*s.twist.twist.linear.z < 0.001) s.twist.twist.linear.z = 0;
	if(s.pose.pose.position.x*s.pose.pose.position.x < 0.001) s.pose.pose.position.x = 0;
	if(s.pose.pose.position.y*s.pose.pose.position.y < 0.001) s.pose.pose.position.y = 0;
	if(s.pose.pose.position.z*s.pose.pose.position.z < 0.001) s.pose.pose.position.z = 0;

	return s;
}

TooN::Vector<10> DroneKalmanFilter::getCurrentPoseSpeedAsVec()
{
	return TooN::makeVector(x.state[0], y.state[0], z.state[0], roll.state, pitch.state, yaw.state[0],
		x.state[1], y.state[1], z.state[1],yaw.state[1]);
}

TooN::Vector<10> DroneKalmanFilter::getCurrentPoseSpeedVariances()
{
	return TooN::makeVector(x.var(0,0), y.var(0,0), z.var(0,0), roll.var, pitch.var, yaw.var(0,0),
		x.var(1,1), y.var(1,1), z.var(1,1),yaw.var(1,1));
}

TooN::Vector<6> DroneKalmanFilter::getCurrentPoseVariances()
{
	return TooN::makeVector(x.var(0,0), y.var(0,0), z.var(0,0), roll.var, pitch.var, yaw.var(0,0));
}

TooN::Vector<6> DroneKalmanFilter::getCurrentOffsets()
{
	TooN::Vector<6> res = TooN::makeVector(0,0,0,0,0,0);
	if(offsets_xyz_initialized)
		res.slice<0,3>() = TooN::makeVector(x_offset, y_offset, z_offset);
	if(rp_offset_framesContributed > 1)
		res.slice<3,3>() = TooN::makeVector(roll_offset, pitch_offset, yaw_offset);
	return res;
}

TooN::Vector<3> DroneKalmanFilter::getCurrentScales()
{
	return TooN::makeVector(scale_xyz_initialized ? xy_scale : 1, scale_xyz_initialized ? xy_scale : 1, scale_xyz_initialized ? z_scale : 1);
}


void DroneKalmanFilter::setCurrentScales(TooN::Vector<3> scales)
{
	if(allSyncLocked) return;
	xy_scale = scales[0];
	z_scale = scales[0];
	scale_from_xy = scale_from_z = scales[0];

	xyz_sum_IMUxIMU = 0.2 * scales[0];
	xyz_sum_PTAMxPTAM = 0.2 / scales[0];
	xyz_sum_PTAMxIMU = 0.2;

	(*scalePairs).clear();

	(*scalePairs).push_back(ScaleStruct(
		 TooN::makeVector(0.2,0.2,0.2) / sqrt(scales[0]),
		 TooN::makeVector(0.2,0.2,0.2) * sqrt(scales[0])
		));

	scale_xyz_initialized = true;
	offsets_xyz_initialized = false;

	initialScaleSet = scales[0];
}

void DroneKalmanFilter::addPTAMObservation(TooN::Vector<6> trans, int time)
{
	if(time > predictdUpToTimestamp)
		predictUpTo(time, true, true);

	observePTAM(trans);
	numGoodPTAMObservations++;
}
void DroneKalmanFilter::addFakePTAMObservation(int time)
{
	if(time > predictdUpToTimestamp)
		predictUpTo(time, true,true);

	lastPosesValid = false;
}

nav_msgs::Odometry DroneKalmanFilter::getPoseAt(ros::Time t, bool useControlGains)
{
	// make shallow copy
	DroneKalmanFilter scopy = DroneKalmanFilter(*this);

	// predict using this copy
	scopy.predictUpTo(getMS(t),false, useControlGains);

	// return value, and discard any changes made to scopy (deleting it)
	return scopy.getCurrentPoseSpeed();
}

TooN::Vector<10> DroneKalmanFilter::getPoseAtAsVec(int timestamp, bool useControlGains)
{
	// make shallow copy
	DroneKalmanFilter scopy = DroneKalmanFilter(*this);

	// predict using this copy
	scopy.predictUpTo(timestamp,false, useControlGains);

	// return value, and discard any changes made to scopy (deleting it)
	return scopy.getCurrentPoseSpeedAsVec();
}

bool DroneKalmanFilter::handleCommand(std::string s)
{
	return false;
}

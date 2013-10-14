#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>


class Stabilizer {
	
	public:
		Stabilizer();
		
		void publishTrajectory(char** argv);
		
	private:
		
		void imuCallback(const sensor_msgs::ImuConstPtr& msg);
		
		void publishTrajectory(const ros::TimerEvent&);
		
		ros::NodeHandle _nh;

		ros::Publisher _trajectoryPublisher;
		ros::Subscriber _imuSubscriber;
		ros::Timer _trajectoryTimer;
		
		double yaw, pitch, roll;
};


Stabilizer::Stabilizer()
{
	_trajectoryPublisher = _nh.advertise<trajectory_msgs::JointTrajectory>("/joints", 10);
	_imuSubscriber = _nh.subscribe("/sensors/imu", 1, &Stabilizer::imuCallback, this);
	
	_trajectoryTimer = _nh.createTimer(ros::Duration(1),
			&Stabilizer::publishTrajectory, this);
	
}

void Stabilizer::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
	
	tf::Matrix3x3 matrix(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
	
	matrix.getRPY(roll, pitch, yaw);
}

void Stabilizer::publishTrajectory(const ros::TimerEvent&) {

	trajectory_msgs::JointTrajectory msg, msg2;
	
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "chassis";
	msg.joint_names.push_back("joint_h_hb");
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(-roll);
	point.time_from_start = ros::Duration(1);
	msg.points.push_back(point);
	
	_trajectoryPublisher.publish(msg);
		
	ros::Duration(0.1).sleep();
	
	msg2.header.stamp = ros::Time::now();
	msg2.header.frame_id = "chassis";
	msg2.joint_names.push_back("joint_hb_ch");
	trajectory_msgs::JointTrajectoryPoint point2;
	point2.positions.push_back(-pitch);
	point2.time_from_start = ros::Duration(1);
	msg2.points.push_back(point2);
	
	_trajectoryPublisher.publish(msg2);
	
}
	
	
int main(int argc,char** argv) {
	
	
	ros::init(argc,argv,"stabilizer");
	
	Stabilizer stabilizer;
	
	ros::spin();
}

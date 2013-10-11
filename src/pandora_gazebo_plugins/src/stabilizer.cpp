#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <trajectory_msgs/JointTrajectory.h>


class Stabilizer {
	
	public:
		Stabilizer();
		
		void publishTrajectory(char** argv);
		
	private:
	
		
		
		ros::NodeHandle _nh;

		ros::Publisher _trajectoryPublisher;
};


Stabilizer::Stabilizer()
{
	_trajectoryPublisher = _nh.advertise<trajectory_msgs::JointTrajectory>("/joints", 10);
	
}



void Stabilizer::publishTrajectory(char** argv) {
long i=0;
while(1){
	i++;
	trajectory_msgs::JointTrajectory msg,msg2,msg3;
	
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "chassis";
	msg.joint_names.push_back("joint_h_hb");
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(i*0.01);
	point.time_from_start = ros::Duration(1);
	msg.points.push_back(point);
	
	_trajectoryPublisher.publish(msg);	
	usleep(100000);
	msg2.header.stamp = ros::Time::now();
	msg2.header.frame_id = "chassis";
	msg2.joint_names.push_back("joint_hb_ch");
	trajectory_msgs::JointTrajectoryPoint point2;
	point2.positions.push_back(i*0.01);
	point2.time_from_start = ros::Duration(1);
	msg2.points.push_back(point2);
	
	_trajectoryPublisher.publish(msg2);
	
	usleep(100000);
}
}
	
	
int main(int argc,char** argv) {
	
	
	ros::init(argc,argv,"stabilizer");
	
	Stabilizer stabilizer;
	
	ros::Duration(2).sleep();
	
	stabilizer.publishTrajectory(argv);
	
//~ 	ros::spin();
}

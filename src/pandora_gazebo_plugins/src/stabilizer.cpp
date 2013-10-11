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
	
	trajectory_msgs::JointTrajectory msg;
	
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "chassis";
	
	msg.joint_names.push_back("hokuyo_joint");
	
	trajectory_msgs::JointTrajectoryPoint point;
	
	point.positions.push_back(atof(argv[1]));
	point.positions.push_back(atof(argv[2]));
	point.positions.push_back(atof(argv[3]));
	
	point.velocities.push_back(atof(argv[4]));
	point.velocities.push_back(atof(argv[5]));
	point.velocities.push_back(atof(argv[6]));

	point.accelerations.push_back(atof(argv[7]));
	point.accelerations.push_back(atof(argv[8]));
	point.accelerations.push_back(atof(argv[9]));
	
	point.time_from_start = ros::Duration(1);
	
	msg.points.push_back(point);
	
	_trajectoryPublisher.publish(msg);	
}
	
	
int main(int argc,char** argv) {
	
	
	ros::init(argc,argv,"stabilizer");
	
	Stabilizer stabilizer;
	
	ros::Duration(2).sleep();
	
	stabilizer.publishTrajectory(argv);
	
//~ 	ros::spin();
}

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

int main(int argc,char** argv){
	
	ros::init(argc,argv,"velocity_publisher");
	
	ros::Publisher velocityPublisher = ros::NodeHandle().advertise<geometry_msgs::Twist>("/gz/cmd_vel",1000);
	
	geometry_msgs::Twist twist ;
	
	twist.linear.x = atof(argv[1]);
	twist.linear.y = 0;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = atof(argv[2]);
	
	
	
	while(ros::ok()){
	
		ros::Duration(1).sleep();
		velocityPublisher.publish(twist);
		
		printf("Publishing : %f , %f \n", twist.linear.x, twist.angular.z );
	}
}

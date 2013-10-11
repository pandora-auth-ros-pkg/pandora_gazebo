#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_gazebo_plugins')
import rospy
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
	
	rospy.init_node('stabilizer')
	
	pub = rospy.Publisher('/joints', JointTrajectory)
	
	msg = JointTrajectory()
	
	msg.header.stamp = rospy.get_rostime()
	msg.header.frame_id = 'chassis'
	
	msg.joint_names.append('hokuyo_joint')
	
	point = JointTrajectoryPoint()

	point.positions = (0, 0, 0, 0, 0, 1)
	point.velocities = (0, 0, 0, 0, 0, 1)
	point.accelerations = (0, 0, 0, 0, 0, 1)
	
	point.time_from_start = rospy.Duration(1)

	msg.points.append(point)
	
	#~ while True:
		#~ pub.publish(msg)
		
	pub.publish(msg)
	pub.publish(msg)

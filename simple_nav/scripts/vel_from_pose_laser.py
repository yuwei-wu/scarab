#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft
import sensor_msgs.msg
import smach

class MoveToGoal(object):
    def __init__(self, name):
        rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.goal_cb)
        rospy.Subscriber(name+"/pose", geometry_msgs.msg.PoseStamped, self.pose_cb)
	rospy.Subscriber(name+"/scan_throttle",sensor_msgs.msg.LaserScan,self.laser_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._goal_available = False
	self._angle=0
	self._angle_b=0

    def laser_cb(self, data):
	ranges=np.array(data.ranges)
	i=480
	while i<=640:
	  if ranges[i]<0.3:
		if i<560: 
		  self._angle_b=-0.785
	  	else:
		   self._angle_b=0.785
	  	rospy.loginfo("avoiding obstacle")
	  i+=1 

class moving_to_goal(smach.State):
    def _init_(self, name):
	smach.State._init_(self,outcomes=['moving forward','turning only'])
 	rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.goal_cb)
        rospy.Subscriber(name+"/pose", geometry_msgs.msg.PoseStamped, self.pose_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._goal_available = False

    def goal_cb(self,data):
   	x = data.pose.position.x
        y = data.pose.position.y
        quat = tft.numpy.array([data.pose.orientation.x,
                                data.pose.orientation.y,
                                data.pose.orientation.z,
                                data.pose.orientation.w])
        eul = tft.euler_from_quaternion(quat)
        angle = eul[-1]  # angle around z from quaternion
  	

        if self._goal_available:
            cmd = geometry_msgs.msg.Twist()
	    dist=np.sqrt((self._goal_x-x)*(self._goal_x-x)+(self._goal_y-y)*(self._goal_y-y))
	    #rospy.loginfo("obtained distance: %s", dist)
	    if self.goal_reached(dist):
	    	self._goal_available=False
	    	cmd.linear.x=0
		cmd.angular.z=0
		rospy.loginfo("goal reached")
 		self._cmd_pub.publish(cmd)
	    else:
		angle_a=np.arctan2((self._goal_y-y),(self._goal_x-x))
		angle_b=angle_a-angle
		self.laser_cb(data)
		cmd.angular.z=np.clip(0.3*angle_b,-0.5,0.5)
		if abs(angle_b)<0.2:
			return 'moving forward'
			#cmd.linear.x=np.clip(0.2*dist,-0.5,0.5)
			cmd.linear.x=0.2
			cmd.angular.z=0
		else:
			cmd.linear.x=0
			return 'turning only'	
		


	


            self._cmd_pub.publish(cmd)
            # rospy.loginfo("publishing vel_cmd: %s" % cmd)

    def goal_reached(self, dist):
	if dist<0.1:
		return True
	else: 
		return False

    def goal_cb(self, data):
        self._goal_x = data.pose.position.x
        self._goal_y = data.pose.position.y
        rospy.loginfo("obtained goal! (%s, %s)", self._goal_x, self._goal_y)
        self._goal_available = True

class avoiding_obstacle(smach.State):

    def _init_(self, name):
	smach.State._init_(self,outcomes=['moving away',])
	rospy.Subscriber(name+"/scan_throttle",sensor_msgs.msg.LaserScan,self.laser_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    def laser_cb(data):
	ranges=np.array(data.ranges)
	i=480
	cmd.linear.x=0.2
	while i<=640:
	  if ranges[i]<0.3:
		if i<560: 
		  cmd.angular.z=np.clip(0.3*0.524,-0.5,0.5)
	  	else:
		   cmd.angular.z=np.clip(0.3*-0.524,-0.5,0.5)
	  	return 'moving away'
	
	  i+=1 
def main():
  
def run(name):
    rospy.init_node('vel_from_pose')
    mtg = MoveToGoal(name)
    rospy.spin()



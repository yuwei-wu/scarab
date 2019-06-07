#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft
import sensor_msgs.msg


class MoveToGoal(object):
    def __init__(self, name):
        rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.goal_cb)
        rospy.Subscriber(name+"/pose", geometry_msgs.msg.PoseStamped, self.pose_cb)
	rospy.Subscriber(name+"/scan_throttle",sensor_msgs.msg.LaserScan,self.laser_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._goal_available = False
	self._angle_b=0
	self._obstacle = False
	self._obstacle_p=0
	#self._temporary_goal=False
	#self._t_goal=0

    def pose_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
  	quat = tft.numpy.array([data.pose.orientation.x,
                                data.pose.orientation.y,
                                data.pose.orientation.z,
                                data.pose.orientation.w])
        eul = tft.euler_from_quaternion(quat)
        self._angle = eul[-1]

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
		self._angle_b=angle_a-self._angle
		cmd.angular.z=np.clip(0.3*self._angle_b,-0.5,0.5)
		#cmd.angular.z=0
		if self._obstacle:
		  #rospy.loginfo("this range is at %s with range %s",i,ranges[i])
		  rospy.loginfo("avoiding obstacle")
		  cmd.linear.x=0.2
		  rospy.loginfo(cmd.linear.x)
		  if self._obstacle_p==0: 
		 	 self._angle_b=0.8/0.3
		  	 rospy.loginfo("turning left")

	  	  else:	
		    	self._angle_b=-0.8/0.3
		     	rospy.loginfo("turning right")
		  cmd.angular.z=np.clip(0.3*self._angle_b,-0.4,0.4)
	  	else:
		  rospy.loginfo("no obstacle")
		
		  if abs(self._angle_b)<0.2:
			rospy.loginfo("moving forward")
			#cmd.linear.x=np.clip(0.2*dist,-0.5,0.5)
			cmd.linear.x=0.2
			cmd.angular.z=0
			#cmd.linear.x=0
		  else:
			cmd.linear.x=0
			rospy.loginfo("turning only")
			rospy.loginfo(self._angle_b)


	


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

    def laser_cb(self, data):
	ranges=np.array(data.ranges)
	i=400
	while i<=560:
	  if ranges[i]<=0.8 and ranges[i]>0.02:
		self._obstacle=True
		if i<=480:
			self._obstacle_p=1
		else:
			self._obstacle_p=0
	  i+=1
	'''if self._obstacle:
	  rospy.loginfo("obstacle detected")
	else:
	  rospy.loginfo("obstacle not detected")

	
    def laser_cb(self, data)
	ranges=np.array(data.ranges)
	i=480
	while i<=640:
	if ranges[i]<=0.8:
		self._temporary_goal=True
		self_t_goal
'''

def run(name):
    rospy.init_node('vel_from_pose')
    mtg = MoveToGoal(name)
    rospy.spin()

if __name__ == '__main__':
    name = 'scarab45'
    run(name)

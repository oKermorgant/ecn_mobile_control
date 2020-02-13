#!/usr/bin/env python

# modules
# ROS stuff and multithreading
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf
import numpy as np
import sys

def bound(v, low, up):
    if v < low:
        return low
    if v > up:
        return up
    return v
    

class Robot:
    def __init__(self, bike):
        self.bike = bike
        
        self.odom = Odometry()
        self.odom.pose.pose.orientation.w = 1.
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        
        if bike:
            self.vmax = 3.
            self.bdotmax = 1.
            self.bmax = 1.
            self.L = 1.6
            self.r = 0.4
            
            self.bdot = 0.
            
            self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            self.joint_msg = JointState()
            self.joint_msg.name = ['frame_to_handlebar','handlebar_to_frontwheel','frame_to_backwheel']
            self.joint_msg.position = [0,0,0]
    
        else:       
            self.vmax = 3.
            self.wmax = 1.5
            
        self.cmd = Twist()
        rospy.Subscriber('cmd', Twist, self.cmd_callback)       
        
        self.br = tf.TransformBroadcaster()
        

    def update(self, dt):
        
        theta = 2*np.arctan2(self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        
        if bike:
            beta = self.joint_msg.position[0]
                        
            self.odom.pose.pose.position.x += self.odom.twist.twist.linear.x*np.cos(theta)*np.cos(beta)*dt
            self.odom.pose.pose.position.y += self.odom.twist.twist.linear.x*np.sin(theta)*np.cos(beta)*dt
            
            self.odom.twist.twist.angular.z = self.odom.twist.twist.linear.x*np.sin(beta)/self.L
            
            # steering update
            self.joint_msg.position[0] = bound(beta + self.bdot*dt, -self.bmax, self.bmax)
            
            
            for i in [1,2]:
                self.joint_msg.position[i] += self.odom.twist.twist.linear.x*dt/self.r
                #self.joint_msg.position[i] = self.joint_msg.position[i] % 2*np.pi
                
            self.joint_msg.header.stamp = rospy.Time.now()
            self.joint_pub.publish(self.joint_msg)
        else:
            self.odom.pose.pose.position.x += self.odom.twist.twist.linear.x*np.cos(theta)*dt
            self.odom.pose.pose.position.y += self.odom.twist.twist.linear.x*np.sin(theta)*dt
            
        
        theta += self.odom.twist.twist.angular.z * dt
        self.odom.pose.pose.orientation.z = np.sin(theta/2)
        self.odom.pose.pose.orientation.w = np.cos(theta/2)
                
        # publish pose as TF
        self.br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, theta),
                        rospy.Time.now(),
                        "footprint",
                        "world")
        
        self.odom_pub.publish(self.odom)  
        
            
    def cmd_callback(self,cmd):
        # saturate received velocities        
        v = bound(cmd.linear.x, -self.vmax, self.vmax)
        
        if bike:
            self.bdot = bound(cmd.angular.z, -self.bdotmax, self.bdotmax)
        else:
            self.odom.twist.twist.angular.z = bound(cmd.angular.z, -self.wmax, self.wmax)   
            
        self.odom.twist.twist.linear.x = v


if __name__ == "__main__":
    
    rospy.init_node('sim')   
    
    bike = sys.argv[1] == 'bike'
    
    robot = Robot(bike)

    dt = 0.01
    rate = rospy.Rate(1./dt)
    
    while not rospy.is_shutdown():

        robot.update(dt)        
        rate.sleep()
    

#!/usr/bin/env python

# modules
# ROS stuff and multithreading
import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
import tf
import numpy as np
import sys
from dynamic_reconfigure.server import Server as DRServer
from ecn_mobile_control.cfg import GainsConfig


class dictToNamespace(object):
  def __init__(self, adict):
    self.__dict__.update(adict)
    
def is_defined(v):
    return type(v) != type(None)

def toPi(v):
    return (v + np.pi) % (2*np.pi) - np.pi

class Traj:
    def __init__(self):
        self.w = .5

    def ref(self, t):
        a = 3
        b = 2
        c,s = np.cos(self.w*t),np.sin(self.w*t)
        
        x = (a + b*c)*c
        y = (a + b*c)*s
        
        vx = -self.w*(a + 2*b*c)*s
        vy = self.w*(a*c - 2*b*s**2 + b)
        
        ax = self.w**2*(-a*c + 4*b*s**2 - 2*b)
        ay = -self.w**2*(a + 4*b*c)*s
        
        return [np.matrix(val).T for val in [[x,y],[vx,vy],[ax,ay]]]
    

class Robot:
    def __init__(self, bike):
 
        self.xy = np.matrix([[0.],[0.]])
        self.theta = 0
        self.v = 0
        self.w = 0
        self.bike = bike == True
        
        # control gains
        self.gains = None
        self.srv = DRServer(GainsConfig, self.gains_cb)
        self.orient = 1
        
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        
        self.carrot_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.js = JointState()
        self.js.name=['carrot', 'carrot_disable']
        self.js.position=[0, 0]
        
        if bike:
            rospy.Subscriber('joint_states', JointState, self.joint_cb)
            self.beta = 0.
            self.L = 1.57
            
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'world'
        self.manual_goal = None
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.t0 = 0
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        
        self.cmd = Twist()
        self.cmd_pub = rospy.Publisher('cmd', Twist, queue_size=10)
        
        self.error = Float32MultiArray()
        self.error.data = [0,0,0,0]
        self.error_pub = rospy.Publisher('error', Float32MultiArray, queue_size=10)
        
        
    def gains_cb(self, config, level):
        self.gains = dictToNamespace(config)
        return config
    
    def odom_cb(self, msg):
        self.xy[0] = msg.pose.pose.position.x
        self.xy[1] = msg.pose.pose.position.y
        self.theta = 2*np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        
    def carrot(self, d):
        self.js.header.stamp = rospy.Time.now()
        self.js.position = d
        self.carrot_pub.publish(self.js)
            
    def joint_cb(self, msg):
        if 'frame_to_handlebar' in msg.name:
            idx = msg.name.index('frame_to_handlebar')
            self.beta = msg.position[idx]
                
    def goal_cb(self, msg):
        self.manual_goal = np.matrix([[msg.pose.position.x], [msg.pose.position.y]])
        self.goal = msg
        
    def reach(self, goal, xyd = np.matrix([[0,0]]).T):
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        d = self.gains.d
        
        self.carrot([d, 0])
                
        self.error.data = [goal[0,0] - self.xy[0,0], 
                    goal[1,0] - self.xy[1,0],
                    0]
        
        if self.bike:
            ctb = np.cos(self.theta+self.beta)
            stb = np.sin(self.theta + self.beta)
            sb = np.sin(self.beta)
            xyp = self.xy + self.L*np.matrix([[c],[s]]) + d*np.matrix([[ctb],[stb]])

            K = np.matrix([[ctb-d/self.L*stb*sb, -d*stb],
                           [stb+d/self.L*ctb*sb, d*ctb]])
        else:
            xyp = self.xy + d*np.matrix([[c],[s]])
            K = np.matrix([[c, -d*s],[s, d*c]])     
                                    
        xyd_cmd = self.gains.Kp*(goal-xyp) + xyd
                
        cmd = np.linalg.inv(K) * xyd_cmd
        
        self.cmd.linear.x = cmd[0,0]
        self.cmd.angular.z = cmd[1,0]   # beta dot in case of bike
        
        return np.linalg.norm(goal - xyp) < 1e-3  
            
    def move(self, t, traj):
        
        if type(self.manual_goal) != type(None):
                        
            # follow goal
            if self.reach(self.manual_goal):        # reached
                if self.t0 == 0:
                    self.t0 = t
                if t - self.t0 > 5.:   # go back to traj after 5 sec
                    self.t0 = 0
                    self.manual_goal = None
        else:                 
            xy,xyd,xydd = traj.ref(t)
            # publish goal for visualization
            self.goal.pose.position.x = xy[0]
            self.goal.pose.position.y = xy[1]
            theta_goal = np.arctan2(xyd[1], xyd[0])
            self.goal.pose.orientation.z = np.sin(theta_goal/2)
            self.goal.pose.orientation.w = np.cos(theta_goal/2)
                        
            if not self.gains.lyapunov:                
                self.reach(xy, xyd)                
            else: 
                self.carrot([0, 99])
                # Lyapunov
                c = np.cos(self.theta)
                s = np.sin(self.theta)
                Kx = self.gains.Kx
                Ky = self.gains.Ky
                Kt = self.gains.Kt
                
                vref = c*xyd[0] + s*xyd[1]
                if abs(vref) > 1e-3:
                    wref = (xyd[0]*xydd[1] - xyd[1]*xydd[0])/vref**2;
                else:
                    wref = 0
                
                # local error
                L = self.bike and self.L or 0
                beta = self.bike and self.beta or 0
                
                self.error.data = [xy[0,0] - self.xy[0,0], 
                               xy[1,0] - self.xy[1,0],
                               toPi(theta_goal - self.theta)]

                xy_err = xy - self.xy #- L*np.matrix([[c],[s]])
                xe = (np.matrix([[c,s]]) * xy_err)[0,0]
                ye = (np.matrix([[-s,c]]) * xy_err)[0,0]
                te = toPi(theta_goal - self.theta - beta)
                
                self.cmd.linear.x = vref*np.cos(te) + Kx*xe
                self.cmd.angular.z = wref + Ky*ye*vref*np.sinc(te) + Kt*te
                
                if self.bike:
                    self.cmd.angular.z -= self.cmd.linear.x/L*np.sin(beta)
                
        self.cmd_pub.publish(self.cmd)   
        self.goal_pub.publish(self.goal)
        self.error.data.append(np.linalg.norm(self.error.data))
        self.error_pub.publish(self.error)           
        

if __name__ == "__main__":
    rospy.init_node('control')
    
    rate = rospy.Rate(1.)
    while not rospy.has_param('robot'):
        rate.sleep()
    
    robot = Robot(rospy.get_param('robot'))
    traj = Traj()
        
    # build path
    path = Path() 
    path.header.frame_id = 'world' 
    for t in np.linspace(-np.pi/traj.w, np.pi/traj.w, int(100/traj.w)):
        pose = PoseStamped()
        pose.pose.orientation.w = 1
            
        path.poses.append(pose)
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    
        
    rate = rospy.Rate(20.)
    

    while not rospy.is_shutdown():
        path_pub.publish(path)
        
        robot.move(rospy.Time.now().to_sec(), traj)
        rate.sleep()
    

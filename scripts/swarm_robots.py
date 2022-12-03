#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from utils.behaviors import Behaviors
from utils.obstacle_avoidance import ObstacleAvoidance
from utils.roaming import Roaming
class SwaromRobots:

    # SwaromRobots Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, radius):

        # ATTRIBUTES
        self.num_robots = rospy.get_param("/num_of_robots")
        self.ns_prefix = rospy.get_param("/robot_name")
        self.odom = np.zeros((self.num_robots, 3))
        self.cmd_vel = np.zeros((self.num_robots, 3))

        self.coeff_sep = rospy.get_param("~coeff_sep")
        self.coeff_coh = rospy.get_param("~coeff_coh")
        self.coeff_ali = rospy.get_param("~coeff_ali")

        self.slowing_distance = 0.5
        self.slowing_speed = 0.1

        self.cmd_pub = [None] * self.num_robots

        self.odom_sub =  [None] * self.num_robots

        self.pose = np.zeros((self.num_robots, 3))
        self.vel = np.zeros((self.num_robots, 3))

        # change dt at a later time
        self.dt = 0.1

        # set maximum values
        self.max_acc = rospy.get_param("~max_acc")
        self.max_vel = rospy.get_param("~max_vel")

        self.fov = rospy.get_param("~fov")
        self.max_see_ahead = rospy.get_param("~max_see_ahead")

        self.oa = ObstacleAvoidance(r=self.num_robots, fov=self.fov, max_see_ahead=self.max_see_ahead)

        self.behaviors = Behaviors(self.num_robots, self.max_acc, self.max_vel)

        self.roaming = Roaming(self.max_vel, self.slowing_speed, self.slowing_distance)
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.dominion = dominion                                        

        # CONTROLLER PARAMETERS                
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.3                

        for r in range(self.num_robots):
            self.cmd_pub[r] = rospy.Publisher(self.ns_prefix+"_{}".format(str(r))+cmd_vel_topic, Twist, queue_size=10) #  publisher to cmd_vel_topic
            self.odom_sub[r] = rospy.Subscriber(self.ns_prefix+"_{}".format(str(r))+odom_topic, Odometry, self.odom_callback, (r)) # : subscriber to odom_topic  

        
        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)  # : subscriber to gridmap_topic from Octomap Server  
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal) # : subscriber to /move_base_simple/goal published by rviz    
        
        # TIMERS
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.05), self.controller)
    
    # Odometry callback: Gets current robot pose and stores it into self.pose
    def odom_callback(self, odom, r):
        """ Odometry callback """
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.pose[r, 0] = odom.pose.pose.position.x
        self.pose[r, 1] = odom.pose.pose.position.y
        self.pose[r, 2] = yaw
        
    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        """
        Callback for the goal position subscriber for swarm destination
        """

        if self.oa.there_is_map:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
             
        
    def get_gridmap(self, gridmap):
        """
        Callback for the map topic. Updates the map array with the new map.
        """
        # to avoid map update too often (change value if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 0.5:            
            self.last_map_time = gridmap.header.stamp

            # Update 
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.oa.set_map(env, gridmap.info.resolution, origin)



    def controller(self, event):
        """
        Velocity controller

        """        
        if self.goal is not None:
            # self.vel[:,0:2] = self.roaming.seek(self.pose[:,0:2], self.goal, self.vel[:,0:2]) * self.dt
            self.vel[:,0:2] = self.roaming.arrival(self.pose[:,0:2], self.goal, self.vel[:,0:2], self.slowing_speed, self.slowing_distance) * self.dt
        # start behaviors
        self.combined_acc = self.behaviors.seperation(self.pose, self.coeff_sep) + self.behaviors.cohesion(self.pose, self.coeff_coh) + self.behaviors.alignment(self.vel, self.coeff_ali)
        self.vel = self.combined_acc * self.dt + self.vel
        
        self.vel_norm = self.dt * self.vel#/np.linalg.norm(self.vel, axis=1).reshape(self.num_robots,1) if np.linalg.norm(self.vel) > self.max_vel else self.vel
        avoidance_force = self.oa.look_ahead(self.pose[:,0:2], self.vel_norm)

        self.vel[:,0:2] = self.vel[:,0:2] + avoidance_force
        for r in range(self.num_robots):
            v = self.vel[r,0]
            u = self.vel[r,1]
            w = self.vel[r,2]
            # Publish velocity commands
            self.__send_commnd__(v, u, w, r)
    
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, u, w, r):
        """
        Publishes the velocity commands to the robots
        """
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = np.clip(u, -self.v_max, self.v_max)
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub[r].publish(cmd)

if __name__ == '__main__':
    rospy.init_node('swarm_robots')   
    node = SwaromRobots('/map', '/odom', '/cmd_vel', np.array([-15.0, 15.0]), 0.3)
    
    # Run forever
    rospy.spin()
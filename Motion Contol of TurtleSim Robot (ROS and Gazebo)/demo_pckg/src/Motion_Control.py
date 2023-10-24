#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill , SetPen


class Turtlesim:
        
    def __init__(self):

        rospy.init_node('turtle_move', anonymous=True)

        kill1 = rospy.ServiceProxy('kill',Kill)
        kill1('turtle1')
        
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(1,2,0, "turtle1")
        rospy.wait_for_service('turtle1/set_pen')
        turtle1_setPen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
        turtle1_setPen(0,255,0,2,0)

        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(5,2,0, "turtle2") 
        rospy.wait_for_service('turtle2/set_pen')
        turtle2_setPen = rospy.ServiceProxy('turtle2/set_pen', SetPen)
        turtle2_setPen(255,255,255,2,0)
        
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(8,4,0, "turtle3")
        rospy.wait_for_service('turtle3/set_pen')
        turtle3_setPen = rospy.ServiceProxy('turtle3/set_pen', SetPen)
        turtle3_setPen(255,0,0,2,0)

        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(8.2,5,0, "turtle4")
        rospy.wait_for_service('turtle4/set_pen')
        turtle4_setPen = rospy.ServiceProxy('turtle4/set_pen', SetPen)
        turtle4_setPen(255,0,0,2,0)
        # Publisher nodes
        
        #Turtle_a
        self.velocity_publisher1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        #Turtle_l
        self.velocity_publisher2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        #Turtle_i
        self.velocity_publisher3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
        
        #Turtle_i_dot
        self.velocity_publisher4 = rospy.Publisher('/turtle4/cmd_vel', Twist, queue_size=10)
        
        
        # Subscriber nodes
        
        #Turtle_a
        self.pose_subscriber1 = rospy.Subscriber('/turtle1/pose', Pose,
                                                        callback = self.call_function1)
        #Turtle_l
        self.pose_subscriber2 = rospy.Subscriber('/turtle2/pose', Pose,
                                                        callback = self.call_function2)
        #Turtle_i
        self.pose_subscriber3 = rospy.Subscriber('/turtle3/pose', Pose,
                                                        callback = self.call_function3)
        #Turtle_i_dot
        self.pose_subscriber4 = rospy.Subscriber('/turtle4/pose', Pose,
                                                        callback = self.call_function4)
                                                        
        self.pose1 = Pose()
        self.pose2 = Pose()
        self.pose3 = Pose()
        self.pose4 = Pose()
        self.rate = rospy.Rate(10)

    def call_function1(self, pose_data):

        self.pose1 = pose_data
        self.pose1.x = round(self.pose1.x, 4)
        self.pose1.y = round(self.pose1.y, 4)
    
    def call_function2(self, pose_data):
        
        self.pose2 = pose_data
        self.pose2.x = round(self.pose2.x, 4)
        self.pose2.y = round(self.pose2.y, 4)
    
    def call_function3(self, pose_data):
        
        self.pose3 = pose_data
        self.pose3.x = round(self.pose3.x, 4)
        self.pose3.y = round(self.pose3.y, 4)
    
    def call_function4(self, pose_data):
        
        self.pose4 = pose_data
        self.pose4.x = round(self.pose4.x, 4)
        self.pose4.y = round(self.pose4.y, 4)                
                
    def current_distance(self, goal_pose, turtle_number):

        if turtle_number == 1:
                output = math.sqrt(((goal_pose.x - self.pose1.x)**2) +
                        ((goal_pose.y - self.pose1.y)**2))
        elif turtle_number == 2:
                output = math.sqrt(((goal_pose.x - self.pose2.x)**2) +
                        ((goal_pose.y - self.pose2.y)**2))
        elif turtle_number == 3:
                output = math.sqrt(((goal_pose.x - self.pose3.x)**2) +
                        ((goal_pose.y - self.pose3.y)**2))
        elif turtle_number == 4:
                output = math.sqrt(((goal_pose.x - self.pose4.x)**2) +
                        ((goal_pose.y - self.pose4.y)**2))

        return output
        
    def linear_velicity(self, goal_pose, turtle_number, constant=1.5):

        if turtle_number == 1:
                output = constant * self.current_distance(goal_pose,1)
        elif turtle_number == 2:
                output = constant * self.current_distance(goal_pose,2)
        elif turtle_number == 3:
                output = constant * self.current_distance(goal_pose,3)
        elif turtle_number == 4:
                output = constant * self.current_distance(goal_pose,4)
        return output
    
    def angular_velocity(self, goal_pose, turtle_number, constant=8):
                
        if turtle_number == 1:
                angle1_cmd = math.atan2(goal_pose.y - self.pose1.y, goal_pose.x - self.pose1.x)
                output = constant * (angle1_cmd - self.pose1.theta)                        
        elif turtle_number == 2:
                angle2_cmd = math.atan2(goal_pose.y - self.pose2.y, goal_pose.x - self.pose2.x)
                output = constant * (angle2_cmd - self.pose2.theta)                        
        elif turtle_number == 3:
                angle3_cmd = math.atan2(goal_pose.y - self.pose3.y, goal_pose.x - self.pose3.x)
                output = constant * (angle3_cmd - self.pose3.theta)                        
        elif turtle_number == 4:
                angle4_cmd = math.atan2(goal_pose.y - self.pose4.y, goal_pose.x - self.pose4.x)
                output = constant * (angle4_cmd - self.pose4.theta)
        return output
    
    def publish_cmd(self, goal_pose, distance_tolerance, vel_msg, turtle_number, direction):

        if turtle_number == 4:

                while True:
                        vel_msg.linear.x = 1
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = 3
                        # Publishing our vel_msg
                        self.velocity_publisher4.publish(vel_msg)

                        self.rate.sleep()

                        if -0.4 < self.pose4.theta < -0.2:
                                break
        
        else:
                        
                while self.current_distance(goal_pose,turtle_number) >= float(distance_tolerance):
        
                        # Porportional controller.

                        # Linear velocity in the x-axis.
                        vel_msg.linear.x = self.linear_velicity(goal_pose, turtle_number)
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0

                        # Angular velocity in the z-axis.
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0

                        if direction == 0:
                                vel_msg.angular.z = self.angular_velocity(goal_pose, turtle_number)
                        else:
                                vel_msg.angular.z = -self.angular_velocity(goal_pose,turtle_number)
                                direction = 0


                        # Publishing vel_msg
                        if turtle_number == 1:
                                self.velocity_publisher1.publish(vel_msg)
                        elif turtle_number == 2:
                                self.velocity_publisher2.publish(vel_msg)
                        elif turtle_number == 3:
                                self.velocity_publisher3.publish(vel_msg)

                        self.rate.sleep()

    def turtle_move(self):

        turtle1_goal_pose1 = Pose()
        turtle1_goal_pose2 = Pose()
        turtle1_goal_pose3 = Pose()
        turtle1_goal_pose4 = Pose()

        turtle2_goal_pose1 = Pose()
        turtle2_goal_pose2 = Pose()
        turtle2_goal_pose3 = Pose()

        turtle3_goal_pose1 = Pose()
        turtle3_goal_pose2 = Pose()
        

        # Turtle_a targets
        turtle1_goal_pose1.x = 2.5
        turtle1_goal_pose1.y = 6

        turtle1_goal_pose2.x = 3.5
        turtle1_goal_pose2.y = 2
        
        turtle1_goal_pose3.x = 3.5
        turtle1_goal_pose3.y = 3.7
        
        turtle1_goal_pose4.x = 2
        turtle1_goal_pose4.y = 4
        
        # Turtle_l targets
        turtle2_goal_pose1.x = 6.2
        turtle2_goal_pose1.y = 6

        turtle2_goal_pose2.x = 5.8
        turtle2_goal_pose2.y = 2
        
        turtle2_goal_pose3.x = 6.4
        turtle2_goal_pose3.y = 2.4

        # Turtle_i targets
        turtle3_goal_pose1.x = 8
        turtle3_goal_pose1.y = 2

        turtle3_goal_pose2.x = 8.5
        turtle3_goal_pose2.y = 2.7

        distance_tolerance = 0.01

        turtle1_vel_msg = Twist()
        turtle2_vel_msg = Twist()
        turtle3_vel_msg = Twist()
        turtle4_vel_msg = Twist()

        self.publish_cmd(turtle1_goal_pose1, distance_tolerance, turtle1_vel_msg, 1, 0)
        self.publish_cmd(turtle1_goal_pose2, distance_tolerance, turtle1_vel_msg, 1, 0)
        self.publish_cmd(turtle1_goal_pose3, distance_tolerance, turtle1_vel_msg, 1, 0)
        self.publish_cmd(turtle1_goal_pose4, distance_tolerance, turtle1_vel_msg, 1, 0)

        self.publish_cmd(turtle2_goal_pose1, distance_tolerance, turtle2_vel_msg, 2, 0)
        self.publish_cmd(turtle2_goal_pose2, distance_tolerance, turtle2_vel_msg, 2, 1)
        self.publish_cmd(turtle2_goal_pose3, distance_tolerance, turtle2_vel_msg, 2, 0)

        self.publish_cmd(turtle3_goal_pose1, distance_tolerance, turtle3_vel_msg, 3, 0)
        self.publish_cmd(turtle3_goal_pose2, distance_tolerance, turtle3_vel_msg, 3, 0)
        
        self.publish_cmd(turtle4_vel_msg, distance_tolerance, turtle4_vel_msg, 4, 0)

        # Stopping our robot after the movement is over.
        turtle1_vel_msg.linear.x = 0
        turtle1_vel_msg.angular.z = 0
        self.velocity_publisher1.publish(turtle1_vel_msg)

        turtle2_vel_msg.linear.x = 0
        turtle2_vel_msg.angular.z = 0
        self.velocity_publisher2.publish(turtle2_vel_msg)

        turtle3_vel_msg.linear.x = 0
        turtle3_vel_msg.angular.z = 0
        self.velocity_publisher3.publish(turtle3_vel_msg)

        turtle4_vel_msg.linear.x = 0
        turtle4_vel_msg.angular.z = 0
        self.velocity_publisher4.publish(turtle4_vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()
 
if __name__ == '__main__':
         
        robot = Turtlesim()
        robot.turtle_move()
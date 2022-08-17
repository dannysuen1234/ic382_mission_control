#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0
y = 0
theta = 0
odom_msg = 0

#add 3 new variables
prev_x = 0
prev_y = 0
distance_moved = 0

#include the goal points to this list. [x, y, angle]
# define distance in meter, angle in radian
goal_list = [ [2, 0, 0], [0, 2, -1.57] ]

#tune the speed of the robot
linear_speed_straight = 0.3
linear_speed_rotation = 0.05
angular_speed_rotation = 0.3

#tune the tolerance of the robot
angle_tolerance = 0.3

#modify this function to adjust the error 
def newOdom(msg):
    global x
    global y
    global theta
    global odom_msg
  
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    odom_msg = 1

def position(x1, y1, x2, y2):
    return ((x1-x2)**2 +(y1-y2)**2)**0.5

print("start")
rospy.init_node("speed_controller")

#sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)


# include angle in the third position of the list 

curr_point = goal_list.pop(0)
goal = Point()
goal.x = curr_point[0]
goal.y = curr_point[1]
distance = (goal.x**2 + goal.y**2)**0.5

# angle to goal is the thrid value of the curr_point
angle_to_goal = curr_point[2]
if angle_to_goal == 0:
  turning = False

else:
  turning = True

while odom_msg <= 0:
    rospy.sleep(5)

print("controller start")

while not rospy.is_shutdown():

    print("odom x: %3.4f, y: %3.4f, theta: %3.4f" % (x,y,theta))

    distance_moved = ((x-prev_x)**2 + (y-prev_y)**2)**0.5
   
    if distance_moved - distance >=0 and not turning:
    
               print("arrived.............................")
               if goal_list:
                    print("next pos")
                    speed.linear.x = 0.0
                    speed.linear.y = 0.0
                    speed.linear.z = 0.0
                    speed.angular.z = 0.0
                    pub.publish(speed)
                    print("stopping the car for two seconds")
                    rospy.sleep(2)
                    distance_moved = 0
                    curr_point = goal_list.pop(0)
                    #distance = (goal.x**2 + goal.y**2)**0.5
                    angle_to_goal = curr_point[2]
                    prev_x = x
                    prev_y = y
                    turning = True

               else:
                    print("end.............................")
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    print("controller publish")
                    pub.publish(speed)
                    rospy.sleep(5)
                    break

                       

    if turning == False:
               print("go straight | pos diff %3.4f" % (position(goal.x, goal.y, x, y)))
               speed.linear.x = linear_speed_straight
               speed.angular.z = 0.0


    else:
               
               if  (theta - angle_to_goal) > angle_tolerance:
                   print("rotating clockwise")
                   speed.linear.x = linear_speed_rotation
                   speed.angular.z = -angular_speed_rotation

               elif (theta - angle_to_goal) < -angle_tolerance:
                   speed.linear.x = linear_speed_rotation
                   speed.angular.z = angular_speed_rotation
                   print("rotating anti-clockwise") 

               
               else:
                   print("go straight | pos diff %3.4f" % (position(goal.x, goal.y, x, y)))
                   turning = False
                   speed.linear.x = 0.0
                   speed.angular.z = 0.0
                   pub.publish(speed)
                   print("stopping for two seconds")
                   distance = (goal.x**2 + goal.y**2)**0.5
                   rospy.sleep(2)
                   speed.linear.x = linear_speed_straight
                   speed.angular.z = 0.0

    print("speed.linear.x", speed.linear.x, "speed.angular.z", speed.angular.z, "theta: ", theta, "angle to goal: ", angle_to_goal)
    pub.publish(speed)

    r.sleep()

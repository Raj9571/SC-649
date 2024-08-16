#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math 
import matplotlib.pyplot as plt
import time

x = 0
y = 0
phi = 0.0

def newOdom(msg):
    global x
    global y
    global phi
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, phi) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def controller():
   
    l = [0]  # l is time
    x1 = [0]  # x-coordinate of robot
    x2 = [0]  # x set point
    y1 = [0]  # y -coordinate of robot
    y2 = [0]  # y set point
    theta1 = [0]
    dis_err = [0]
    u1 = [0]
    rospy.init_node("speed_controller")
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    rate = rospy.Rate(10)
    goal = Point()
    gamma = 0.2
    h = 0.2
    const = 0.4
    k=0
	
    while not rospy.is_shutdown() and k < 500:
      
        k = k + 1
        print(k)
        goal.x = -2
        goal.y = 2
        inc_x = goal.x - x
        inc_y = goal.y - y
        l.append((k + 1) / 10)
        x1.append(x)
        y1.append(y)
        theta1.append(phi)
        x2.append(goal.x)
        y2.append(goal.y)
        angle_to_goal = math.atan2(inc_y, inc_x)
        dis_err = (inc_x**2+inc_y**2)**0.5
        if(dis_err<0.05):
           angle_to_goal=0

        alpha=angle_to_goal - phi

        
        if(alpha>=-2*math.pi and alpha<=-math.pi):
         alpha = alpha+2*math.pi
        elif(alpha>-math.pi and alpha<math.pi):
         alpha = alpha
        elif(alpha>=math.pi and alpha<=2*math.pi):
         alpha=alpha-2*math.pi
           
        #alpha = math.atan2(math.sin(angle_to_goal - phi), math.cos(angle_to_goal - phi))
        
        u = gamma * math.cos(alpha) * dis_err
        if(abs(alpha)<=0.01):
         omega=(const * alpha) + gamma * math.cos(alpha) * (alpha + (h * angle_to_goal))
        else:
         omega = (const * alpha) + gamma * (math.sin(2 * alpha) / (2 * alpha)) * (alpha + (h * angle_to_goal))
        speed.linear.x = u
        speed.angular.z = omega
        print("alpha:",alpha,"phi:",phi,"dis_err:",dis_err)
        print("U:",u,"omega:",omega,"angletogoal:",angle_to_goal)
        pub.publish(speed)
        rate.sleep()
    if (k>=500):
     plt.figure(1)
     plt.plot(x1,y1 , label='Robot position')
     plt.plot(x2,y2, label='Goal')
     plt.xlabel('x co-ordinate')
     plt.ylabel('y co-ordinate')
     plt.axis('scaled')
     plt.legend()
     plt.show()
     rospy.signal_shutdown("data collected")
    

if __name__ == '__main__':
    try:
        time.sleep(3)
        controller()
    except rospy.ROSInterruptException:
        pass

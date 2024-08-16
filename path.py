#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math 
import matplotlib.pyplot as plt
import time


def newOdom(msg):
    global x
    global y
    global phi
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, phi) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def controller():
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    rospy.init_node("speed_controller")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    rate = rospy.Rate(15)
    goal = Point()
    k = 0
    l = [0]  # l is time
    x1 = [x]  # x-coordinate of robot
    x2 = [0]  # x set point
    y1 = [y]  # y -coordinate of robot
    y2 = [0]  # y set point
    ug=[0]
    dis_err = [0]
    gamma = 0.2
    lamb=0.001
    h = 0.2
    const = 0.43
    epsilon=0.02
    s=0
    s_dot=0
    goal.x=0
    goal.y=0
    while not rospy.is_shutdown() and k < 850:
        
        k = k + 1
        print(k)
        print(x,y)
        print(goal)
        inc_x = goal.x - x
        inc_y = goal.y - y
        dis_err = (inc_x**2+inc_y**2)**0.5
        e=dis_err
        theta=math.atan(1.25*4*(goal.x**3)/(1+goal.x**8))
        angle_to_goal = math.atan2(inc_y, inc_x) - theta
        

        l.append((k + 1) / 10)
        x1.append(x)
        y1.append(y)
        x2.append(goal.x)
        y2.append(goal.y)
        
        
        if(dis_err<0.1):
            angle_to_goal=0
        alpha = (angle_to_goal - (phi-theta))     
        if(alpha>=-2*math.pi and alpha<=-math.pi):
         alpha = alpha+2*math.pi
        elif(alpha>-math.pi and alpha<math.pi):
         alpha = alpha
        elif(alpha>=math.pi and alpha<=2*math.pi):
         alpha=alpha-2*math.pi
           
        V=(lamb*e*e)+(alpha**2)+h*(angle_to_goal**2)
        print("V:",V)
        s_dot=max(0,1-(V/epsilon))
        s=s+s_dot*k*0.005
        ds=s_dot*k*0.005
        print(s_dot)
        
        dy=ds*math.sin(theta)
        dx=ds*math.cos(theta)
        print(dx,dy)
        goal.y=goal.y+dy
        goal.x=goal.x+dx
        u = gamma * math.cos(alpha) * dis_err
        ug.append(u)
        print("u:",u)
        if(abs(alpha)<=0.01):
         omega=(const * alpha) + gamma * math.cos(alpha) * (alpha + (h * angle_to_goal))
        else:
         omega = (const * alpha) + gamma * (math.sin(2 * alpha) / (2 * alpha)) * (alpha + (h * angle_to_goal))
        speed.linear.x = u
        speed.linear.x = u
        speed.angular.z = omega
        #print("alpha:",alpha,"phi:",phi,"dis_err:",dis_err)
        #print("U:",u,"omega:",omega,"angletogoal:",angle_to_goal)
        pub.publish(speed)
        rate.sleep()
    if (k>=850):
     plt.figure(1)
     plt.plot(x1,y1 , label='Robot position')
     plt.plot(x2,y2, 'r--', label='Goal')
     plt.plot(x1,ug,'y--',label='u')

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

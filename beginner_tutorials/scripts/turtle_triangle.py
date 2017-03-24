#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

theta=0.0


def pose_callback(pose):
    theta= pose.theta
    print "theta = "+str(theta)
    
    
def turtle_triangle():
    x_vel=2.1
    rospy.init_node('turtle_triangle', anonymous=True)
    
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    rate_straight = rospy.Rate(1)
    rate_turn = rospy.Rate(1)
    vel=Twist()
    while not rospy.is_shutdown():
        #first line
        
        if vel.linear.x==x_vel:
            vel.linear.x=0
            vel.angular.z=2.07
        else:
            vel.linear.x=x_vel
            vel.angular.z=0
        
        pub.publish(vel)
        rate_straight.sleep()
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_triangle()
    except rospy.ROSInterruptException:
        pass

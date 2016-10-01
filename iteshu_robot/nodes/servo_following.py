#!/usr/bin/env python
"""
servo_following.py
 This program will read the position of a user given by openni_tracker
 then it will move the kinect by using the servo motor connected to an arduino board
 It relies on openni_tracker and serial_node.py nodes, so be sure you 
 launch them first.
maintainer: arturoescobedo.iq@gmail.com
"""
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from copy import deepcopy 
import numpy as np

class TrackedPerson():
    def __init__( self ):
        """ Constants
        """
        
        """Members 
        """ 
        self.last_angle_from_kinect = 0
        self.current_angle_from_kinect = 1
        self.person_number = 1 #Number of the user being tracked
        """ Publishers
        """
        
        """ Subscribers
        """
        self.__listener = tf.TransformListener() #NOTE THIS: the listener should be declared in the class

    def get_angle_from_kinect(self):
        """ Get the position (angle) of the tracked user with respect to the kinect """
        try:
            now = rospy.Time(0)
            (trans, rot) = self.__listener.lookupTransform( 'openni_depth_frame', 'head_'+str(self.person_number),  now)
        except ( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException ):
            pass
        self.last_angle_from_kinect = self.current_angle_from_kinect
        self.current_angle_from_kinect = np.rad2deg(np.arctan2( trans[1] , trans[0] )) #Angle of the user with respect to the kinect
        return self.current_angle_from_kinect
    
    def loop_exists(self):
        if self.current_angle_from_kinect == self.last_angle_from_kinect:
            return True
        else:
            return False
        
        



class Servo():
    def __init__( self ):
        """ Constants
        """
        self.C_TIME_FOR_ANGLE_INCREMENT = 0.15 #time in seconds
        self.C_ANGLE_INCREMENT= 1 #Value of desired angle steps to move the servo (It is related to the resolution of the servo) mine is 10 degrees.
        self.C_INIT_ANGLE = 90 #Servo starting position 90 -> to the front, 0->right, 180 left.
        """For PID controller """
        self.dt = 0.1 #Period of time to change the min resolution angle of the servo
        self.previous_angle_error = 0.0
        self.integral_error = 0
        self.derivative = 0
        self.Kp = 0.2  #Best tuned params I have found are kp 0.13, ki 0.17, and kd 0.025
        self.Ki = 0.01
        self.Kd = 0.0
        """Members 
        """ 
        self.cant_move_more_flag=0
        self.angle = UInt16() #Values 0 "right"  - 180 "left", The angle where we want the serve moves to 
        """Publishers
        """
        self.angle_pub = rospy.Publisher( 'arduino/servo', UInt16, queue_size=10 ) #Should have values between 0 and 180
        self.servo_tf_br = tf.TransformBroadcaster()
        """Subscribers
        """
        
    def move_angle(self, angle):
        """ angle: Is the angle between the tracked person and kinect
        """
        print angle
        self.pid_control(angle)
            
    
    def pid_control(self, angle_error):
        self.angle_error = angle_error
        rospy.sleep(rospy.Duration.from_sec(self.dt))
        self.integral_error = self.integral_error + (self.angle_error*self.dt)
        self.derivative_error = (self.angle_error - self.previous_angle_error)/self.dt
        self.pid_output = (self.Kp*self.angle_error) + (self.Ki*self.integral_error) + (self.Kd*self.derivative_error)
        self.previous_angle_error = self.angle_error
        self.angle += self.pid_output
        
        if self.angle >= 0 and self.angle <= 180:
            self.angle_pub.publish(self.angle)  
        else:
            self.cant_move_more_flag=1
            rospy.loginfo("Servo can't move more")
            self.previous_angle_error=0
            self.integral_error=0
            self.derivative_error=0
            self.angle_error=0
            self.go_home()
         

        
        
        
    def move_right(self):
            self.angle -= self.C_ANGLE_INCREMENT
            self.angle_pub.publish(self.angle)

    
    def move_left(self):   
            self.angle += self.C_ANGLE_INCREMENT
            self.angle_pub.publish(self.angle)


    def go_home(self):
        self.cant_move_more_flag=0
        self.angle_pub.publish(self.C_INIT_ANGLE)
        self.angle = self.C_INIT_ANGLE
        
        
class ServoFollowing():
    def __init__( self ):
        """ Constants
        """
        
        """Members 
        """ 
        self.servo = Servo()
        self.user = TrackedPerson()
        """ Publishers
        """
 
        rospy.on_shutdown( self.cleanup )
        
        self.cleanup()
        
        r = rospy.Rate( 10.0 )
        
        while not rospy.is_shutdown():
            try:
                angle = self.user.get_angle_from_kinect()  
                print "Angle between tracked person"+str(self.user.person_number)+ " and kinect:"                
                print angle
            except:
                print "Cannot find the tracked person "+str(self.user.person_number)
                #If there i no user (i) look for another user.
                self.look_for_another_user()
                rospy.sleep(0.1)#time in seconds
                continue
            if self.user.loop_exists():
                print "Loop Exists on user "+str(self.user.person_number)
                self.servo.go_home()
                #If there i no user (i) look for another user.
                self.look_for_another_user()
                print "I will try with user "+str(self.user.person_number)
                rospy.sleep(2)
                #continue
            self.servo.move_angle(angle)
            r.sleep()
     
    def look_for_another_user(self):
        if self.user.person_number<15: #15 is the maximum number of users generated by the kinect tracker
            self.user.person_number+=1
        else:
            self.user.person_number=1       
    
    
    def cleanup(self):
        print 'Go home'
        self.servo.go_home()
    
        
    

if __name__ == "__main__":
    rospy.init_node( 'servo_following' )
    try:
        ServoFollowing()
    except:
        rospy.logfatal("servo_following.py controller died")
        pass

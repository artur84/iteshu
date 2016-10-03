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
        self.number = 1 #Number of the user being tracked
        self.frame_root="head_" #frame root name beeing tracked it can be torso_, head_ etc.
        self.current_name=self.frame_root+str(self.number) #Frame beeing tracked root+number
        self.last_name=self.frame_root+str(self.number)
        self.available_list=[] #The list containing all the available users in the scene 
        """ Publishers
        """
        
        """ Subscribers
        """
        self.__listener = tf.TransformListener() #NOTE THIS: the listener should be declared in the class

    def get_angle_from_kinect(self):
        """ Get the position (angle) of the tracked user with respect to the kinect """
        self.current_name = deepcopy(self.available_list[0]) #Takes the first user in the list
        #print self.current_name
        try:#Check if the desired tf is active
            self.__listener.waitForTransform('openni_depth_frame', self.current_name,  rospy.Time.now(), rospy.Duration(0.05))
        except:
            raise
        
        try:#Compute the desired tf
            now = rospy.Time(0)
            (trans, rot) = self.__listener.lookupTransform( 'openni_depth_frame', self.current_name,  now)  
        except ( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException ):
            raise
        self.last_name = deepcopy(self.current_name)
        
        self.last_angle_from_kinect = self.current_angle_from_kinect
        self.current_angle_from_kinect = np.rad2deg(np.arctan2( trans[1] , trans[0] )) #Angle of the user with respect to the kinect
        return self.current_angle_from_kinect
       
    def get_available_list(self):
        """ Gets the list of all available frames 
        """
        self.available_list=[]
        for n in range(8):
            name = self.frame_root+str(n)
            if self.__listener.frameExists(name):
                try:
                    self.__listener.waitForTransform('openni_depth_frame', name,  rospy.Time.now(), rospy.Duration(0.05))
                    self.available_list.append(name)
                except:
                    continue
                
        return self.available_list
            
    def loop_exists(self):
        if self.current_angle_from_kinect == self.last_angle_from_kinect:
            return True
        else:
            return False
        
    def look_for_another_user(self):
        if self.number<15: #15 is the maximum number of users generated by the kinect tracker
            self.number+=1
        else:
            self.number=1  
        
        



class Servo():
    def __init__( self ):
        """ Constants
        """
        self.C_TIME_FOR_ANGLE_INCREMENT = 0.15 #time in seconds
        self.C_ANGLE_INCREMENT= 1 #Value of desired angle steps to move the servo (It is related to the resolution of the servo) mine is 10 degrees.
        self.C_INIT_ANGLE = 110 #!!!!!!!IT MUST BE EQUAL TO THE ONE OF THE ARDUINO!!!!!!!!!
        """For PID controller """
        self.dt = 0.1 #Period of time to change the min resolution angle of the servo
        self.previous_p_error = 0.0
        self.p_error=0.0
        self.integral_error = 0.0
        self.derivative_error = 0.0
        self.Kp = 0.3  #Best tuned params I have found are kp 0.13, ki 0.17, and kd 0.025
        self.Ki = 0.01
        self.Kd = 0.0
        """Members 
        """ 
        self.cant_move_more_flag=0
        self.ros_angle = UInt16() #It will be initialized in go_home Values 0 "right"  - 180 "left", The angle where we want the serve moves to 
        self.angle=self.C_INIT_ANGLE #It will be initialized in go_home Values 0 "right
        """Publishers
        """
        self.angle_pub = rospy.Publisher( 'arduino/servo', UInt16, queue_size=10 ) #Should have values between 0 and 180
        self.servo_tf_br = tf.TransformBroadcaster()
        """Subscribers
        """
        
    def move_angle(self, angle):
        """ angle: Is the angle between the tracked person and kinect
        """
        #print angle
        self.pid_control(angle)
            
    
    def pid_control(self, angle_error):
        self.p_error = angle_error
        rospy.sleep(rospy.Duration.from_sec(self.dt))
        self.integral_error = self.integral_error + (self.p_error*self.dt)
        self.derivative_error = (self.p_error - self.previous_p_error)/self.dt
        self.pid_output = (self.Kp*self.p_error) + (self.Ki*self.integral_error) + (self.Kd*self.derivative_error)
        self.previous_p_error = self.p_error
        self.angle += self.pid_output
        
        if self.angle >= 0 and self.angle <= 180:
            self.ros_angle.data=self.angle
            print self.angle
            self.angle_pub.publish(self.ros_angle)  
        else:
            self.cant_move_more_flag=1
            rospy.loginfo("Servo can't move more")
            self.previous_p_error=0
            self.integral_error=0
            self.derivative_error=0
            self.p_error=0
            self.go_home()

    def go_home(self):
        self.ros_angle.data=self.C_INIT_ANGLE
        self.angle=self.C_INIT_ANGLE
        self.cant_move_more_flag=0
        self.angle_pub.publish(self.ros_angle)
        
        
class ServoFollowing():
    def __init__( self ):
        """ Constants
        """
        
        """Members 
        """ 
        self.servo = Servo()
        self.user = TrackedPerson()
        self.current_amount_of_users=0
        self.last_amount_of_users=0
        self.first_loop_flag=1
        """ Publishers
        """
 
        rospy.on_shutdown( self.cleanup )
        
        self.cleanup()
        
        r = rospy.Rate( 10.0 )
        list=[]
        while not rospy.is_shutdown():
            #print self.user.current_name
            #print self.user.last_name
            self.user.get_available_list()
            self.current_amount_of_users= len(self.user.available_list)
            ####Say hello when new user enters
            if self.current_amount_of_users>self.last_amount_of_users:
                print "New user entered"
                print "current users:"
                print self.user.available_list
            elif self.current_amount_of_users <self.last_amount_of_users:
                print "A user got out"
                print "current users:"
                print self.user.available_list
            else:
                pass
            #### Track users only if there are more than 0
            if self.current_amount_of_users>0: #There are available users
                try:
                    angle = self.user.get_angle_from_kinect()      
                except:
                    continue
                if self.first_loop_flag:
                    self.servo.move_angle(0)
                    self.first_loop_flag=0
                else:
                    pass
                    self.servo.move_angle(angle)
            else:
                pass
                #print "There are no users"
                
            self.last_amount_of_users=deepcopy(self.current_amount_of_users)
            r.sleep()
     
        
    
    
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

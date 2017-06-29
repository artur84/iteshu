#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import roslib
from iteshu_robot._keywords_to_command import *
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import unicodedata

class WheelchairTalk:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        time_since_last_message=0
        time_since_last_sound=0
        #voice_*_diphone, * = kal, el (spanish), rab (british)
        self.voice = rospy.get_param("~voice", "voice_el_diphone")
        self.wavepath = rospy.get_param("~wavepath", "/home/beny/catkin_ws/src/iteshu/iteshu_robot/sounds")
        self.command_to_phrase = COMMAND_TO_PHRASE  # this is deffined in wheelchairint/keywords_tocommand.py
        # Create the sound client object
        self.soundhandle = SoundClient()
        # Announce that we are ready for input
        rospy.sleep(2)
        self.soundhandle.stopAll()
        rospy.sleep(1)
        self.soundhandle.playWave(self.wavepath + "/R2D2.wav")
        rospy.sleep(5)
        self.soundhandle.say("Hola", self.voice)
        rospy.sleep(2)
        self.soundhandle.say("Mi nombre es robot iteshu", self.voice)
#         rospy.sleep(5)
#         rospy.loginfo("Say a command...")
        # Subscribe to the recognizer output
        rospy.Subscriber('recognizer/output', String, self.rec_out_callback)
       
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
#            rospy.loginfo("wheelchair talk is running correctly.")
            rospy.sleep(1)
            if (time_since_last_sound >= 18):
                self.soundhandle.stopAll()
                rospy.sleep(1)
                self.soundhandle.say("hola", self.voice)
                rospy.sleep(1)
                time_since_last_sound=0
                
            
            if (time_since_last_message >= 25):
                self.soundhandle.stopAll()
                rospy.sleep(1)
                self.soundhandle.say("Buenas tardes", self.voice)
                rospy.sleep(2)
                self.soundhandle.say("Bienvenidos", self.voice)
                rospy.sleep(2)
                self.soundhandle.say("Los puedo seguir", self.voice)
                rospy.sleep(5)
                time_since_last_message=0
                
            time_since_last_message+=1
            time_since_last_sound+=1
            r.sleep()

    def rec_out_callback(self, msg):
        # Print the recognized words on the screen
        self.soundhandle.stopAll()
        rospy.sleep(1)
        self.current_command = msg.data
        self.from_utf8_to_ascii()
        # Speak-out the recognized words.
        try:  # If the command was recognized
            self.soundhandle.say(self.current_command, self.voice)
            rospy.sleep(1)
        except:
            self.soundhandle.say("No entiendo", self.voice)

    def from_utf8_to_ascii(self):
        """Use this function to be able to speak spanish"""
        #unicode_str=self.current_command.encode('ascii','ignore')
        s=self.current_command.decode('utf-8')
        nfkd_form = unicodedata.normalize('NFKD', s)
        only_ascii = nfkd_form.encode('ASCII', 'ignore')
        self.current_command=only_ascii
        
        
                
                
            
        
    def cleanup(self):
        self.soundhandle.say("Adios", self.voice)
        rospy.sleep(4)

if __name__ == "__main__":
    rospy.init_node('wheelchair_talk')
    try:
        WheelchairTalk()
    except:
        pass

#!/usr/bin/env python

import roslib
from wheelchairint._keywords_to_command import *
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class WheelchairTalk:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        time_since_last_message=0
        time_since_last_sound=0
        #voice_*_diphone, * = kal, el (spanish), rab (british)
        self.voice = rospy.get_param("~voice", "voice_el_diphone")
        self.wavepath = rospy.get_param("~wavepath", "")
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
        self.soundhandle.say("Mi nombre es robot coyote", self.voice)
        rospy.sleep(5)
        rospy.loginfo("Say a command...")

        # Subscribe to the recognizer output
        rospy.Subscriber('recognizer/output', String, self.rec_out_callback)
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
#            rospy.loginfo("wheelchair talk is running correctly.")
            rospy.sleep(1)
            if (time_since_last_sound >= 5):
                self.soundhandle.stopAll()
                rospy.sleep(1)
                self.soundhandle.playWave(self.wavepath + "/R2D2.wav")
                rospy.sleep(5)
                time_since_last_sound=0
            
            if (time_since_last_message >= 12):
                self.soundhandle.stopAll()
                rospy.sleep(1)
                self.soundhandle.say("Hola", self.voice)
                rospy.sleep(2)
                self.soundhandle.say("Mi nombre es robot coyote", self.voice)
                rospy.sleep(5)
                time_since_last_message=0
                
            time_since_last_message+=1
            time_since_last_sound+=1
            r.sleep()

    def rec_out_callback(self, msg):
        # Print the recognized words on the screen
        self.current_command = msg.data
        # Speak-out the recognized words.
        try:  # If the command was recognized
            self.soundhandle.say(self.command_to_phrase[self.current_command], self.voice)
        except:
            self.soundhandle.say("repeat", self.voice)


    def cleanup(self):
        self.soundhandle.say("Adios", self.voice)
        rospy.sleep(4)

if __name__ == "__main__":
    rospy.init_node('wheelchair_talk')
    try:
        WheelchairTalk()
    except:
        pass

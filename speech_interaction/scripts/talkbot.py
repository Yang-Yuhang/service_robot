#!/usr/bin/env python

"""
    talkbot.py 
    
    Use the sound_play client to answer what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class TalkBot:
     def __init__(self, script_path):
         rospy.init_node('talkbot')

         rospy.on_shutdown(self.cleanup)

         # Create the sound client object
         #self.soundhandle = SoundClient()
         self.soundhandle = SoundClient(blocking=True)

         # Wait a moment to let the client connect to the sound_play server
         rospy.sleep(1)

         # Make sure any lingering sound_play processes are stopped.
         self.soundhandle.stopAll()

         
         #Announce that we are ready
         self.soundhandle.say('Hello, I am Bully. What can I do for you?',volume=0.03)
         rospy.sleep(3)

         rospy.loginfo("Say one of the navigation commands...")

         # Subscribe to the recognizer output and set the callback function
         rospy.Subscriber('/lm_data', String, self.talkback)
         

     def talkback(self, msg):
         #Print the recognized words on the screen
         rospy.loginfo(msg.data)

         if msg.data.find('HOW-OLD-ARE-YOU')>-1:
             rospy.loginfo("Talkbot: I am twenty-one years old.")
             self.soundhandle.say("I am twenty-one years old.", volume=0.03)
             #rospy.sleep(2)
         elif msg.data.find('COLOR-YOU-LIKE')>-1:
             rospy.loginfo("Talkbot: OK. I like purple best")
             self.soundhandle.say("I like purple best.", volume=0.03)
	     #rospy.sleep(2)
         elif msg.data.find('ARE-YOU-FROM')>-1:
             rospy.loginfo("Talkbot: I am from Yunan Province,China.")
             self.soundhandle.say("I am from Yunan Province,China.", volume=0.03)
             #rospy.sleep(2)
         elif msg.data.find('SPORT-YOU-LIKE')>-1:
             rospy.loginfo("Talkbot: I like playing badminton.")
             self.soundhandle.say("I like playing badminton.", volume=0.03)
             #rospy.sleep(2)
         elif msg.data.find('INTRODUCE-YOURSELF')>-1:
             rospy.loginfo("Talkbot: I am Bully, a service robot.")
             self.soundhandle.say("I am Bully, a service robot.", volume=0.03)
             #rospy.sleep(2)
         elif msg.data=='':
             rospy.sleep(1)
         else:
             rospy.loginfo("Talkbot: Sorry I can not understand,can you repeat it?")
             self.soundhandle.say("Sorry I can not understand,can you repeat it?", volume=0.03)
             #rospy.sleep(3)

     def cleanup(self):
         self.soundhandle.stopAll()
         rospy.loginfo("Shutting down talkbot node...")

if __name__=="__main__":
    try:
        TalkBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TalkBot node terminated.")








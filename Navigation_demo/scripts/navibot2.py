#!/usr/bin/env python

"""
    talkbot.py 
    
    Use the sound_play client to answer what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.msg import RotatedRectStamped
from opencv_apps.msg import Face
from opencv_apps.msg import Rect
from opencv_apps.msg import RotatedRect
from opencv_apps.msg import Point2D


class NaviBot:
     def __init__(self, script_path):
         rospy.init_node('navibot')

         rospy.on_shutdown(self.cleanup)

         # Create the sound client object
         #self.soundhandle = SoundClient()
         self.soundhandle = SoundClient(blocking=True)

         # Wait a moment to let the client connect to the sound_play server
         rospy.sleep(1)

         # Make sure any lingering sound_play processes are stopped.
         self.soundhandle.stopAll()

         
         #Announce that we are ready
         self.soundhandle.say('Hello, I am Bully. What can I do for you?',volume=0.05)
         rospy.sleep(3)

         rospy.loginfo("Say one of the navigation commands...")

         # Subscribe to the recognizer output and set the callback function
         rospy.Subscriber('/lm_data', String, self.talkback)


         #Publish to the navito_point topic to use rviz
         self.navi = rospy.Publisher("/navi_to_point", String, queue_size=10) 

         # Subscribe to the navigation result
         self.navigation_back=""
         rospy.Subscriber('/navigation_feed_point', String, self.naviback)

     def naviback(self, res):
         self.navigation_back=res.data

     def talkback(self, msg):
         #Print the recognized words on the screen
         rospy.loginfo(msg.data)

         if msg.data.find('HOW-OLD-ARE-YOU')>-1:
             rospy.loginfo("Bully: I am twenty-one years old.")
             self.soundhandle.say("I am twenty-one years old.", volume=0.05)
             #rospy.sleep(1)
         elif msg.data.find('COLOR-YOU-LIKE')>-1:
             rospy.loginfo("Bully:I like purple best")
             self.soundhandle.say("I like purple best.", volume=0.05)
             #rospy.sleep(1)
         elif msg.data.find('WHAT-BLUE-IS-LIKE')>-1:
             rospy.loginfo("Bully:Blue is the color of sky.")
             self.soundhandle.say("Blue is the color of sky.", volume=0.05)
             #rospy.sleep(1)
         elif msg.data.find('ARE-YOU-FROM')>-1:
             rospy.loginfo("Bully: I am from Yunan Province, China.")
             self.soundhandle.say("I am from Yunan Province, China.", volume=0.05)
             #rospy.sleep(2)
         elif msg.data.find('SPORT-YOU-LIKE')>-1:
             rospy.loginfo("Bully: I like playing badminton.")
             self.soundhandle.say("I like playing badminton.", volume=0.05)
             #rospy.sleep(2)
         elif msg.data.find('INTRODUCE-YOURSELF')>-1:
             rospy.loginfo("Bully: I am Bully, a service robot.")
             self.soundhandle.say("I am Bully, a service robot.", volume=0.05)
             #rospy.sleep(2)
         elif msg.data.find('GO-TO-THE-BOOKSHELF')>-1:
             rospy.loginfo("Bully: I am going to the bookshelf.")
             self.soundhandle.say("I am going to the bookshelf.", volume=0.05)
             self.navi.publish('go to the bookshelf')
             while (True):
                 if self.navigation_back == "Reached the bookshelf":
                     self.soundhandle.say("I reached the bookshelf.", volume=0.05)
                     break
             #rospy.sleep(5)
         elif msg.data.find('GO-TO-THE-BEDROOM')>-1:
             rospy.loginfo("Bully: I am going to the bedroom.")
             self.soundhandle.say("I am going to the bedroom.", volume=0.05)
             self.navi.publish('go to the bedroom')
             while (True):
                 if self.navigation_back == "reached the bedroom":
                     self.soundhandle.say("I reached the bedroom.", volume=0.05)
                     break
             #rospy.sleep(5)
         elif msg.data.find('GO-TO-THE-KITCHEN')>-1:
             rospy.loginfo("Bully: I am going to the kitchen.")
             self.soundhandle.say("I am going to the kitchen.", volume=0.05)
             self.navi.publish('go to the kitchen')
             while (True):
                 if self.navigation_back == "reached the kitchen":
                     self.soundhandle.say("I reached the kitchen.", volume=0.05)
                     break
             #rospy.sleep(5)
         elif msg.data.find('GO-TO-THE-TABLE')>-1:
             rospy.loginfo("Bully: I am going to the table.")
             self.soundhandle.say("I am going to the table.", volume=0.05)
             self.navi.publish('go to the table')
             while (True):
                 if self.navigation_back == "reached the table":
                     self.soundhandle.say("I reached the table.", volume=0.05)
                     break
             #rospy.sleep(5)
         elif msg.data=='':
             #rospy.loginfo("Bully: Sorry I can hear you, can you speak louder?")
             #self.soundhandle.say("Sorry I can hear you, can you speak louder?", volume=0.05)
             rospy.sleep(1)
         else:
             rospy.loginfo("Bully: Sorry I can not understand, can you repeat it?")
             self.soundhandle.say("Sorry I can not understand, can you repeat it?", volume=0.05)
             #rospy.sleep(2)

     def cleanup(self):
         self.soundhandle.stopAll()
         rospy.loginfo("Shutting down navibot node...")

if __name__=="__main__":
    try:
        NaviBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("navibot node terminated.")



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
         self.soundhandle.say('Hello, I am Bully. What can I do for you?',volume=0.1)
         rospy.sleep(3)

         rospy.loginfo("Say one of the navigation commands...")

         # Subscribe to the recognizer output and set the callback function
         rospy.Subscriber('/lm_data', String, self.talkback)

         #the position of face detected
         self.face_x=0
         self.face_y=0
         # Subscribe to the face_detection output
         rospy.Subscriber('/face_detection/faces', FaceArrayStamped, self.face_back)
  
         # the center of blue object
         self.blue_x = 0
         self.blue_width = 0
         # Subscribe to the blue_tracting output
         rospy.Subscriber('/camshift/track_box', RotatedRectStamped, self.blue_back)

         #Publish to the take_photo topic to use take_photo node
         self.take_photo = rospy.Publisher("/take_photo", String, queue_size=10)

     def blue_back(self,blue_data):
          self.blue_x = blue_data.rect.center.x 
          self.blue_width = blue_data.rect.size.width
             
     def face_back(self,face_data):
         pos = face_data.faces
         if pos:
             self.face_x=pos[0].face.x
             self.face_y=pos[0].face.y

     def talkback(self, msg):
         #Print the recognized words on the screen
         rospy.loginfo(msg.data)

         if msg.data.find('HOW-OLD-ARE-YOU')>-1:
             rospy.loginfo("Talkbot: I am twenty-one years old.")
             self.soundhandle.say("I am twenty-one years old.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data.find('COLOR-YOU-LIKE')>-1:
             rospy.loginfo("Talkbot:I like purple best")
             self.soundhandle.say("I like purple best.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data.find('WHAT-BLUE-IS-LIKE')>-1:
             rospy.loginfo("Talkbot:Blue is the color of sky.")
             self.soundhandle.say("Blue is the color of sky.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data.find('RECOGNIZE-BLUE')>-1:
             rospy.loginfo("Talkbot:Yes, let me try. I will totally try for 5 times.")
             self.soundhandle.say("Yes, let me try. I will totally try for 5 times.", volume=0.1)
             #cycling identification for 5 times
             i=0
             while(i<=5):
                 if i==5:
                     rospy.loginfo("Talkbot: I will stop tracking blue.")
                     self.soundhandle.say("I will stop tracking blue.", volume=0.1)
                     break
                 if self.blue_x != 0 and self.blue_width != 0:
                     rospy.loginfo("Talkbot: Look. I found blue. And I can track it.")
                     self.soundhandle.say("Look. I found blue. And I can track it.", volume=0.1)
                     i=i+1
                     rospy.sleep(3)
                 else:
                     rospy.loginfo("Talkbot: Sorry. I don't find blue.")
                     self.soundhandle.say("Sorry. I don't find blue.", volume=0.1)
                     i=i+1
                     rospy.sleep(2)
	         #rospy.sleep(1)
         elif msg.data.find('ARE-YOU-FROM')>-1:
             rospy.loginfo("Talkbot: I am from Yunan Province, China.")
             self.soundhandle.say("I am from Yunan Province, China.", volume=0.1)
             #rospy.sleep(2)
         elif msg.data.find('SPORT-YOU-LIKE')>-1:
             rospy.loginfo("Talkbot: I like playing badminton.")
             self.soundhandle.say("I like playing badminton.", volume=0.1)
             #rospy.sleep(2)
         elif msg.data.find('INTRODUCE-YOURSELF')>-1:
             rospy.loginfo("Talkbot: I am Bully, a service robot.")
             self.soundhandle.say("I am Bully, a service robot.", volume=0.1)
             #rospy.sleep(2)
         elif msg.data.find('TAKE-A-PHOTO')>-1:
             rospy.loginfo("Talkbot: OK, please stand in front of me and look at my eyes.")
             self.soundhandle.say("OK, please stand in front of me and look at my eyes.", volume=0.1)
             #rospy.sleep(1)
             while(True):
                 if self.face_x<240 and self.face_x>0:
                     if self.face_y<160 and self.face_y>0:
                         rospy.loginfo("Talkbot: Please lower your head and stand a little to the left.")
                         self.soundhandle.say("Please lower your head and stand a little to the left.", volume=0.1)
                     elif self.face_y>300:
                         rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up, and stand a little to the left.")
                         self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up, and stand a little to the left.", volume=0.1)
                     else:
                         rospy.loginfo("Talkbot: Please stand a little to the left.")
                         self.soundhandle.say("Please stand a little to the left.", volume=0.1)
                     self.face_x=0 
                     self.face_y=0
                     rospy.sleep(1)
                 elif self.face_x>420:
                     if self.face_y<160 and self.face_y>0:
                         rospy.loginfo("Talkbot: Please lower your head and stand a little to the right.")
                         self.soundhandle.say("Please lower your head and stand a little to the right.", volume=0.1)
                     elif self.face_y>300:
                         rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up, and stand a little to the right.")
                         self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up, and stand a little to the right.", volume=0.1)
                     else:
                         rospy.loginfo("Talkbot: Please stand a little to the right.")
                         self.soundhandle.say("Please stand a little to the right.", volume=0.1)
                     self.face_x=0
                     self.face_y=0
                     rospy.sleep(1)
                 elif self.face_x>=240 and self.face_x<=420:
                     if self.face_y<160 and self.face_y>0:
                         rospy.loginfo("Talkbot: Please lower your head.")
                         self.soundhandle.say("Please lower your head.", volume=0.1)
                         self.face_x=0
                         self.face_y=0
                         rospy.sleep(1)
                     elif self.face_y>300:
                         rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up.")
                         self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up.", volume=0.1)
                         self.face_x=0
                         self.face_y=0
                         rospy.sleep(1)
                     else:
                         rospy.loginfo("Talkbot: OK, please don't move.")
                         self.soundhandle.say("OK, please don't move.", volume=0.1)
                         break
                 elif self.face_x==0 or self.face_y==0:
                     rospy.loginfo("Talkbot: I can't catch your face, please stand and face to me.")
                     self.soundhandle.say("I can't catch your face, please stand and face to me.", volume=0.1)
             rospy.loginfo("Talkbot: 3! 2! 1!")
             self.soundhandle.say("3! 2! 1!", volume=0.1)
             self.take_photo.publish('take photo')
             rospy.loginfo("Talkbot: You can see this photo.")
             self.soundhandle.say("You can see this photo.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data=='':
             rospy.sleep(1)
         else:
             rospy.loginfo("Talkbot: Sorry I can not understand, can you repeat it?")
             self.soundhandle.say("Sorry I can not understand, can you repeat it?", volume=0.1)
             #rospy.sleep(2)

     def cleanup(self):
         self.soundhandle.stopAll()
         rospy.loginfo("Shutting down talkbot node...")

if __name__=="__main__":
    try:
        TalkBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("talkbot node terminated.")


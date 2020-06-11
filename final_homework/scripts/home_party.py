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
from geometry_msgs.msg import Twist
from math import pi


class TalkBot:
     def __init__(self, script_path):
         rospy.init_node('talkbot')

         rospy.on_shutdown(self.cleanup)

         # Create the sound client object
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

         # Subscribe to the face_reconnitiontion output
         self.recognition_x=0
         self.recognition_y=0
         self.recognition_confidence=0
         self.recognition_label=""
         rospy.Subscriber('/face_recognition/output', FaceArrayStamped, self.recogniton_back)

         # the center of blue object
         self.blue_x = 0
         self.blue_width = 0
         # Subscribe to the blue_tracting output
         rospy.Subscriber('/camshift/track_box', RotatedRectStamped, self.blue_back)

         #Publish to the take_photo topic to use take_photo node
         self.take_photo = rospy.Publisher("/take_photo", String, queue_size=10)

         #Publish to the navito_point topic to use rviz
         self.navi = rospy.Publisher("/navi_to_point", String, queue_size=10) 

         # Subscribe to the navigation result
         self.navigation_back=""
         rospy.Subscriber('/navigation_feed_point', String, self.naviback)

     def blue_back(self,blue_data):
          self.blue_x = blue_data.rect.center.x 
          self.blue_width = blue_data.rect.size.width

     def recogniton_back(self,face_rec):
         rec = face_rec.faces  
         if rec:
             self.recognition_x=rec[0].face.x
             self.recognition_y=rec[0].face.y
             self.recognition_label=rec[0].label
             self.recognition_confidence=rec[0].confidence
         else:
             self.recognition_x=0
             self.recognition_y=0
             self.recognition_label=""
             self.recognition_confidence=0

     def naviback(self, res):
         self.navigation_back=res.data

     def talkback(self, msg):
         #Print the recognized words on the screen
         rospy.loginfo(msg.data)


	 ii = 1
	 move = Twist()
	 pubb = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
	 ratee = rospy.Rate(10)
	 move.linear.x = 0
         move.linear.y = 0
         move.linear.z = 0
         move.angular.x = 0
         move.angular.y = 0
         move.angular.z = 0.3
         zz = move.angular.z


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
             rospy.loginfo("Talkbot:  Do you want to take a photo of ." + self.recognition_label + "?")
             self.soundhandle.say("Do you want to take a photo of ." + self.recognition_label +"?", volume=0.1)
             rospy.loginfo("Talkbot: OK, please stand in front of me and look at my eyes.")
             self.soundhandle.say("OK, please stand in front of me and look at my eyes.", volume=0.1)
             #rospy.sleep(1)
             while(True):
                 if self.recognition_confidence>1600:
                     if self.recognition_x<240 and self.recognition_x>0:
                         if self.recognition_y<160 and self.recognition_y>0:
                             rospy.loginfo("Talkbot: Please lower your head and stand a little to the left.")
                             self.soundhandle.say("Please lower your head and stand a little to the left.", volume=0.1)
                         elif self.recognition_y>300:
                             rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up, and stand a little to the left.")
                             self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up, and stand a little to the left.", volume=0.1)
                         else:
                             rospy.loginfo("Talkbot: Please stand a little to the left.")
                             self.soundhandle.say("Please stand a little to the left.", volume=0.1)
                         self.recognition_x=0 
                         self.recognition_y=0
                         self.recognition_confidence=0
                         self.recognition_label=""
                         rospy.sleep(1)
                     elif self.recognition_x>420:
                         if self.recognition_y<160 and self.recognition_y>0:
                             rospy.loginfo("Talkbot: Please lower your head and stand a little to the right.")
                             self.soundhandle.say("Please lower your head and stand a little to the right.", volume=0.1)
                         elif self.recognition_y>300:
                             rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up, and stand a little to the right.")
                             self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up, and stand a little to the right.", volume=0.1)
                         else:
                             rospy.loginfo("Talkbot: Please stand a little to the right.")
                             self.soundhandle.say("Please stand a little to the right.", volume=0.1)
                         self.recognition_x=0
                         self.recognition_y=0
                         self.recognition_confidence=0
                         self.recognition_label=""
                         rospy.sleep(1)
                     elif self.recognition_x>=240 and self.recognition_x<=420:
                         if self.recognition_y<160 and self.recognition_y>0:
                             rospy.loginfo("Talkbot: Please lower your head.")
                             self.soundhandle.say("Please lower your head.", volume=0.1)
                             self.recognition_x=0
                             self.recognition_y=0
                             self.recognition_confidence=0
                             self.recognition_label=""
                             rospy.sleep(1)
                         elif self.recognition_y>300:
                             rospy.loginfo("Talkbot: Sorry, you are a little lower than my camera, please stand up.")
                             self.soundhandle.say("Sorry, you are a little lower than my camera, please stand up.", volume=0.1)
                             self.recognition_x=0
                             self.recognition_y=0
                             self.recognition_confidence=0
                             self.recognition_label=""
                             rospy.sleep(1)
                         else:
                             rospy.loginfo("Talkbot: OK, please don't move.")
                             self.soundhandle.say("OK, please don't move.", volume=0.1)
                             break
                 elif self.recognition_confidence<=1600 and self.recognition_confidence>0:
                     rospy.loginfo("Talkbot: You are not "+self.recognition_label+". I won't take pictures.")
                     self.soundhandle.say(" You are not "+self.recognition_label+". I won't take pictures." , volume=0.1)
                     self.recognition_x=0
                     self.recognition_y=0
                     self.recognition_confidence=0
                     self.recognition_label=""

                 elif self.recognition_x==0 or self.recognition_y==0:
                     rospy.loginfo("Talkbot: I can't catch your face, please stand and face to me.")
                     self.soundhandle.say("I can't catch your face, please stand and face to me.", volume=0.1)
             rospy.loginfo("Talkbot: 3! 2! 1!")
             self.soundhandle.say("3! 2! 1!", volume=0.1)
             self.take_photo.publish('take photo')
             rospy.loginfo("Talkbot: You can see this photo.")
             self.soundhandle.say("You can see this photo.", volume=0.1)
             #rospy.sleep(1)
        
        # the robot will go to 3 places to find a person: study -> bedroom -> living room
        # whether the robot find the person or not, the robot will return to the original point and report
         elif msg.data.find('PLEASE-FIND-YANG-YUHANG')>-1:
             rospy.loginfo("Bully: I will go to find Yang Yuhang.")
             self.soundhandle.say("I will go to find Yang Yuhang.", volume=0.1)
             i = 1
             while(True):
		 findd = 0
                 if i == 1:
                     rospy.loginfo("Bully: I am going to the study to find him.")
                     self.navi.publish('go to the study')
                     rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
		     ii = 0
		     while ii < (1/zz) * 20 *pi:
			pubb.publish(move)
                     	if self.recognition_confidence>2000 and self.recognition_label=="Yang Yuhang":
			     move.angular.z = 0
			     pubb.publish(move)
                             rospy.loginfo("Bully: I find Yang Yuhang, he is in the study. Returning...")
                             self.navi.publish('go back')
                             rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
                             self.soundhandle.say("I find Yang Yuhang, he is in the study.", volume=0.1)
			     findd = 1
		             break
		        else:
			     ii = ii+1
			     ratee.sleep()
		     if findd == 1:
			break
                     else:
                        rospy.loginfo("Bully: Yang Yuhang is not in the study. I will go to the bedroom.")
                        i = 2
                 if i == 2:
                     rospy.loginfo("Bully: I am going to the bedroom to find him.")
                     self.navi.publish('go to the bedroom')
                     rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
		     ii = 0
		     while ii < (1/zz) * 20 *pi:
			pubb.publish(move)
                     	if self.recognition_confidence>2000 and self.recognition_label=="Yang Yuhang":
			     move.angular.z = 0
			     pubb.publish(move)
                             rospy.loginfo("Bully: I find Yang Yuhang, he is in the bedroom. Returning...")
                             self.navi.publish('go back')
                             rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
                             self.soundhandle.say("I find Yang Yuhang, he is in the bedroom.", volume=0.1)
			     findd = 1
                             break
                        else:
			     ii = ii+1
			     ratee.sleep()
		     if findd == 1:
			break
                     else:
                         rospy.loginfo("Bully: Yang Yuhang is not in the bedroom. I will go to the living room.")
                         i = 3
                 if i == 3:
                     rospy.loginfo("Bully: I am going to the living room to find him.")
                     self.navi.publish('go to the living room')
                     rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
		     ii = 0
		     while ii < (1/zz) * 20 *pi:
			 pubb.publish(move)
                     	 if self.recognition_confidence>2000 and self.recognition_label=="Yang Yuhang":
			     move.angular.z = 0
			     pubb.publish(move)
                             rospy.loginfo("Bully: I find Yang Yuhang, he is in the living room. Returning...")
                             self.navi.publish('go back')
                             rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
                             self.soundhandle.say("I find Yang Yuhang, he is in the living room.", volume=0.1)
			     findd = 1
                             break
                         else:
			     ii = ii+1
			     ratee.sleep()
		     if findd == 1:
			 break
                     else:
                         rospy.loginfo("Bully: Yang Yuhang is not in the living room. I will return.")
                         self.navi.publish('go back')
                         rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
                         rospy.loginfo("Bully: Sorry I don't find Yang Yuhang.")
                         self.soundhandle.say("Sorry I don't find Yang Yuhang.", volume=0.1)
                         break
             #rospy.sleep(1)
         elif msg.data.find('PLEASE-RETURN-BACK')>-1:
             rospy.loginfo("Bully: I am going back.")
             self.soundhandle.say("I am going back.", volume=0.1)
             self.navi.publish('go back')
             rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
             self.soundhandle.say("I returned.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data.find('GO-TO-THE-STUDY')>-1:
             rospy.loginfo("Bully: I am going to the study.")
             self.soundhandle.say("I am going to the study.", volume=0.1)
             self.navi.publish('go to the study')
             rospy.wait_for_message('/navigation_feed_point', String, timeout=None)
             self.soundhandle.say("I reached the study.", volume=0.1)
             #rospy.sleep(1)
         elif msg.data.find('GO-TO-THE-BEDROOM')>-1:
             rospy.loginfo("Bully: I am going to the bedroom.")
             self.soundhandle.say("I am going to the bedroom.", volume=0.1)
             self.navi.publish('go to the bedroom')
             while (True):
                 if self.navigation_back == "reached the bedroom":
                     self.soundhandle.say("I reached the bedroom.", volume=0.1)
                     break
             #rospy.sleep(1)
         elif msg.data.find('GO-TO-THE-KITCHEN')>-1:
             rospy.loginfo("Bully: I am going to the kitchen.")
             self.soundhandle.say("I am going to the kitchen.", volume=0.1)
             self.navi.publish('go to the kitchen')
             while (True):
                 if self.navigation_back == "reached the kitchen":
                     self.soundhandle.say("I reached the kitchen.", volume=0.1)
                     break
             #rospy.sleep(1)
         elif msg.data.find('GO-TO-THE-LIVING-ROOM')>-1:
             rospy.loginfo("Bully: I am going to the living room.")
             self.soundhandle.say("I am going to the living room.", volume=0.1)
             self.navi.publish('go to the living room')
             while (True):
                 if self.navigation_back == "reached the living room":
                     self.soundhandle.say("I reached the living room.", volume=0.1)
                     break
             #rospy.sleep(1)      
         elif msg.data.find('GO-TO-THE-DINING-ROOM')>-1:
             rospy.loginfo("Bully: I am going to the dining room.")
             self.soundhandle.say("I am going to the dining room.", volume=0.1)
             self.navi.publish('go to the dining room')
             while (True):
                 if self.navigation_back == "reached the dining room":
                     self.soundhandle.say("I reached the dining room.", volume=0.1)
                     break
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






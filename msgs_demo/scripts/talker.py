#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=20)
    rospy.init_node('talker', anonymous=True)  
    rate = rospy.Rate(10) # 10times/s
    i=1
    while not rospy.is_shutdown():
        hello_str = "1710675 %s" % i   
        rospy.loginfo(hello_str)  
        pub.publish(hello_str)
        i=i+1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

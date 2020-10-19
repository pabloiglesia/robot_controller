#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

def talker():
    pub = rospy.Publisher('switch_on_off', Empty, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Enviando mensaje")
        pub.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
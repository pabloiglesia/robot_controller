#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('toggle_led', Empty, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Enviando mensaje")
        pub.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


#
#
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#
#
# def listener():
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber("object_gripped", Bool, callback)
#
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#
#
# if __name__ == '__main__':
#     print("Preparado para escuchar:")
#     listener()
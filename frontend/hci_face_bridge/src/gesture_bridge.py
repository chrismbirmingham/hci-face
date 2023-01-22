#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
import requests
import time




if __name__ == "__main__":

    rospy.init_node("gesture_bridge")
    rospy.loginfo("gesture_bridge started!")

    gesture_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
    rospy.sleep(3.0)
    rate = rospy.Rate(5)
    gesture_pub.publish("QT/hi")
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("getting gesture")
            r = requests.get("http://192.168.1.136:8000/api/gestureControl")
            gesture = r.text
            if gesture is not "":
                rospy.loginfo(gesture)
                gesture_pub.publish(gesture)
        except requests.exceptions.ConnectionError as e:
            print(e)
        rate.sleep()
    rospy.loginfo("finished")



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
    
    gesture_pub.publish("QT/hi")
    while True:
        try:
            r = requests.get("http://192.168.1.136:8000/api/gestureControl")
            gesture = r.text
            if gesture is not "":
                gesture_pub.publish(gesture)
        except Exception as e:
            print(e)
        time.sleep(.1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    rospy.loginfo("finished")

# rostopic pub /qt_robot/gesture/show std_msgs/String "data: 'QT/hi'"
# QT/
# angry, bye-bye, kiss, send_kiss, show_right, surprise, up_left,bored, bye, happy, point_front, show_left, show_tablet, swipe_left, up_right,breathing_exercise, challenge, hi, sad, show_QT, sneezing, swipe_right, yawn,
# QT/emotions/
# afraid, angry, calm, disgusted, happy, hoora, sad, shy, surprised,

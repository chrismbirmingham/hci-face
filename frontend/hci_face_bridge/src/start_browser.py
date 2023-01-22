#!/usr/bin/env python
import rospy
import subprocess
import os

kill_command = "killall -15 chomium-browser"
start_command = "chromium-browser --start-fullscreen --display=:0 http://192.168.1.136:3000/"#.format(url=rospy.get_param('qt_robot/face/url'))


def callback_srv(_):

    return resp

if __name__ == "__main__":
    os.system(kill_command)

    rospy.init_node("start_browser")
    rospy.loginfo("start_browser started!")

    rospy.sleep(3.0)
    
    process = subprocess.Popen(start_command, shell=True)

    rospy.loginfo("Browser started with PID: %d" % process.pid)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    rospy.loginfo("finished")


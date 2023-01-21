import requests
import subprocess
import time
import os

env = os.environ.copy()
# ssh qtrobot@192.168.100.1
while True:
    r = requests.get("http://192.168.1.136:8000/api/gestureControl")
    gesture = r.text
    if gesture is not "":
        cmd = ["rostopic", "pub", "/qt_robot/gesture/play", "std_msgs/String", "data: 'QT/happy'"]
        process = subprocess.Popen(cmd,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
    time.sleep(1)

# env['ROS_MASTER_URI'] = 'http://10.42.0.49:11311'
action_publisher = subprocess.run(
    ["/opt/ros/melodic/bin/rostopic", "pub", "-r", "20",
     "/robot_operation", "std_msgs/String", "start"],
    env=env, check=True)


# rostopic pub /qt_robot/gesture/show std_msgs/String "data: 'QT/hi'"
# QT/
# angry, bye-bye, kiss, send_kiss, show_right, surprise, up_left,bored, bye, happy, point_front, show_left, show_tablet, swipe_left, up_right,breathing_exercise, challenge, hi, sad, show_QT, sneezing, swipe_right, yawn,
# QT/emotions/
# afraid, angry, calm, disgusted, happy, hoora, sad, shy, surprised,
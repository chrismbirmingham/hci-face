# hci_face_bridge

This package will automatically start the face on QT and will recieve and play gestures from the HCI server backend. Currently only functions over a local area network.


## Installation on QT

To set up this bridge, clone the hci-face repository to the nuc. 

The hci_face_bridge package will need to be copied to the pi. 
```
scp -r hci-face/frontend/hci_face_bridge qtrobot@192.168.100.1:/home/qtrobot/catkin_ws/src/hci_face_bridge
```

You should then ssh into the pi:
```
ssh qtrobot@192.168.100.1
```

And enter the catkin_ws directory:
```
cd catkin_ws
```


Then you must copy the autostart script to the pi's autostart folder:
```
cp hci_face_bridge/start_HCI_FACE.sh /home/qtrobot/robot/autostart/
```

And build the hci_face_bridge package:
```
catkin_make --pkg hci_face_bridge
```

Finally, `exit` to return to the nuc, and open a web browser on QT (e.g., Firefox) and go to http://192.168.100.1:8080/.
> 
1. Click ‘Autostart’. You’ll be prompted for a username and password. Enter `qtrobot` for both.
2. Click the ‘Active’ checkbox next to `start_HCI_FACE.sh`.

When you restart QT the gestures and face should run automatically, though it may take up to two minutes to fully load.

## Additional Support

There can be issues with setting up HCI-FACE on QT and it may not always start perfectly every time. 

If you are having connectivity challenges on startup, I recommend disabling the autostart script and launching qt_robot_pi.launch directly.

The following commands can be useful in making debugging what is not working
```
ssh qtrobot@192.168.100.1
chromium-browser --start-fullscreen --display=:0 http://192.168.1.136:3000/
rostopic pub /qt_robot/gesture/play std_msgs/String "data: 'QT/hi'"
roslaunch hci_face_bridge qt_robot_pi.launch
roslaunch qt_motor qt_motor.launch
```

## Available preset gestures on QT

These can be accessed through either the server or topic ROS API

QT/[GESTURE]
- angry, bye-bye, kiss, send_kiss, show_right, surprise, up_left,bored, bye, happy, point_front, show_left, show_tablet, swipe_left, up_right,breathing_exercise, challenge, hi, sad, show_QT, sneezing, swipe_right, yawn,

QT/emotions/[EMOTION]
- afraid, angry, calm, disgusted, happy, hoora, sad, shy, surprised,

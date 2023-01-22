# hci_face_bridge

This package will automatically start the face on QT and will recieve and play gestures from the HCI server backend.


## Installation 

To set up this bridge, clone the repository to the nuc. 

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

## Alternative
```
ssh qtrobot@192.168.100.1
chromium-browser --start-fullscreen --display=:0 http://192.168.1.136:3000/
rostopic pub /qt_robot/gesture/show std_msgs/String "data: 'QT/hi'"
```
QT/
- angry, bye-bye, kiss, send_kiss, show_right, surprise, up_left,bored, bye, happy, point_front, show_left, show_tablet, swipe_left, up_right,breathing_exercise, challenge, hi, sad, show_QT, sneezing, swipe_right, yawn,


QT/emotions/
- afraid, angry, calm, disgusted, happy, hoora, sad, shy, surprised,
# hci_face_bridge

This package will automatically start the face on QT and will recieve and play gestures from the HCI server backend.


## Installation 

To set up this bridge, clone the repository to the nuc. 

The hci_face_bridge package will need to be copied to the pi. 
```
scp -r hci_face_bridge qtrobot@192.168.100.1:/home/qtrobot/catkin_ws/hci_face_bridge
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
scp hci_face_bridge/start_HCI_FACE.sh /home/qtrobot/robot/autostart/
```

And build the hci_face_bridge package:
```
catkin_make --pkg hci_face_bridge
```

Finally, `exit` to return to the nuc, and open a web browser on QT (e.g., Firefox) and go to http://192.168.100.1:8080/.
> 
1. Click ‘Autostart’. You’ll be prompted for a username and password. Enter `qtrobot` for both.
2. Click the ‘Active’ checkbox next to `start_HCI_FACE.sh`.
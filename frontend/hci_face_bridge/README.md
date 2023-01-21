# hci_face_bridge

This package will automatically start the face on QT and will recieve and play gestures from the HCI server backend.

Note, start_HCI_FACE.sh needs to be copied to the pi and set to run on launch:

```
scp start_HCI_FACE.sh qtrobot@192.168.100.1:/home/qtrobot/robot/autostart/
```

> Open a webbrowser on QT (e.g., Firefox) and go to http://192.168.100.1:8080/.
> 
1. Click ‘Autostart’. You’ll be prompted for a username and password. Enter `qtrobot` for both.
2. Click the ‘Active’ checkbox next to `start_usc.sh`.
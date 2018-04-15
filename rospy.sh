#!/bin/bash

sudo pip3 install catkin_pkg
sudo pip3 install pymorse
sudo pip3 install rospkg

sudo pip3 install opencv-python
sudo ln -sf /usr/local/lib/python3.5/site-packages/cv2.so /usr/local/lib/python3.5/dist-packages/cv2.so

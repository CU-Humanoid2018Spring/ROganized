# Simulation Environment

## Prerequisite

Make sure you follow HW2 and installed Fetch packages.

## How to use it?

1. This will create the simulated world. You can insert more objects through
Gazebo gui.

```$ roslaunch team2_gazebo pick_place_table.launch```

2. This script will take in images from Fetch's camera, process the images and
 show them in OpenCV viewer. You can use this to visualize your result.

```
$ rosrun team2_gazebo find_obejct.py
```


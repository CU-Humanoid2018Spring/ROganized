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

## To Kelsey

1. Try `roslaunch team2_gazebo camera.launch` to view the default setting.

2. You will need to relocate to objects. I didn't do it before, but [this](http://answers.gazebosim.org/question/16067/how-to-get-the-position-of-certain-object-in-gazebo/) might provide with some hints.

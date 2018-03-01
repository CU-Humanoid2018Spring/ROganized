# Simulation Environment

## Prerequisite

Make sure you follow HW2 and installed Fetch packages.

## (Old: the fetch model will be merged in the future)

1. This will create the simulated world. You can insert more objects through
Gazebo gui.

```$ roslaunch team2_gazebo pick_place_table.launch```

2. This script will take in images from Fetch's camera, process the images and
 show them in OpenCV viewer. You can use this to visualize your result.

```
$ rosrun team2_gazebo find_object.py
```

## Generate training dataset

1. If you want to see the gazebo gui, use 
   `$ roslaunch team2_gazebo camera.launch gui:=true`.
   Otherwise, `roslaunch team2_gazebo camera.launch`
 
2. Run `$ rosrun team2_gazebo spawn_random_objects.py`, the plastic cup will switch
positions randomly.

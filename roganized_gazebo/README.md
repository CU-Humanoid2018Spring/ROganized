# Simulation Environment

## Prerequisite

Make sure you follow HW2 and installed Fetch packages.

## (Old: the fetch model will be merged in the future)

1. This will create the simulated world. You can insert more objects through
Gazebo gui.

```$ roslaunch roganized_gazebo pick_place_table.launch```

2. This script will take in images from Fetch's camera, process the images and
 show them in OpenCV viewer. You can use this to visualize your result.

```
$ rosrun roganized_gazebo find_object.py
```

## Generate training dataset

1. If you want to see the gazebo gui, use 
   `$ roslaunch roganized_gazebo camera.launch display:=true`.
   Otherwise, `roslaunch roganized_gazebo camera.launch display:=false`
 
2. Run `$ rosrun roganized_rl scene_generator.py <scene_type> <img_dir>,`
where scene_type is messy/neat_linear/neat_equid/neat_cluster, and the program
will save scene images to data/img_dir.

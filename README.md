
## Installation
Before installing the toolchain, make sure you have a proper github ssh key. If not, please follow [this guide](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) before moving forward.

```
$ wget https://raw.githubusercontent.com/CU-Humanoid2018Spring/ROganized/master/install.sh
$ bash install.sh

$ echo "source ~/roganized_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

After installation, please run the below commands to verify.

```
$ roslaunch roganzied_gazebo simple_grasp.launch

# Open another terminal
$ rostopic list | grep camera
```

If you see the following, then you are all set.
```
/camera/depth_registered/camera_info
/camera/depth_registered/image_raw
/camera/depth_registered/points
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/head_camera/crop_decimate/parameter_descriptions
/head_camera/crop_decimate/parameter_updates
/head_camera/depth_downsample/camera_info
/head_camera/depth_downsample/image_raw
/head_camera/depth_downsample/points
/head_camera/depth_registered/camera_info
/head_camera/depth_registered/image_raw
/head_camera/depth_registered/points
/head_camera/head_camera_nodelet_manager/bond
/head_camera/parameter_descriptions
/head_camera/parameter_updates
/head_camera/rgb/camera_info
/head_camera/rgb/image_raw
```

Or you may see the following. That means Gazebo is not publishing images.
```
/head_camera/crop_decimate/parameter_descriptions
/head_camera/crop_decimate/parameter_updates
/head_camera/depth_downsample/camera_info
/head_camera/depth_downsample/image_raw
/head_camera/depth_downsample/points
/head_camera/head_camera_nodelet_manager/bond
```

# awsp_ros
Collection of packages as a ROS workspace for the AAU AWSP 5th and 6th semester project.


The repo hosts a variety of packages originally developed standalone. This repo was
created to unify and sync these packages into one, coherent ROS workspace. If you wish to
use these packages standalone visit the links below.

You can find these repos here:
- [partesian_pose](https://github.com/EduPonz/cartesian_pose)
- [gnss_l86_interface](https://github.com/EduPonz/gnss_l86_interface)
- [catamaran_controller](https://github.com/IvelinPenchev/catamaran_controller)
- [gy_88_interface](https://github.com/dmicha16/gy_88_interface)
- [ros_sensor_filter_kit](https://github.com/dmicha16/ros_sensor_filter_kit)

## Getting Started
This guide assumes that you have already install the `catkin` environment for `ROS Kinetic`. Else, follow [this installation guide](http://wiki.ros.org/catkin#Installing_catkin).

You'll have to initialize a workspace, you can choose to do it manually:
```shell
source /opt/ros/kinetic/setup.bash
mkdir -p ~/<catkin_ws_name>/src
cd ~/<catkin_ws_name>/
catkin_make
source devel/setup.bash
```

Then, you just need to replace the content of the `<catkin_ws_name>/src` directory with the `awsp_ros`. To do that just run:
```shell
cd <catkin_ws_name>
rm -r src/
git clone https://github.com/dmicha16/awsp_ros.git
mv awsp_ros/ src/
catkin_make
```

OR, you can do it by running the script `setup_awsp_ws.sh` inside `src/awsp_scripts
`by typing `./setup_awsp_ws.sh`. This will create everything for you right away.

___

Credits to Kugle-ROS project for a workspace structure inspiration.

[Kugle-ROS](https://github.com/mindThomas/Kugle-ROS)
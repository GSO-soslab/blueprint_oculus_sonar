# Blueprint Oculus Sonar ROS2 driver

This is a ROS2 metapackage including:
 * A ROS2 package **oculus_interfaces** containing the useful ROS messages definitions,
 * A ROS2 package **oculus_ros2** interfacing the driver messages with ROS2 topics,

This ROS2 metapackage was developed and tested using:<br>
* Ubuntu 24.04 LTS<br>
* ROS2 jazzy
* M750d Sonar
### Installzation
```sh
$ cd ~/YOUR_WS/src
$ git clone https://github.com/GSO-soslab/blueprint_oculus_sonar
$ cd blueprint_oculus_sonar
$ git branch jazzy-devel
$ cd ../..
$ colcon build --packages-select oculus_interfaces oculus_sonar oculus_ros2
```

### Launch

```sh
$ ros2 launch oculus_ros2 default.launch.py
```

### Acknowledgement
This repository is modified version of [ENSTABretagneRobotics](https://github.com/ENSTABretagneRobotics/oculus_ros2) ROS2 driver for the Blueprint Oculus Driver to suit our needs. We really appreciate their work.
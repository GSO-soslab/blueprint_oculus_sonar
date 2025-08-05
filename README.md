# Blueprint Oculus Sonar ROS2 driver

This is a ROS2 metapackage including:
 * A ROS2 package **oculus_interfaces** containing the useful ROS messages definitions,
 * A ROS2 package **oculus_ros2** interfacing the driver messages with ROS2 topics,

This ROS2 metapackage was developed and tested using:<br>
* Ubuntu 24.04 LTS<br>
* ROS2 jazzy
* M750d Sonar
### Dependencies
`sudo apt-get install libboost-dev libboost-system-dev libboost-thread-dev ros-jazzy-cv-bridge`
### Messages
* Fan Shaped Image : "oculus/image" (sensor_msgs::msg::Image)
* Bin-Beam Image : "oculus/raw_image" (sensor_msgs::msg:Image)
* Pressure : "oculus/pressure" (sensor_msgs::msg::FluidPressure)
* Temperature : "oculus/temperature" (sensor_msgs::msg::Temperature)
* Depth : "oculus/depth/odometry" (nav_msgs::msg::Odometry)
* Status : "oculus/status" (oculus_interfaces::msg::OculusStatus)
* Ping : "oculus/ping"
(oculus_interfaces::msg::Ping)
### Installation
```sh
$ cd ~/YOUR_WS/src
$ git clone https://github.com/GSO-soslab/blueprint_oculus_sonar
$ cd blueprint_oculus_sonar
$ git checkout jazzy-devel
$ cd ../..
$ colcon build --packages-select oculus_interfaces oculus_ros2
```

### Launch

```sh
$ ros2 launch oculus_ros2 default.launch.py
```

### Troubleshooting
The sonar itself has a fixed IP address which may or may not be indicated on the box. To avoid bricking of the sonar by "lack of post-it", the sonar makes itself known on the network by broadcasting its own IP address. This library should always detect the IP of a plugged in Oculus sonar. You should make sure your own system configuration match the configuration of the sonar (i.e. your system and the sonar must be on the same subnet).

### Acknowledgement
This repository is modified version of [ENSTABretagneRobotics](https://github.com/ENSTABretagneRobotics/oculus_ros2) ROS2 driver for the Blueprint Oculus Driver to suit our needs. We really appreciate their work.
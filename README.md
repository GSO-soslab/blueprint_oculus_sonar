# Blueprint Oculus Sonar ROS1 driver
This driver will be integrated in our ALPHA AUV serials, please check [alpha_core](https://github.com/uri-ocean-robotics/alpha_core/tree/noetic-devel/external)
for other sensor drivers. This driver publish all ROS standard msg for sonar usage, besides the raw msg which directly publish the information from sonar.

**Standard msg rostopic**
  - Fan-shape image: rostopic("/image"), msg type(sensor_msgs::Image) 
  - raw bin-beam image: rostopic("/raw_image"), msg type(sensor_msgs::Image) 
  - IMU orientation: rostopic("/orientation"), msg type(geometry_msgs::QuaternionStamped) 
  - pressure: rostopic("/pressure"), msg type(sensor_msgs::FluidPressure) 

**Structure**
- [g3_log_ros](https://gitlab.com/apl-ocean-engineering/g3log_ros): log information for liboculus
- [liboculus](https://github.com/apl-ocean-engineering/liboculus): interface for blueprint oculus sonar from APL-UW
- blueprint_oculus_sonar: the ros1 driver


### Installzation
```sh
$ cd ~/YOUR_CATKIN_WS/src
$ git clone https://github.com/GSO-soslab/blueprint_oculus_sonar
$ cd blueprint_oculus_sonar
$ git submodule update --init --recursive
$ cd ../../
$ rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
$ catkin build -DCMAKE_BUILD_TYPE=Release -j$(nproc) blueprint_oculus_sonar
```

### Acknowledgement
We really appreciate APL-UW release their [oculus interface lib](https://github.com/apl-ocean-engineering/liboculus), our driver adapted parts of code from their [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver). We also adapted some code from [sonar_oculus](https://github.com/RobustFieldAutonomyLab/bluerov/tree/master/sonar_oculus) from Robust Field Autonomy Lab at SIT, thanks their work.

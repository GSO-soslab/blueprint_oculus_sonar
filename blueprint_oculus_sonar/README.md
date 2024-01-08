# blueprint_oculus_sonar
Blueprint Oculus Multibeam Forward-Looking Sonar (FLS) ROS 1 driver, tested with M750d.

### Installzation
```sh
$ cd ~/YOUR_CATKIN_WS/src
$ git clone https://gitlab.com/apl-ocean-engineering/g3log_ros 
$ git clone https://github.com/GSO-soslab/liboculus
$ git clone https://github.com/GSO-soslab/blueprint_oculus_sonar
$ cd ..
$ rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
$ catkin build -DCMAKE_BUILD_TYPE=Release -j$(nproc) blueprint_oculus_sonar
```

### Acknowledgement
We really appreciate APL-UW release their [oculus interface lib](https://github.com/apl-ocean-engineering/liboculus), our driver adapted parts of code from their [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver). We also adapted some code from [sonar_oculus](https://github.com/RobustFieldAutonomyLab/bluerov/tree/master/sonar_oculus) from Robust Field Autonomy Lab at SIT, thanks their work.

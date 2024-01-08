#include "g3log_ros/ROSLogSink.h"
#include "g3log_ros/g3logger.h"
#include "nodelet/loader.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "oculus_sonar_driver");

  libg3logger::G3Logger<ROSLogSink> log_worker("oculus_sonar_driver");

  nodelet::Loader nodelet(true);
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "blueprint_oculus/driver", remap, nargv);

  ros::spin();
  return 0;
}

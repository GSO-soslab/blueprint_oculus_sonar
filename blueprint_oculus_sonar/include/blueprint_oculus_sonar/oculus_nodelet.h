#pragma once

#include <dynamic_reconfigure/server.h>

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Header.h>

// Used to get sonar ping info
#include "liboculus/IoServiceThread.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarConfiguration.h"
#include "liboculus/StatusRx.h"

#include "blueprint_oculus_sonar/oculus_interface.h"

#include "blueprint_oculus_sonar/OculusSonarConfig.h"
#include <blueprint_oculus_sonar/OculusInfo.h>


namespace blueprint_oculus_sonar {

class OculusDriverRos1 : public nodelet::Nodelet {

public:
    OculusDriverRos1();

    virtual ~OculusDriverRos1();

    //! callback for getting data from the liboculus  
    template <typename Ping_t>
    void pingCallback(const Ping_t &ping);

    //! callback from dynamic reconfiguration 
    void configCallback(const blueprint_oculus_sonar::OculusSonarConfig &config,
                        uint32_t level);

private:

    //! ros nodelet initialization 
    void onInit() override;

    ros::Publisher sonar_image_pub_;

    ros::Publisher sonar_raw_pub_;

    ros::Publisher sonar_info_pub_;

    ros::Publisher raw_data_pub_;

    ros::Publisher orientation_pub_;

    ros::Publisher pressure_pub_;

    liboculus::IoServiceThread io_srv_;

    OculusInterface oculus_interface_;
    
    liboculus::StatusRx status_interface_;    

    std::string ip_address_;

    std::string frame_id_;

    liboculus::SonarConfiguration sonar_config_;

    typedef dynamic_reconfigure::Server<
        blueprint_oculus_sonar::OculusSonarConfig>
        ReconfigureServer;

    std::shared_ptr<ReconfigureServer> reconfigure_server_; 

    cv::Mat map_bb_x_, map_bb_y_;
    
    cv::Mat map_img_x_, map_img_y_;

    int num_bearings_, num_ranges_;

    int img_cols_, img_rows_;
};

}   // namespace blueprint_oculus_sonar
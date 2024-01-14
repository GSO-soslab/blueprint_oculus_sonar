#pragma once

#include <blueprint_oculus_sonar/OculusInfo.h>
#include "liboculus/Constants.h"
#include "liboculus/SimplePingResult.h"
#include "ros/ros.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace blueprint_oculus_sonar
{


double interpolateBin(const std::vector<double> &bearings, double bearing)
{
    // check, bearings_rad: [-1.5, ..., 1.5]
    if( (bearing < bearings.front()) || (bearing > bearings.back()) ) {
    //   printf("warning: bearing=%f!\n", bearing);
      return -1;
    }

    // find the prev and next index of bearings
    double bin;

    if(bearing == bearings.front()){
        bin = 0;
    }
    else if (bearing == bearings.back()) {  
        bin = bearings.size() - 1;
    }
    else {
        auto frame = std::find_if(bearings.begin(), bearings.end(),
                    [&](const auto& rad){return rad > bearing ;});
        // find the interval                    
        double begin_bearing = *(frame-1);
        double begin_bin = (frame-1)-bearings.begin();
        double end_bearing = *(frame);
        double end_bin = frame - bearings.begin();

        // interpolate
        double lambda = (bearing - begin_bearing) / (end_bearing - begin_bearing);
        bin = (1 - lambda) * begin_bin + lambda * end_bin;
    }

    return bin;
}

template <typename PingT>
void pingToPingCommon( 
    const liboculus::SimplePingResult<PingT> &ping,
    blueprint_oculus_sonar::OculusInfo& msg_info) {

    // TVG
    for (unsigned int i = 0; i < ping.gains().size(); i++) {
      msg_info.tvg.push_back(ping.gains().at(i));
    }    

    // Fields from OculusMessageHeader
    msg_info.src_device_id = ping.ping()->fireMessage.head.srcDeviceId;
    msg_info.dst_device_id = ping.ping()->fireMessage.head.dstDeviceId;
    msg_info.msg_id = ping.ping()->fireMessage.head.msgId;
    msg_info.msg_version = ping.ping()->fireMessage.head.msgVersion;
    msg_info.payload_size = ping.ping()->fireMessage.head.payloadSize;
    // msg_info.spare2 = ping.ping()->fireMessage.head.spare2;

    // ## Fields from OculusSimpleFireMessage / OculusSimpleFireMessage2
    msg_info.master_mode = ping.ping()->fireMessage.masterMode;
    //! TODO: Define ROS Msg ENUMs for the ping rate types
    msg_info.ping_rate = ping.ping()->fireMessage.pingRate;
    msg_info.network_speed = ping.ping()->fireMessage.networkSpeed;
    msg_info.gamma_correction = ping.ping()->fireMessage.gammaCorrection;
    msg_info.flags = ping.ping()->fireMessage.flags;
    msg_info.gain_percent = ping.ping()->fireMessage.gainPercent;
    msg_info.speed_of_sound = ping.ping()->fireMessage.speedOfSound;
    msg_info.salinity = ping.ping()->fireMessage.salinity;

    // Fields from OculusSimplePingResult / OculusSimplePingResult2
    msg_info.ping_id = ping.ping()->pingId;
    msg_info.status = ping.ping()->status;
    msg_info.frequency = ping.ping()->frequency;
    msg_info.temperature = ping.ping()->temperature;
    msg_info.pressure = ping.ping()->pressure;
    msg_info.speed_of_sound_used = ping.ping()->speedOfSoundUsed;
    msg_info.ping_start_time = ping.ping()->pingStartTime;

    //! TODO: Define ROS Msg ENUMs for the data size types?
    msg_info.data_size = ping.ping()->dataSize;
    msg_info.range_resolution = ping.ping()->rangeResolution;

    msg_info.n_ranges = ping.ping()->nRanges;
    msg_info.n_beams = ping.ping()->nBeams;

    // uint32 spare0 uint32 spare1 uint32 spare2 uint32 spare3
    msg_info.image_offset = ping.ping()->imageOffset;
    msg_info.image_size = ping.ping()->imageSize;
    msg_info.message_size = ping.ping()->messageSize;
}

template <typename PingT>
void pingToPingResult(
    const liboculus::SimplePingResult<PingT> &ping,
    blueprint_oculus_sonar::OculusInfo& msg_info,
    geometry_msgs::QuaternionStamped::Ptr& orientation,
    sensor_msgs::FluidPressure::Ptr& pressure) {
      
    pingToPingCommon(ping, msg_info);
}

template <>
void pingToPingResult<OculusSimplePingResult>(
    const liboculus::SimplePingResultV1 &ping,
    blueprint_oculus_sonar::OculusInfo& msg_info,
    geometry_msgs::QuaternionStamped::Ptr& orientation,
    sensor_msgs::FluidPressure::Ptr& pressure) {

    // get the common information
    pingToPingCommon(ping, msg_info);

    // get the V1 version information
    msg_info.range = ping.ping()->fireMessage.range;

    // package the pressure (convert from bar to Pascals)
    pressure = boost::make_shared<sensor_msgs::FluidPressure>();
    pressure->fluid_pressure = ping.ping()->pressure * 100000;
}

template <>
void pingToPingResult<OculusSimplePingResult2>(
    const liboculus::SimplePingResultV2& ping,
    blueprint_oculus_sonar::OculusInfo& msg_info,
    geometry_msgs::QuaternionStamped::Ptr& orientation,
    sensor_msgs::FluidPressure::Ptr& pressure) {

    // get the common information
    pingToPingCommon(ping, msg_info);

    // get the V2 version information
    msg_info.range = ping.ping()->fireMessage.rangePercent;
    msg_info.ext_flags = ping.ping()->fireMessage.extFlags;
    msg_info.heading = ping.ping()->heading;
    msg_info.pitch = ping.ping()->pitch;
    msg_info.roll = ping.ping()->roll;

    // package the orientation
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(ping.ping()->roll*M_PI/180, 
                          ping.ping()->pitch*M_PI/180, 
                          ping.ping()->heading*M_PI/180);
    orientation = boost::make_shared<geometry_msgs::QuaternionStamped>();
    orientation->quaternion = tf2::toMsg(quaternion_tf2);

    // package the pressure (convert from bar to Pascals)
    pressure = boost::make_shared<sensor_msgs::FluidPressure>();
    pressure->fluid_pressure = ping.ping()->pressure * 100000;    
}

template <typename PingT>
void pingToIntensity(
    const liboculus::SimplePingResult<PingT> &ping,
    cv::Mat& intensity) {

    // grab the raw intensity
    const int num_bearings = ping.ping()->nBeams;
    const int num_ranges = ping.ping()->nRanges;
    intensity = cv::Mat(cv::Size(num_bearings, num_ranges), CV_8UC1); // CV_8UC1, CV_8UC3,CV_32FC1

    // uint8_t test8=0;
    // uint16_t test16=0;
    // uint32_t test32=0;

    for (unsigned int r = 0; r < num_ranges; r++) {
        for (unsigned int b = 0; b < num_bearings; b++) {
            if (ping.dataSize() == 1) {
                const uint8_t data = ping.image().at_uint8(b, r);
                intensity.at<uchar>(r,b) = static_cast<float>(data) / UINT8_MAX * 255;

                // test8 = test8 < data ? data : test8;
            } 
            else if (ping.dataSize() == 2) {
                const uint16_t data = ping.image().at_uint16(b, r);
                intensity.at<uchar>(r,b) = static_cast<float>(data) / UINT16_MAX * 255;

                // test16 = test16 < data ? data : test16;
            } 
            else if (ping.dataSize() == 4) {
                const uint32_t data = ping.image().at_uint32(b, r);
                intensity.at<uchar>(r,b) = static_cast<float>(data) / UINT32_MAX * 255;

                // test32 = test32 < data ? data : test32;
            }
        }
    }

    //! TEST:
    // printf("test8=%d, test16=%d, test32=%d\n", test8, test16, test32);
    // printf("ping datasize=%d\n", ping.dataSize());
}

template <typename PingT>
void pingToImageConversion(
    const liboculus::SimplePingResult<PingT> &ping,
    int& bearings, int& ranges,
    cv::Mat& map_bb_x, cv::Mat& map_bb_y,
    int& img_cols, int& img_rows, 
    cv::Mat& map_img_x, cv::Mat& map_img_y) {

    // check the actual sonar image size 

    double new_height =  ping.ping()->rangeResolution * ping.ping()->nRanges;
    double new_width = sin((ping.bearings().back() - ping.bearings().front()) 
                            * M_PI / 180 / 2.0 ) * new_height * 2;
    auto new_cols = ceil(new_width / ping.ping()->rangeResolution);
    auto new_rows = ping.ping()->nRanges;

    double reverse_z = 1.0;

    // check if we need re-generate the map
    if(bearings != ping.ping()->nBeams ||
       ranges != ping.ping()->nRanges ||
       img_cols != new_cols ||
       img_rows != new_rows) {

        // save the info
        bearings = ping.ping()->nBeams;
        ranges = ping.ping()->nRanges;

        img_cols = new_cols;
        img_rows = new_rows;

        // map function for raw bin-beam image
        map_bb_x = cv::Mat::zeros(cv::Size(bearings, ranges), CV_32FC1);
        map_bb_y = cv::Mat::zeros(cv::Size(bearings, ranges), CV_32FC1);

        for( int i = 0; i < map_bb_x.rows; i++ )
        {
            for( int j = 0; j < map_bb_x.cols; j++ )
            {
                map_bb_x.at<float>(i, j) = (float)(j); 
                map_bb_y.at<float>(i, j) = (float)(map_bb_x.rows - i);
            }
        }   

        // bearing angle table
        std::vector<double> bearings_rad;     
        for(unsigned int i = 0; i < bearings; i ++) {
            bearings_rad.push_back(ping.bearings().at_rad(i));
        }

        // map function for actual sonar image
        map_img_x = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_32FC1);
        map_img_y = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_32FC1);        

        for( int i = 0; i < map_img_x.rows; i++ )
        {
            for( int j = 0; j < map_img_x.cols; j++ )
            {
                double x = ping.ping()->rangeResolution * (map_img_x.rows - i);
                double y = ping.ping()->rangeResolution * ( j - map_img_x.cols / 2.0 + 0.5);
                double bearing = atan2(y,x) * reverse_z; 
                double r = sqrt(pow(x,2) + pow(y,2));
                
                double interp_bin = interpolateBin(bearings_rad, bearing);

                map_img_x.at<float>(i, j) = (float)(interp_bin); 
                map_img_y.at<float>(i, j) = (float)(r / ping.ping()->rangeResolution);
            }
        }            
    }

}

} // namespace blueprint_oculus_sonar

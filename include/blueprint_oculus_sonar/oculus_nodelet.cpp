
#include <boost/asio.hpp>

#include "liboculus/Constants.h"
#include "liboculus/SonarConfiguration.h"

#include "blueprint_oculus_sonar/oculus_ping_conversion.h"
#include "blueprint_oculus_sonar/oculus_nodelet.h"
#include "blueprint_oculus_sonar/oculus_interface.h"
#include <blueprint_oculus_sonar/RawData.h>

namespace blueprint_oculus_sonar{

using liboculus::SimplePingResultV1;
using liboculus::SimplePingResultV2;

OculusDriverRos1::OculusDriverRos1()
    : Nodelet(),
      io_srv_(),
      oculus_interface_(io_srv_.context()),
      status_interface_(io_srv_.context()),
      sonar_config_(),
      reconfigure_server_(),
      num_bearings_(0), num_ranges_(0),
      img_cols_(0), img_rows_(0)
      {}

OculusDriverRos1::~OculusDriverRos1() {
    io_srv_.stop();
    io_srv_.join();
}

void OculusDriverRos1::onInit() {
    ros::NodeHandle n_(getMTNodeHandle());
    ros::NodeHandle pn_(getMTPrivateNodeHandle());

    NODELET_DEBUG_STREAM("Advertising topics in namespace " << n_.getNamespace());
    NODELET_DEBUG_STREAM("Private namespace would be:" << pn_.getNamespace());

    sonar_image_pub_ = n_.advertise<sensor_msgs::Image>("image", 10);

    sonar_raw_pub_ = n_.advertise<sensor_msgs::Image>("raw_image", 10);

    sonar_info_pub_ = n_.advertise<blueprint_oculus_sonar::OculusInfo>("info", 10);

    raw_data_pub_ = n_.advertise<blueprint_oculus_sonar::RawData>("raw_data", 100);

    orientation_pub_ = n_.advertise<geometry_msgs::QuaternionStamped>("orientation", 10);

    pressure_pub_ = n_.advertise<sensor_msgs::FluidPressure>("pressure", 10);

    pn_.getParam("ip_address", ip_address_);
    pn_.getParam("frame_id", frame_id_);
    NODELET_INFO_STREAM("Publishing data with frame = " << frame_id_);
    NODELET_INFO_STREAM("Will try to connect to sonar at: " << ip_address_);

    oculus_interface_.setRawPublisher(raw_data_pub_);

    oculus_interface_.setCallback<SimplePingResultV1>(
        std::bind(&OculusDriverRos1::pingCallback<SimplePingResultV1>, this,
                  std::placeholders::_1));

    oculus_interface_.setCallback<SimplePingResultV2>(
        std::bind(&OculusDriverRos1::pingCallback<SimplePingResultV2>, this,
                  std::placeholders::_1));

    // When the node connects, start the sonar pinging by sending
    // a OculusSimpleFireMessage current configuration.
    oculus_interface_.setOnConnectCallback(
        [&]() { oculus_interface_.sendSimpleFireMessage(sonar_config_); });
    printf("\n=========================\n Here init\n=========================\n");

    // It is not necessary to load any of the parameters controlled by
    // dynamic reconfigure, since dynamic reconfigure will read them from
    // the launch file and immediately publish an update message at launch.

    if (ip_address_ == "auto") {
      NODELET_INFO_STREAM("Attempting to auto-detect sonar");
      status_interface_.setCallback(
          [&](const liboculus::SonarStatus &status, bool is_valid) {
            if (!is_valid || oculus_interface_.isConnected()) return;
            NODELET_WARN_STREAM("Auto-detected IP:" << status.ipAddr());
            oculus_interface_.connect(status.ipAddr());
          });
    } else {
      NODELET_INFO_STREAM("Opening sonar at " << ip_address_);
      oculus_interface_.connect(ip_address_);
    }

    io_srv_.start();

    reconfigure_server_.reset(new ReconfigureServer(pn_));
    reconfigure_server_->setCallback(
        boost::bind(&OculusDriverRos1::configCallback, this, _1, _2));
}

template <typename Ping_t>
void OculusDriverRos1::pingCallback(const Ping_t &ping) {

    // regenerate the map function
    pingToImageConversion(
        ping, 
        num_bearings_, num_ranges_, map_bb_x_, map_bb_y_,
        img_cols_, img_rows_, map_img_x_, map_img_y_);

    // get header 
    //! TODO: how to get the sonar system time ?
    std_msgs::Header header;
    header.seq = ping.ping()->pingId;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;

    // get oculus sonar info
    blueprint_oculus_sonar::OculusInfo msg_info;
    geometry_msgs::QuaternionStamped::Ptr  msg_orientation_ptr;
    sensor_msgs::FluidPressure::Ptr  msg_pressure_ptr;
    pingToPingResult(ping, msg_info, msg_orientation_ptr, msg_pressure_ptr);

    // publish the sonar info
    msg_info.header = header;
    sonar_info_pub_.publish(msg_info);

    // publish the orientation if need
    if(msg_orientation_ptr != nullptr &&
       orientation_pub_.getNumSubscribers() != 0){
          msg_orientation_ptr->header = header;
          orientation_pub_.publish(msg_orientation_ptr);
    }
    
    // publish the pressure if need
    if(msg_pressure_ptr != nullptr && 
       pressure_pub_.getNumSubscribers() != 0) {
          msg_pressure_ptr->header = header;
          pressure_pub_.publish(msg_pressure_ptr);
       }

    // get oculus sonar raw intensity
    cv::Mat intensity;
    pingToIntensity(ping, intensity);

    // bin-beam image
    cv::Mat img_raw(intensity.size(), intensity.type());
    cv::remap(intensity, img_raw, map_bb_x_, map_bb_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    sensor_msgs::ImagePtr msg_img_raw = cv_bridge::CvImage(header, "mono8", img_raw).toImageMsg();
    sonar_raw_pub_.publish(msg_img_raw);

    // actual image
    cv::Mat img(cv::Size(img_cols_, img_rows_), intensity.type());
    cv::remap( intensity, img, map_img_x_, map_img_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
    sonar_image_pub_.publish(msg_img);

}

// Updates sonar parameters
void OculusDriverRos1::configCallback(
    const blueprint_oculus_sonar::OculusSonarConfig &config, 
    uint32_t level) {

    ROS_INFO_STREAM("Setting sonar range to " << config.range << " m");
    sonar_config_.setRange(config.range);

    ROS_INFO_STREAM("Setting gain to " << config.gain << " pct");
    sonar_config_.setGainPercent(config.gain);

    ROS_INFO_STREAM("Setting gamma to " << config.gamma);
    sonar_config_.setGamma(config.gamma);

    ROS_INFO_STREAM("Setting ping rate to ("
                    << config.ping_rate << "): "
                    << liboculus::PingRateToHz(config.ping_rate) << " Hz");
    switch (config.ping_rate) {
      case OculusSonar_Normal:
        sonar_config_.setPingRate(pingRateNormal);
        break;
      case OculusSonar_High:
        sonar_config_.setPingRate(pingRateHigh);
        break;
      case OculusSonar_Highest:
        sonar_config_.setPingRate(pingRateHighest);
        break;
      case OculusSonar_Low:
        sonar_config_.setPingRate(pingRateLow);
        break;
      case OculusSonar_Lowest:
        sonar_config_.setPingRate(pingRateLowest);
        break;
      case OculusSonar_Standby:
        sonar_config_.setPingRate(pingRateStandby);
        break;
      default:
        ROS_WARN_STREAM("Unknown ping rate " << config.ping_rate);
    }

    ROS_INFO_STREAM("Setting freq mode to "
                    << liboculus::FreqModeToString(config.freq_mode));
    switch (config.freq_mode) {
      case OculusSonar_LowFrequency:
        sonar_config_.setFreqMode(liboculus::OCULUS_LOW_FREQ);
        break;
      case OculusSonar_HighFrequency:
        sonar_config_.setFreqMode(liboculus::OCULUS_HIGH_FREQ);
        break;
      default:
        ROS_WARN_STREAM("Unknown frequency mode " << config.freq_mode);
    }

    sonar_config_.sendRangeAsMeters(config.send_range_as_meters)
        .setSendGain(config.send_gain)
        .setSimpleReturn(config.send_simple_return)
        .setGainAssistance(config.gain_assistance);

    if (config.num_beams == OculusSonar_256beams) {
      sonar_config_.use256Beams();
    } else {
      sonar_config_.use512Beams();
    }

    // Rather than trust that the .cfg and Oculus enums are the same,
    // do an explicit mapping
    ROS_INFO_STREAM("config.data_size: " << config.data_size);
    switch (config.data_size) {
      case OculusSonar_8bit:
        sonar_config_.setDataSize(dataSize8Bit);
        break;
      case OculusSonar_16bit:
        sonar_config_.setDataSize(dataSize16Bit);
        break;
      case OculusSonar_32bit:
        sonar_config_.setDataSize(dataSize32Bit);
        break;
      default:
        ROS_WARN_STREAM("Unknown data size " << config.data_size);
    }

    ROS_INFO_STREAM("Setting flags: 0x"
                    // << std::hex << std::setw(2) << std::setfill('0')
                    // << static_cast<unsigned int>(sonar_config_.flags()())
                    // << std::dec << std::setw(0)
                    << "\n send range in meters "
                    << sonar_config_.getSendRangeAsMeters()
                    << "\n            data size "
                    << liboculus::DataSizeToString(sonar_config_.getDataSize())
                    << "\n      send gain       " << sonar_config_.getSendGain()
                    << "\n      simple return   "
                    << sonar_config_.getSimpleReturn()
                    << "\n      gain assistance "
                    << sonar_config_.getGainAssistance()
                    << "\n      use 512 beams " << sonar_config_.get512Beams());

    // Update the sonar with new params
    if (oculus_interface_.isConnected()) {
      oculus_interface_.sendSimpleFireMessage(sonar_config_);
    }

}

} // namespace blueprint_oculus_sonar

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(blueprint_oculus_sonar::OculusDriverRos1, nodelet::Nodelet);

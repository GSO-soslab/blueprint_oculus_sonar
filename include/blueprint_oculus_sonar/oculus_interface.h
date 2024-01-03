#pragma once

#include <blueprint_oculus_sonar/RawData.h>

#include <vector>

#include "liboculus/DataRx.h"

namespace blueprint_oculus_sonar {

class OculusInterface : public liboculus::DataRx {

public:
    explicit OculusInterface(
        const liboculus::IoServiceThread::IoContextPtr &iosrv)
        : DataRx(iosrv), count_(0) {}

    ~OculusInterface() {}

    // Assume ros::Publishers are copiable
    void setRawPublisher(ros::Publisher pub) { raw_data_pub_ = pub; }

    void haveWritten(const std::vector<uint8_t> &bytes) override {
      doPublish(bytes, blueprint_oculus_sonar::RawData::DATA_OUT);
    }

    void haveRead(const std::vector<uint8_t> &bytes) override {
      doPublish(bytes, blueprint_oculus_sonar::RawData::DATA_IN);
    }

    void doPublish(const std::vector<uint8_t> &bytes, uint8_t direction) {
      if (bytes.size() == 0) return;

      blueprint_oculus_sonar::RawData raw_msg;

      raw_msg.header.seq = count_++;
      raw_msg.header.stamp = ros::Time::now();
      raw_msg.direction = direction;
      auto raw_size = bytes.size();
      raw_msg.data.resize(raw_size);
      memcpy(raw_msg.data.data(), bytes.data(), raw_size);
      raw_data_pub_.publish(raw_msg);
    }

    ros::Publisher raw_data_pub_;

    unsigned int count_;    
};

} // namespace blueprint_oculus_sonar
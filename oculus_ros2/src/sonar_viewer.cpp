/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, ENSTA-Bretagne
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <oculus_ros2/sonar_viewer.hpp>

SonarViewer::SonarViewer(rclcpp::Node* node)
  : image_publisher_(node->create_publisher<sensor_msgs::msg::Image>("oculus/image", 10)),
    raw_image_publisher_(node->create_publisher<sensor_msgs::msg::Image>("oculus/raw_image", 10)),
    node_(node),
    map_bb_x_(),
    map_bb_y_(),
    map_img_x_(),
    map_img_y_(),
    num_bearings_(0),
    num_ranges_(0),
    img_cols_(0),
    img_rows_(0)
{}

SonarViewer::~SonarViewer() {}

void SonarViewer::publishFan(const oculus_interfaces::msg::Ping& ros_ping_msg) const {
  // const int offset = ping->ping_data_offset(); // TODO(hugoyvrn)
  const int offset = -16;  // quick fix TODO(hugoyvrn, why 229?)

  publishFan(
      ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.master_mode, ros_ping_msg.header);
}

void SonarViewer::publishFan(const oculus::PingMessage::ConstPtr& ping, const std::string& frame_id) const {
  std_msgs::msg::Header header;
  header.stamp = oculus::toMsg(ping->timestamp());
  header.frame_id = frame_id;

  if (!ping->has_gains()) {
    RCLCPP_WARN(node_->get_logger(), "Gains are not send by the sonar. The conic image view is wrong.");
  }
  publishFan(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->master_mode(), header);
}

void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<uint8_t>& ping_data,
    const int& master_mode,
    const std_msgs::msg::Header& header) const {
  const int step = width + SIZE_OF_GAIN_;
  // const float theta_shift = 270;
  const int mat_encoding = CV_8U;
  const char* ros_image_encoding = sensor_msgs::image_encodings::MONO8;

  const double bearing =
      (master_mode == 1) ? LOW_FREQUENCY_BEARING_APERTURE_ * M_PI / 180 : HIGHT_FREQUENCY_BEARING_APERTURE_ * M_PI / 180;
  const float bearing_ratio = 2 * bearing / width;
  const int negative_height = static_cast<int>(std::floor(height * std::sin(-bearing)));
  const int positive_height = static_cast<int>(std::ceil(height * std::sin(bearing)));
  const int image_width = positive_height - negative_height;
  const int origin_width = abs(negative_height);  // x coordinate of the origin
  const cv::Size image_size(image_width, height);
  cv::Mat map(image_size, CV_32FC2);
  cv::parallel_for_(cv::Range(0, map.total()), [&](const cv::Range& range) {
    for (auto i = range.start; i < range.end; i++) {
      int y = i / map.cols;
      int x = i % map.cols;

      // Calculate range and bearing of this pixel from origin
      const float dx = x - origin_width;
      const float dy = map.rows - y;

      const float range = sqrt(dx * dx + dy * dy);
      const float bearing_x_y = atan2(dx, dy);

      float xp = range;
      // Linear interpolation, TODO: use a better interpolation method
      float yp = (bearing_x_y + bearing) / bearing_ratio;

      map.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  });

  cv::Mat source_map_1, source_map_2;
  cv::convertMaps(map, cv::Mat(), source_map_1, source_map_2, CV_16SC2);

  cv::Mat sonar_mat_data(height, step, mat_encoding);  // Note that the width is 'step' to include gain data
  // Copy the data including gain data
  for (int i = 0; i < height; ++i)
    std::copy(ping_data.begin() + offset + i * step, ping_data.begin() + offset + (i + 1) * step, sonar_mat_data.ptr<uint8_t>(i));

  // Now remove the gain data from sonar_mat_data
  cv::Mat sonar_mat_data_without_gain(height, width, mat_encoding);
  for (int i = 0; i < height; ++i)
    std::copy(sonar_mat_data.ptr<uint8_t>(i) + SIZE_OF_GAIN_, sonar_mat_data.ptr<uint8_t>(i) + step,
        sonar_mat_data_without_gain.ptr<uint8_t>(i));

  cv::Mat out = cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();
  cv::remap(sonar_mat_data_without_gain.t(), out, source_map_1, source_map_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT,
      cv::Scalar(std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()));

  // Publish sonar conic image
  sensor_msgs::msg::Image msg;
  cv_bridge::CvImage(header, ros_image_encoding, out).toImageMsg(msg);
  image_publisher_->publish(msg);
}

void SonarViewer::publishRaw(const oculus_interfaces::msg::Ping& ros_ping_msg, const std::string& frame_id){
  
  pingToImageConversion(
        ros_ping_msg, 
        num_bearings_, num_ranges_, map_bb_x_, map_bb_y_,
        img_cols_, img_rows_, map_img_x_, map_img_y_);
  
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = ros_ping_msg.header.stamp;
  const char* ros_image_encoding = sensor_msgs::image_encodings::MONO8;

  cv::Mat intensity;
  pingToIntensity(ros_ping_msg, intensity);

  cv::Mat img_raw(intensity.size(), intensity.type());
  cv::remap(intensity, img_raw, map_bb_x_, map_bb_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
  
  // Publish Bin-Beam image
  sensor_msgs::msg::Image bin_beam_img_msg;
  cv_bridge::CvImage(header, ros_image_encoding, img_raw).toImageMsg(bin_beam_img_msg);
  raw_image_publisher_->publish(bin_beam_img_msg);
}

void SonarViewer::pingToIntensity(
    const oculus_interfaces::msg::Ping& ros_ping_msg,
    cv::Mat& intensity) {

    // grab the raw intensity
    const int num_bearings = ros_ping_msg.n_beams;
    const int num_ranges = ros_ping_msg.n_ranges;
    intensity = cv::Mat(cv::Size(num_bearings, num_ranges), CV_8UC1); // CV_8UC1, CV_8UC3,CV_32FC1

    // uint8_t test8=0;
    // uint16_t test16=0;
    // uint32_t test32=0;

    for (unsigned int r = 0; r < num_ranges; r++) {
        for (unsigned int b = 0; b < num_bearings; b++) {
          if (ros_ping_msg.sample_size == 1) {
            size_t index = r * ros_ping_msg.step + b;
            uint8_t data = ros_ping_msg.ping_data[index];

            intensity.at<uchar>(r, b) = static_cast<float>(data) / UINT8_MAX * 255;
            }
          else if (ros_ping_msg.sample_size == 2) {
              size_t index = r * ros_ping_msg.step + b * 2;
              uint16_t data = ros_ping_msg.ping_data[index] |
                            (ros_ping_msg.ping_data[index + 1] << 8);

              intensity.at<uchar>(r, b) = static_cast<float>(data) / UINT16_MAX * 255;
          }
          else if (ros_ping_msg.sample_size == 4) {
                size_t index = r * ros_ping_msg.step + b * 4;
                uint32_t data =  ros_ping_msg.ping_data[index] |
                                (ros_ping_msg.ping_data[index + 1] << 8) |
                                (ros_ping_msg.ping_data[index + 2] << 16) |
                                (ros_ping_msg.ping_data[index + 3] << 24);

                intensity.at<uchar>(r, b) = static_cast<float>(data) / UINT32_MAX * 255;
            }
        }
    }

    //! TEST:
    // printf("test8=%d, test16=%d, test32=%d\n", test8, test16, test32);
    // printf("ping datasize=%d\n", ping.dataSize());
}

void SonarViewer::pingToImageConversion(
    const oculus_interfaces::msg::Ping& ros_ping_msg,
    int& bearings, int& ranges,
    cv::Mat& map_bb_x, cv::Mat& map_bb_y,
    int& img_cols, int& img_rows, 
    cv::Mat& map_img_x, cv::Mat& map_img_y) {

    // check the actual sonar image size 

    double new_height =  ros_ping_msg.range_resolution * ros_ping_msg.n_ranges;
    double new_width = sin((ros_ping_msg.bearings.back() - ros_ping_msg.bearings.front()) 
                        * 0.01 * M_PI / 180.0 / 2.0) * new_height * 2;
    auto new_cols = ceil(new_width / ros_ping_msg.range_resolution);
    auto new_rows = ros_ping_msg.n_ranges;

    double reverse_z = 1.0;

    // check if we need re-generate the map
    if(bearings != ros_ping_msg.n_beams ||
       ranges != ros_ping_msg.n_ranges ||
       img_cols != new_cols ||
       img_rows != new_rows) {

        // save the info
        bearings = ros_ping_msg.n_beams;
        ranges = ros_ping_msg.n_ranges;

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
        for (size_t i = 0; i < ros_ping_msg.bearings.size(); ++i) {
            double angle_rad = ros_ping_msg.bearings[i] * 0.01 * M_PI / 180.0;
            bearings_rad.push_back(angle_rad);
        }

        // map function for actual sonar image
        map_img_x = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_32FC1);
        map_img_y = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_32FC1);        

        for( int i = 0; i < map_img_x.rows; i++ )
        {
            for( int j = 0; j < map_img_x.cols; j++ )
            {
                double x = ros_ping_msg.range_resolution * (map_img_x.rows - i);
                double y = ros_ping_msg.range_resolution * ( j - map_img_x.cols / 2.0 + 0.5);
                double bearing = atan2(y,x) * reverse_z; 
                double r = sqrt(pow(x,2) + pow(y,2));
                
                double interp_bin = interpolateBin(bearings_rad, bearing);

                map_img_x.at<float>(i, j) = (float)(interp_bin); 
                map_img_y.at<float>(i, j) = (float)(r / ros_ping_msg.range_resolution);
            }
        }            
    }

}

double SonarViewer::interpolateBin(const std::vector<double> &bearings, double bearing)
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
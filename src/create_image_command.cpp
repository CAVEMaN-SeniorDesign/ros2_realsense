#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RealSensePicture_Command : public rclcpp::Node
{
public:
  RealSensePicture_Command()
  : Node("image_publisher_command"), count_(0)
  {
    this->declare_parameter<std::string>("topic" , "/joy/buttons");
    this->declare_parameter<int>("exposure_time" , 15);

    exposure_time_ = this->get_parameter("exposure_time").as_int();

    std::string color_dir = "/root/images_Color";
    std::string depth_dir = "/root/images_Depth";

    // Creating the images directories
    mkdir(color_dir.c_str(), 0777);
    mkdir(depth_dir.c_str(), 0777);

    // Get time
    const auto p1 = std::chrono::system_clock::now();

    std::string timeStart = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

    color_time_dir = color_dir + "/" + timeStart;
    depth_time_dir = depth_dir + "/" + timeStart;
    
    // Create time directory
    mkdir(color_time_dir.c_str(), 0777);
    mkdir(depth_time_dir.c_str(), 0777);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      this->get_parameter("topic").as_string(), 10, std::bind(&RealSensePicture_Command::topic_callback, this, _1));
  }

private:

  void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
  {
      std::ofstream csv;

      csv.open(filename);

      //    std::cout << "Writing metadata to " << filename << endl;
      csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

      // Record all the available metadata attributes
      for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
      {
          if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
          {
              csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                  << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
          }
      }

      csv.close();
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    
    if (strcmp(msg->data.c_str(), "1"))
    {

      // Declare depth colorizer for pretty visualization of depth data
      rs2::colorizer color_map;

      // Declare RealSense pipeline, encapsulating the actual device and sensors
      rs2::pipeline pipe;
      // Start streaming with default recommended configuration
      pipe.start();

      // Capture a couple frames to give autoexposure, etc. a chance to settle
      for (auto i = 0; i < exposure_time_; ++i) pipe.wait_for_frames();

      // Wait for the next set of frames from the camera. Now that autoexposure, etc.
      // has settled, we will write these to disk
      for (auto&& frame : pipe.wait_for_frames())
      {
          // We can only save video frames as pngs, so we skip the rest
          if (auto vf = frame.as<rs2::video_frame>())
          {
              //auto stream = frame.get_profile().stream_type();
              // Use the colorizer to get an rgb image for the depth stream
              if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

              // Write images to disk
              if (vf.get_profile().stream_name() == "Color"){
                  std::stringstream png_file;
                  png_file << color_time_dir << "/image_exposure" << exposure_time_ << "-" << vf.get_profile().stream_name() << count_ << ".png";
                  stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                              vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                  std::cout << "Saved " << png_file.str() << std::endl;
              }
              else{
                  std::stringstream png_file;
                  png_file << depth_time_dir << "/image_exposure" << exposure_time_ << "-" << vf.get_profile().stream_name() << count_ << ".png";
                  stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                              vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                  std::cout << "Saved " << png_file.str() << std::endl;
              }
                  
              // Record per-frame metadata for UVC streams
              //std::stringstream csv_file;
              //csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
              //        << "-metadata.csv";
              //metadata_to_csv(vf, csv_file.str());
          }
      }

      count_++;
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
  std::string color_time_dir;
  std::string depth_time_dir;
  int exposure_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSensePicture_Command>());
  rclcpp::shutdown();
  return 0;
}

/*
catch(const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
*/
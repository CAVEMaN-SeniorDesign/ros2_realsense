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
//#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RosbagControl : public rclcpp::Node
{
public:
  RosbagControl()
  : Node("rosbag_control"), count_(0), mode_(false)
  {
    this->declare_parameter<std::string>("topic" , "/camera/camera/color/image_raw");

    color_dir = "/root/images_Color";
    depth_dir = "/root/images_Depth";

    // Creating the images directories
    mkdir(color_dir.c_str(), 0777);
    mkdir(depth_dir.c_str(), 0777);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      this->get_parameter("topic").as_string(), 10, std::bind(&RosbagControl::topic_callback, this, _1));
  }

private:

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    
    //if (msg->buttons[3] == 1)
    //{
        if (mode_ == false){

            // Get time
            const auto p1 = std::chrono::system_clock::now();

            std::string timeStart = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

            color_time_dir = color_dir + "/" + timeStart;
            
            // Create time directory
            mkdir(color_time_dir.c_str(), 0777);

            std::string command = "ros2 bag record -o " + color_time_dir + "/imageBag " + this->get_parameter("topic").as_string();
            int returnCode = system(command.c_str());
            if(returnCode){
                RCLCPP_INFO(this->get_logger(), "Started recording topic %s", command.c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Failed to start recording topic %s", command.c_str());
            }


            mode_ = true;
        }
        else{
            std::string command = "kill -KILL -17802";
            int returnCode = system(command.c_str());
            if(returnCode){
                RCLCPP_INFO(this->get_logger(), "Stopped recording topic %s", command.c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Failed to stop recording topic %s", command.c_str());
            }
            mode_ = true;
        }
    //}
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
  std::string color_time_dir;
  std::string color_dir;
  std::string depth_dir;
  bool mode_; // mode_ = false record rosbag, mode_ = true finsih recording and convert to pngs
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosbagControl>());
  rclcpp::shutdown();
  return 0;
}
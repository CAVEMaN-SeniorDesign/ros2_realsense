#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

using std::placeholders::_1;

class ConvertImageRaw: public rclcpp::Node
{
    public:
        ConvertImageRaw()
        : Node("image_converter"), count_(0)
        {
            this->declare_parameter<std::string>("topic" , "/camera/camera/color/image_raw");

            std::string color_dir = "/root/images_Color";

            // Creating the images directories
            mkdir(color_dir.c_str(), 0777);

            // Get time
            const auto p1 = std::chrono::system_clock::now();

            std::string timeStart = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

            color_time_dir = color_dir + "/" + timeStart;
            
            // Create time directory
            mkdir(color_time_dir.c_str(), 0777);

            // Get compression parameters
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);

            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("topic").as_string(), 10, std::bind(&ConvertImageRaw::topic_callback, this, _1));
        }

    private:
        
        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to publish image.");
                return;
            }


            bool success; 
            std::string image_name = color_time_dir + "/" + "color_image" + std::to_string(count_) + ".png";

            success = cv::imwrite(image_name.c_str(), cv_ptr->image, compression_params);

            if(success){
                std::cout << "Successful print to " << image_name.c_str() << std::endl;
            }
            else{
                std::cout << "Unsuccessful print" << std::endl;
            }

            count_++;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        std::string color_time_dir;
        int count_;
        std::vector<int> compression_params;

};

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvertImageRaw>());
    rclcpp::shutdown();
    return 0;
}
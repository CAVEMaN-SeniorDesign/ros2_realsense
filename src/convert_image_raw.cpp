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
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

class ConvertImageRaw: public rclcpp::Node
{
    public:
        ConvertImageRaw()
        : Node("image_converter"), count_color_(0), count_depth_(0), previous_joy_(0)
        {
            this->declare_parameter<std::string>("topic_color" , "/camera/camera/color/image_raw");
            this->declare_parameter<std::string>("topic_depth" , "/camera/camera/depth/image_rect_raw");
            this->declare_parameter<std::string>("topic_joy" , "/joy");
            this->declare_parameter<bool>("use_controller", false);

            
            color_dir = "/root/images_Color";
            depth_dir = "/root/images_Depth";

            // Creating the images directories
            mkdir(color_dir.c_str(), 0777);
            mkdir(depth_dir.c_str(), 0777);

            if (this->get_parameter("use_controller").as_bool() == true){
                mode_ = false;
            }
            else{

                // Get time
                const auto p1 = std::chrono::system_clock::now();

                std::string timeStart = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

                color_time_dir = color_dir + "/" + timeStart;
                depth_time_dir = depth_dir + "/" + timeStart;

                // Create time directory
                mkdir(color_time_dir.c_str(), 0777);
                mkdir(depth_time_dir.c_str(), 0777);
                
                mode_ = true;
            }

            // Get compression parameters
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            
            subscription_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
                this->get_parameter("topic_joy").as_string(), 10, std::bind(&ConvertImageRaw::get_joy, this, _1));

            subscription_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
                this->get_parameter("topic_depth").as_string(), 10, std::bind(&ConvertImageRaw::publish_depth, this, _1));

            subscription_color_ = this->create_subscription<sensor_msgs::msg::Image>(
                this->get_parameter("topic_color").as_string(), 10, std::bind(&ConvertImageRaw::publish_color, this, _1));
        }

    private:
        
        void get_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->buttons[3]);

            if ((msg->buttons[3] == 1) && (previous_joy_ == 0)){
                if(mode_ == false){
                    // Get time
                    const auto p1 = std::chrono::system_clock::now();

                    std::string timeStart = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

                    color_time_dir = color_dir + "/" + timeStart;
                    depth_time_dir = depth_dir + "/" + timeStart;

                    // Create time directory
                    mkdir(color_time_dir.c_str(), 0777);
                    mkdir(depth_time_dir.c_str(), 0777);
                    count_color_ = 0; 
                    count_depth_ = 0;
                    mode_ = true;
                }
                else{
                    mode_ = false;
                }
            }

            previous_joy_ = msg->buttons[3];
        }

        void publish_depth(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            if (mode_ == true){
                cv_bridge::CvImageConstPtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                }
                catch (cv_bridge::Exception& e)
                {
                    RCLCPP_INFO(this->get_logger(), "Failed to publish depth image.");
                    return;
                }

                bool success; 
                std::string image_name = depth_time_dir + "/" + "depth_image" + std::to_string(count_depth_) + ".jpeg";

                success = cv::imwrite(image_name.c_str(), cv_ptr->image, compression_params);

                if(success){
                    std::cout << "Successful print to " << image_name.c_str() << std::endl;
                }
                else{
                    std::cout << "Unsuccessful depth print" << std::endl;
                }

                count_depth_++;
            }
        }
        
        void publish_color(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            if (mode_ == true){
                cv_bridge::CvImageConstPtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    RCLCPP_INFO(this->get_logger(), "Failed to publish color image.");
                    return;
                }

                bool success; 
                std::string image_name = color_time_dir + "/" + "color_image" + std::to_string(count_color_) + ".jpeg";

                success = cv::imwrite(image_name.c_str(), cv_ptr->image, compression_params);

                if(success){
                    std::cout << "Successful print to " << image_name.c_str() << std::endl;
                }
                else{
                    std::cout << "Unsuccessful color print" << std::endl;
                }

                count_color_++;
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_color_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy_;
        std::string color_dir;
        std::string depth_dir;
        std::string color_time_dir;
        std::string depth_time_dir;
        int count_color_, count_depth_, previous_joy_;
        std::vector<int> compression_params;
        bool mode_;  // mode_ = 0 do not publish pictures, mode_ = 1 publish


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvertImageRaw>());
    rclcpp::shutdown();
    return 0;
}
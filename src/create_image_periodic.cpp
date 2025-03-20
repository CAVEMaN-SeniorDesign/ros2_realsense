#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class RealSensePicture_Periodic: public rclcpp::Node
{
    public:
        RealSensePicture_Periodic()
        : Node("image_publisher_periodic"), count_(0)
        {
            this->declare_parameter<int>("period_ms" , 500);
            std::chrono::milliseconds time_ms{this->get_parameter("period_ms").as_int()};
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
                time_ms, std::bind(&RealSensePicture_Periodic::timer_callback, this));
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
        
        void timer_callback()
        {
            // Declare depth colorizer for pretty visualization of depth data
            rs2::colorizer color_map;

            // Declare RealSense pipeline, encapsulating the actual device and sensors
            rs2::pipeline pipe;
            // Start streaming with default recommended configuration
            pipe.start();

            // Capture 30 frames to give autoexposure, etc. a chance to settle
            //for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

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
                        png_file << "images_Color/rs-save-to-disk-output-" << vf.get_profile().stream_name() << count_ << ".png";
                        stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                                    vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                        std::cout << "Saved " << png_file.str() << std::endl;
                    }
                    else{
                        std::stringstream png_file;
                        png_file << "images_Depth/rs-save-to-disk-output-" << vf.get_profile().stream_name() << count_ << ".png";
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

        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        //std::string period_;
        int period_int;
};
// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char * argv[]) try
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSensePicture_Periodic>());
    rclcpp::shutdown();
    return 0;
}

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
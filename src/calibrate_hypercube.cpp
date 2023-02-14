#include <ros/ros.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "hsi_driver/DataCube.h"
#include <cv_bridge/cv_bridge.h>

// Calibration routine for hyperspectral datacubes
// Authored by Nathaniel Hanson

class DatacubeCorrecter
{
public:
    DatacubeCorrecter(ros::NodeHandle *nh)
    {
        this->corrected_pub = nh->advertise<hsi_driver::DataCube>("calibrated_cubes", 10);
        this->raw_sub = nh->subscribe("data_cubes", 1000, &DatacubeCorrecter::correctCubes, this);

        ros::param::get("~camera_model", this->model);
        if (this->model.compare("imec") == 0)
        {
            // Load the camera intrinsics
        }
        else if (this->model.compare("ximea") == 0)
        {
            // Load the camera intrinsics
        }
    }

    ~DatacubeCorrecter()
    {
    }

private:
    ros::Publisher corrected_pub;
    ros::Subscriber raw_sub;
    std::string model;

    void correctCubes(hsi_driver::DataCube msg) {


    }

    // void stream_camera_feed()
    // {
    //     this->ret_val = cameraStart(this->camera);
        
    //     int width = this->frame.format.width;
    //     int height = this->frame.format.height;
    //     cv::Mat image = cv::Mat::zeros(width, height, CV_32F);
    //     cv_bridge::CvImage cv_image;
    //     cv_image.encoding = "32FC1";    
    //     sensor_msgs::Image msg;
    //     while (ros::ok()) {
            
    //         cv_image.header.stamp = ros::Time::now();
    //         this->ret_val =cameraTrigger(this->camera);
    //         this->ret_val = cameraAcquireFrame(this->camera, &this->frame);
    //         for(int row = 0; row < height; row++) {
    //             for(int col = 0; col < width; col++) {
    //                 image.at<float>(row, col) = this->frame.pp_data[row][col];
    //             }
    //         }
    //         cv_image.image = image;
    //         cv_image.toImageMsg(msg);
    //         this->raw_pub.publish(msg);
    //         ros::spinOnce();
    //     }
    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cube_correcter", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    DatacubeCorrecter nc = DatacubeCorrecter(&nh);
    ros::spin();
}
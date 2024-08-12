#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include "hyper_drive/DataCube.h"
#include "hyper_drive/MultipleDataCubes.h"
#include "hyper_drive/npy.hpp"
#include <chrono>

// Global variables to reduce computation
Eigen::Matrix3d h_ximea_2_img;
Eigen::Matrix3d h_imec_2_img;
float scale = 1.0f/3.0f;
Eigen::Matrix3d scale_matrix;
Eigen::Matrix3d h_ximea_2_img_scaled;
Eigen::Matrix3d h_imec_2_img_scaled;
cv::Mat h_ximea_2_img_cv;
cv::Mat h_imec_2_img_cv;

// Check if mask already computed
bool mask = false;
ros::Publisher registerd_cube_publisher;

cv::Mat apply_homography(cv::Mat rgb_img, cv::Mat homography, cv::Mat hyper_cube) {
    cv::Size output_size(rgb_img.cols, rgb_img.rows);
    std::cout << output_size << "\n";
    cv::Mat warped_image;
    cv::warpPerspective(hyper_cube, warped_image, homography, output_size);
    return warped_image;
}

cv::Mat generate_registered_cube(cv::Mat h_ximea_2_img, cv::Mat h_imec_2_img, cv::Mat im_dest,
                                cv::Mat ximea_cube, cv::Mat imec_cube) {
    cv::Mat ximea_registered = apply_homography(im_dest, h_ximea_2_img, ximea_cube);
    cv::Mat imec_registered = apply_homography(im_dest, h_imec_2_img, imec_cube);
    cv::Mat merged_result;
    std::vector<cv::Mat> merge_data = {ximea_registered, imec_registered};
    cv::merge(merge_data, merged_result);
    return merged_result;
}

void hyperspectral_data_callback(const hyper_drive::MultipleDataCubes::ConstPtr &msg) {
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    auto t1 = high_resolution_clock::now();
    std::cout << "[+] Msg Received" << std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->im, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat image = cv_ptr->image;
    hyper_drive::DataCube ximea = msg->cubes[0];
    hyper_drive::DataCube imec = msg->cubes[1];
    int ximea_channels = (int) ximea.data.size() / (ximea.width * ximea.height);
    int imec_channels = (int) imec.data.size() / (imec.width * imec.height);
    
    cv::Mat imec_matrix(imec.height, imec.width, CV_16SC(imec_channels), imec.data.data());
    cv::Mat ximea_matrix(ximea.height, ximea.width, CV_16SC(ximea_channels), ximea.data.data());
    if (!false) {

    }
    // cv::Mat mask = apply_homography(image, h_ximea_2_img_cv, ximea_matrix);
    cv::Mat registered_cube = generate_registered_cube(h_ximea_2_img_cv, h_imec_2_img_cv, image, ximea_matrix, imec_matrix);
    cv::Mat registered_cube_resized;
    hyper_drive::DataCube pub_msg;
    // cv::Size resize_height(registered_cube.rows * registered_cube.cols * registered_cube.channels(), 1);
    // cv::resize(registered_cube, registered_cube_resized, resize_height);
    std::vector<int16_t> flattened_cube(registered_cube.begin<int16_t>(), registered_cube.end<int16_t>());

    // std::vector<int16_t> pub_data(registered_cube_resized.ptr<float>(), registered_cube_resized.ptr<float>() + registered_cube_resized.cols);
    
    pub_msg.data = flattened_cube;
    pub_msg.height = registered_cube.rows;
    pub_msg.width = registered_cube.cols;
    registerd_cube_publisher.publish(pub_msg);
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_double.count() << "ms\n";
    return;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "register_hsi");
    ros::NodeHandle nh;
    // Setting global variables
    h_ximea_2_img << 5.041012041909088914, 0.1509927559331974145, -126.7157229620469678,
        -0.1943643338890190253, 4.995535765615253254, 429.0650394879531859,
        -0.00001818947125201643539, 0.00001467280150152038717, 1;
    h_imec_2_img << 8.181859562627073146e+00, 4.378020216586171742e-02, 2.627367514514044728e+02,
                -1.216783595443193811e-01, 8.194135859694744894e+00, 2.042626789827522487e+02,
                -5.832502150206408064e-05, -5.955423000811831294e-06, 1.000000000000000000e+00;

    scale_matrix << scale, 0, 0, 0, scale, 0, 0, 0, 1;
    h_ximea_2_img_scaled = (scale_matrix * h_ximea_2_img * scale_matrix.inverse()).transpose();
    h_imec_2_img_scaled = (scale_matrix * h_imec_2_img * scale_matrix.inverse()).transpose();
    h_imec_2_img_cv = cv::Mat(h_imec_2_img_scaled.rows(), h_imec_2_img_scaled.cols(), CV_64FC1, h_imec_2_img_scaled.data());
    h_ximea_2_img_cv = cv::Mat(h_ximea_2_img_scaled.rows(), h_ximea_2_img_scaled.cols(), CV_64FC1, h_ximea_2_img_scaled.data());
    // std::vector<int> dimensions = {215, 407, 24};
    // npy::npy_data<double> d = npy::read_npy<double>("/home/river/hyper_drive_data/VNIR_RAW/olin_collection_05_16_23_parcel_b_v2_68.npy");
    ros::Subscriber synchronous_cube_subscriber = nh.subscribe("synchronous_cubes", 1000, hyperspectral_data_callback);
    registerd_cube_publisher = nh.advertise<hyper_drive::DataCube>("registered_cube", 1000);
    // std::vector<double> data = d.data;
    // std::vector<unsigned long> shape = d.shape;
    // cv::Mat result(215, 407, CV_64FC(24), data.data());
    // std::cout << result << data.data()[data.size()-1] << std::endl;
    ros::spin();

    return 0;
}
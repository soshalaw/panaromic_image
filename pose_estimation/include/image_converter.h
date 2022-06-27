
#include <array>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ocam_functions.h"
#include "pose_estimate.h"

static const std::string OPENCV_WINDOW = "Image window";

class image_converter
{
public:

    image_converter();

    ~image_converter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);  //function to convert data to cv::Mat

private :
    cv::Mat img, frame, new_image;
    std::array<double,3> c, c_new;
    int img_counter = 1;
    std::vector<int> id;

    double theta_min;
    double theta_max;
    double delta_min;
    double delta_max;
    double omega;
    double phi;

    double marker_len;
    double fps;

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distcoefs = cv::Mat::zeros(5, 1, CV_64FC1);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;

    ocam_functions panoramic;
    pose_estimate estimate = pose_estimate(camera_matrix, distcoefs, phi, omega);

};

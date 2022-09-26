
#include <array>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ocam_functions.h"

static const std::string OPENCV_WINDOW = "Image window";

class image_converter
{

public:

    image_converter();

    ~image_converter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);  //function to convert data to cv::Mat

    void panorama_merged();


private :
    cv::Mat img, frame;
    std::array<double,3> c;
    int img_counter = 1;

    //calibration data camera 01 & 02
    std::vector<double> invpol;
    std::vector<double> pol;
    double yc;
    double xc;

    double c_;
    double d;
    double e;

    //horizontal vertical fov projected image mode
    double theta_min, theta_max, alpha_min, alpha_max;

    //horizontal fov panorama mode
    double delta_min, delta_max;

    // multiple projected images
    int num_img; //number of projected images

    double beta_min, beta_max; // horizontal fov

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    ocam_functions panorama;

};

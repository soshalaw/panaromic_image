#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ocam_functions.h"
#include "pose_estimate.h"

static const std::string OPENCV_WINDOW = "Image window";

class bridge
{

private :
    cv::Mat img, new_image, resized_image;
    std::array<double,3>  c;
    int img_counter = 1;
    std::vector<int> id = {0};

    double marker_len = 0.2;
    double theta_min =  -CV_PI/6;
    double theta_max =  CV_PI/6;
    double delta_min = CV_PI/2 + CV_PI/8;
    double delta_max = CV_PI/2 + 3*CV_PI/8;
    double omega = CV_PI/4;
    double phi = CV_PI/2;

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distcoefs = cv::Mat::zeros(5, 1, CV_64FC1);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub, omega_sub;
    paranomic panaromic;
    arucoMarker estimate = arucoMarker(camera_matrix, distcoefs, phi, omega);

public:

    bridge()
    : it_(nh_)
    {
        image_sub = it_.subscribe("usb_cam/image_raw", 1 , &bridge::imageCb, this);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~bridge()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)  //function to convert data to cv::Mat
    {
        cv::Mat frame;
        cv_bridge::CvImagePtr cv_ptr;

        ROS_INFO_STREAM(omega);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }

        img = cv_ptr->image;

        frame = panaromic.slice(img, c, theta_min, theta_max, delta_min, delta_max);

        panaromic.def_camera_matrix(camera_matrix, distcoefs, theta_min, theta_max, delta_min, delta_max);

        new_image = estimate.pose_marker(frame, c, id, marker_len);

        cv::imshow(OPENCV_WINDOW, new_image);

        int k = cv::waitKey(1);

        if (k%256 == 32)
        {
            std::string name = "/home/tue-me-minicar-laptop-02/internship/camera_calibration/camera_01/validation_/validation_9/open_cv_img" + std::to_string(img_counter) + ".png" ;
            cv::imwrite(name, new_image);
            img_counter++;
        }
    }
};

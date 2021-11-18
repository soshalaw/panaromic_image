#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ocam_functions.h"

static const std::string OPENCV_WINDOW = "Image window";

class bridge
{
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub;

public:

        cv::Mat img;

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

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exceprion: %s",e.what());
            return;
        }

        img = cv_ptr->image;
    }

    void show_image()
    {
        cv::Mat frame, frame_new, frame_new1, frame_flipped;

        if(img.empty()){
            break;
            std::cout << "Frame empty" << std::endl;
        }

        frame_new = panaromic.panaroma3(img);

        cv::imshow("Frame",img);

        cv::waitKey(3);
    }
};

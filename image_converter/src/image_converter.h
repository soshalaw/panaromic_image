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
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub;
        arucoMarker estimate;

public:

        cv::Mat img, new_image, resized_image;
        int l = 1;
        double c[3];
        
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
        paranomic panaromic;

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

        frame = panaromic.slice(img);

        new_image = estimate.pose(frame);

        double width = new_image.size().width;
        double height = new_image.size().height;

        cv::resize(new_image, resized_image, cv::Size(width*1.5, height*1.5));

        cv::imshow(OPENCV_WINDOW, resized_image);

        int k = cv::waitKey(1);

        if (k%256 == 32)
        {
            std::string str = "/home/soshala/internship/camera_calibration/camera_01/data_12_512_60_45/opencv_frame_" +std::to_string(l)+ ".png";
            cv::imwrite(str,new_image);
            l++;
        }

    }

};

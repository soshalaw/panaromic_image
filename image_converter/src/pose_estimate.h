#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class arucoMarker
{
public:
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -1.32802856e-01, -3.02795796e+1, 1.19753409e-01, 1.19284710e-01, 3.76373075e+02);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    ros::NodeHandle nh;
    ros::Publisher pub;

    arucoMarker()
    {
        camera_matrix.at<double>(0,0) = 2.61091273e+03;
        camera_matrix.at<double>(0,2) = 5.06812801e+02;
        camera_matrix.at<double>(1,1) = 1.76595876e+03;
        camera_matrix.at<double>(1,2) = 3.63574526e+02;

        //pub = nh.advertise<>
    }


    cv::Mat pose(cv::Mat image)
    {
        cv::Mat new_image;
        image.copyTo(new_image);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(new_image, Dictionary, corners, ids );

        if (ids.size() > 0)
        {
            std::cout << "marker detected" << std::endl;
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);            
        }
        else
        {
            std::cout << "marker not detected" << std::endl;
        }

        return new_image;
    }

};

#endif // POSE_ESTIMATE_H

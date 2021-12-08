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
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -0.08155198, -0.14501826, -0.06165639, -0.0117106, 0.07766493);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    ros::NodeHandle nh;
    ros::Publisher pub;

    arucoMarker()
    {
        camera_matrix.at<double>(0,0) = 416.05541356;
        camera_matrix.at<double>(0,2) = 365.55906184;
        camera_matrix.at<double>(1,1) = 425.61843071;
        camera_matrix.at<double>(1,2) = 93.6874598;

        //pub = nh.advertise<>
    }


    cv::Mat pose(cv::Mat image)
    {
        cv::Mat new_image;
        image.copyTo(new_image);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(new_image, Dictionary, corners, ids, ,camera_matrix, distcoefs);

        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);
        }

        return new_image;
    }

};

#endif // POSE_ESTIMATE_H

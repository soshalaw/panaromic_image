#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class arucoMarker
{
public:

    ros::NodeHandle nh;
    ros::Publisher pub;

    arucoMarker()
    {
        camera_matrix.at<double>(0,0) = 590.6194;
        camera_matrix.at<double>(0,2) = 519.6931;
        camera_matrix.at<double>(1,1) = 598.2359;
        camera_matrix.at<double>(1,2) = 285.4423;

    }


    cv::Mat pose(cv::Mat image)
    {
        image.copyTo(new_image);

        parameters->adaptiveThreshWinSizeMax = 400;
        parameters->minDistanceToBorder = 0;


        cv::aruco::detectMarkers(new_image, Dictionary, corners, ids, parameters, rejected_corners);

        if (ids.size() > 0)
        {
            //cv::aruco::drawDetectedMarkers(new_image, corners, ids);
        }

        return new_image;
    }

private:

    cv::Mat new_image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -0.09252277, 0.17653478, -0.01388358, 0.00633439, -0.11124765);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

};

#endif // POSE_ESTIMATE_H

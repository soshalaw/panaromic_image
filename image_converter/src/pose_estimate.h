#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "tf/tf.h"

class arucoMarker
{
private:

    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    ros::NodeHandle nh;
    ros::Publisher pub;
    cv::Mat gray_image, new_image;

public:

    arucoMarker(){}

    cv::Mat pose(cv::Mat image)
    {
        image.copyTo(new_image);
        cv::cvtColor(new_image, gray_image, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(gray_image, Dictionary, corners, ids);

        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);
        }

        return new_image;
    }

};

#endif // POSE_ESTIMATE_H

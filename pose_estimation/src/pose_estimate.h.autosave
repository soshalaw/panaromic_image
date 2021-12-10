 #ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"

class arucoMarker
{
public:

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -1.32802856e-01, -3.02795796e+1, 1.19753409e-01, 1.19284710e-01, 3.76373075e+02);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    ros::NodeHandle nh;
    ros::Publisher pub;

    geometry_msgs::Pose position;


    tf::Transform transform;
    tf::Quaternion q;

    arucoMarker()
    {
        camera_matrix.at<double>(0,0) = 2.61091273e+03;
        camera_matrix.at<double>(0,2) = 5.06812801e+02;
        camera_matrix.at<double>(1,1) = 1.76595876e+03;
        camera_matrix.at<double>(1,2) = 3.63574526e+02;

        pub = nh.advertise<geometry_msgs::Pose>("pose",5);


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
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.1, camera_matrix, distcoefs, rvecs, tvecs); //0.1 length of the markers to be detected

            for (int i = 0; i < ids.size(); i++ )
            {
                cv::aruco::drawAxis(new_image, camera_matrix, distcoefs, rvecs[i], tvecs[i], 0.1); //0.1 length of the drawn axis
                broadcast(rvecs[i], tvecs[i]);

            }

        }

        return new_image;
    }

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs)
    {
        static tf::TransformBroadcaster br;

        transform.setOrigin(tf::Vector3(tvecs[0], tvecs[1], tvecs[2]));
        q.setRPY(rvecs[0], rvecs[1], rvecs[2]);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Marker", "Camera"));

    }

};

#endif // POSE_ESTIMATE_H

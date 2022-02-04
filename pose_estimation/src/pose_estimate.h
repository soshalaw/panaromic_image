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
private:

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    //512p 60_45
    //cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -0.427771206145061, 0.633996195883629, -0.045465437991085, 0.039523036767817, -0.404909636477268);
    //768p 90_60
    //cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << 0.00652234, -0.00916935, -0.04434304, 0.00916721, -0.0195544);
    cv::Mat distcoefs = cv::Mat::zeros(5, 1, CV_64FC1);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    ros::NodeHandle nh;
    ros::Publisher pub;

    geometry_msgs::Pose position;

    tf::Transform transform;
    tf::Quaternion q;

    int blur_window_size;
    double x_, y_, z_, mod;
    double x, y, z, cp_x, cp_y, cp_z, p_x, p_y, p_z, x_tr, y_tr, z_tr, x_pr, y_pr, abs_x, abs_y, abs_z, abs_dist, est_dist;
    double perimeter;           // definition of the peremeter to compute the number of pixel per square of the marker
    double pixels_per_sqr;      // number of pixels per square of the marker
    double marker_sqr = 6;          // number of squares in the marker

public:

    arucoMarker()
    {
        camera_matrix.at<double>(0,0) = 565.5172;
        camera_matrix.at<double>(0,2) = 256;
        camera_matrix.at<double>(1,1) = 565.5172;
        camera_matrix.at<double>(1,2) = 126.0153;

        blur_window_size = 7;

        pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
    }

    cv::Mat pose_marker(cv::Mat image, double c[3], std::vector<int> id, double marker_len)
    {
        cv::Mat new_image, gray_img;
        image.copyTo(new_image);
        cv::cvtColor(new_image, gray_img, cv::COLOR_BGR2GRAY);

        std::vector<int> ids, ids_to_est;
        std::vector<std::vector<cv::Point2f>> corners, corners_to_est;
        std::vector<cv::Vec3d> rvecs, tvecs;

        int k = 0; //counter for the array of markers to estimate the pose

        cv::aruco::detectMarkers(gray_img, Dictionary, corners, ids);

        if (ids.size() > 0)
        {
            for (int i = 0; i < id.size(); i++)
            {
                for (int j = 0; j < ids.size(); j++)
                {
                    if (id[i] == ids[j])
                    {
                        ids_to_est.push_back(ids[j]);
                        corners_to_est.push_back(corners[j]);
                        k++;
                    }
                }
            }

            if(ids_to_est.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(new_image, corners_to_est, ids_to_est);
                cv::aruco::estimatePoseSingleMarkers(corners_to_est, marker_len, camera_matrix, distcoefs, rvecs, tvecs);

                for (int i = 0; i < ids_to_est.size(); i++ )
                {
                    perimeter = cv::arcLength(corners_to_est[i], true);
                    pixels_per_sqr = perimeter/marker_sqr;

                    if (pixels_per_sqr < 0)
                    {
                        cv::aruco::drawAxis(new_image, camera_matrix, distcoefs, rvecs[i], tvecs[i], 0.1); //0.1 length of the drawn axis
                        broadcast(rvecs[i], tvecs[i], c);
                        ROS_INFO_STREAM("Pixels per square: " << pixels_per_sqr);
                    }else
                    {
                        ROS_INFO_STREAM("Marker out of range");
                    }
                }
            }
        }

        return new_image;
    }

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs, double c[3])
    {
        cp_x = c[0];
        cp_y = c[1];
        cp_z = c[2];
        
        static tf::TransformBroadcaster br;      
        
        x_ = tvecs[0];
        y_ = tvecs[1];
        z_ = tvecs[2];

        x_pr = x_/z_;
        y_pr = y_/z_;

        x_tr = cp_y*x_pr + cp_x*cp_z*y_pr + cp_x;
        y_tr = -cp_x*x_pr + cp_y*cp_z*y_pr + cp_y;
        z_tr = -(cp_y*cp_y + cp_x*cp_x)*y_ + cp_z;

        mod = sqrt(x_tr*x_tr + y_tr*y_tr + z_tr*z_tr);

        x = x_tr/mod;
        y = y_tr/mod;
        z = z_tr/mod;
        
        p_x = x*z_;
        p_y = y*z_;
        p_z = z*z_;

        est_dist = sqrt((p_x*p_x + p_y*p_y + p_z*p_z));

        transform.setOrigin(tf::Vector3(p_x, p_y, p_z));
        q.setRPY(rvecs[0], rvecs[1], rvecs[2]);
        q.normalize();
        transform.setRotation(q);

        geometry_msgs::Pose pose;

        pose.position.x = p_x;
        pose.position.y = p_y;
        pose.position.z = p_z;

        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.z = q[2];
        pose.orientation.w = q[3];

        pub.publish(pose);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fisheyed_camera", "fisheyed_marker_pred"));

        ROS_INFO_STREAM(" ets x: "<< p_x << " est y: " << p_y << " est z: " << p_z );
    }
};

#endif // POSE_ESTIMATE_H

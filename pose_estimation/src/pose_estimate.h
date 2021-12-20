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
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -0.09252277, 0.17653478, -0.01388358, 0.00633439, -0.11124765);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    ros::NodeHandle nh;
    ros::Publisher pub;

    geometry_msgs::Pose position;


    tf::Transform transform;
    tf::Transform transform2;
    tf::Quaternion q;

    int blur_window_size;

    arucoMarker()
    {
        //resolution 512
        camera_matrix.at<double>(0,0) = 283.430152308643;
        camera_matrix.at<double>(0,2) = 261.212883329824;
        camera_matrix.at<double>(1,1) = 282.882284500171;
        camera_matrix.at<double>(1,2) = 147.952673195220;

        //resolution 1024
        /*camera_matrix.at<double>(0,0) = 590.619400577650;
        camera_matrix.at<double>(0,2) = 519.693135481507;
        camera_matrix.at<double>(1,1) = 598.235920789867;
        camera_matrix.at<double>(1,2) = 285.442343211893;*/


        blur_window_size = 7;

        pub = nh.advertise<geometry_msgs::Pose>("pose",5);
    }


    cv::Mat pose(cv::Mat image, double c[3])
    {
        cv::Mat new_image, gray_img;
        image.copyTo(new_image);
        //cv::cvtColor(new_image, gray_img, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::GaussianBlur(new_image, gray_img, cv::Size(blur_window_size, blur_window_size), 0, 0);

        cv::aruco::detectMarkers(gray_img, Dictionary, corners, ids );

        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(gray_img, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.1, camera_matrix, distcoefs, rvecs, tvecs); //0.1 length of the markers to be detected

            for (int i = 0; i < ids.size(); i++ )
            {
                cv::aruco::drawAxis(gray_img, camera_matrix, distcoefs, rvecs[i], tvecs[i], 0.1); //0.1 length of the drawn axis
                broadcast(rvecs[i], tvecs[i], c);
            }

        }

        return gray_img;
    }

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs, double c[3])
    {
        
        double cp_x = c[0];
        double cp_y = c[1];
        double cp_z = c[2];
        
        static tf::TransformBroadcaster br;

        transform.setOrigin(tf::Vector3(tvecs[0], tvecs[1], tvecs[2]));
        q.setRPY(rvecs[0], rvecs[1], rvecs[2]);
        transform.setRotation(q);
        
        double x_ = transform.inverse().getOrigin().x();
        double y_ = transform.inverse().getOrigin().y();
        double z_ = transform.inverse().getOrigin().z();

        double x_tr = -cp_y*x_ - cp_x*cp_z*y_ + cp_x;
        double y_tr = cp_x*x_ - cp_y*cp_z*y_ + cp_y;
        double z_tr = -(-cp_y*cp_y - cp_x*cp_x)*y_ + cp_z;

        double mod = sqrt(x_tr*x_tr + y_tr*y_tr + z_tr*z_tr);

        double x = x_tr/mod;
        double y = y_tr/mod;
        double z = z_tr/mod;
        
        double p_x = x*z_;
        double p_y = y*z_;
        double p_z = z*z_;


        transform2.setOrigin(tf::Vector3(p_x, p_y, p_z));
        transform2.setRotation(transform.inverse().getRotation());

        br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "fisheyed_camera", "fisheyed_marker_pred"));

    }

};

#endif // POSE_ESTIMATE_H

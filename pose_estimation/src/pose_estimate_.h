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
    cv::Mat distcoefs = (cv::Mat_<double>(5 , 1) << -0.09252277, 0.17653478, -0.01388358, 0.00633439, -0.11124765);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber abs_cam;
    ros::Subscriber abs_mrkr;

    geometry_msgs::Pose position;

    tf::Transform transform;
    tf::Transform transform2;
    tf::Quaternion q;

    int blur_window_size;
    double cam_x, cam_y, cam_z, mrkr_x, mrkr_y, mrkr_z, x_, y_, z_, mod, error, accuracy;
    double x, y, z, cp_x, cp_y, cp_z, p_x, p_y, p_z, x_tr, y_tr, z_tr, abs_x, abs_y, abs_z, abs_dist, est_dist;

public:

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


    cv::Mat pose_marker(cv::Mat image, double c[3])
    {
        cv::Mat new_image, gray_img;
        image.copyTo(new_image);
        cv::cvtColor(new_image, gray_img, cv::COLOR_BGR2GRAY);
<<<<<<< HEAD
=======
        cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(Dictionary);
>>>>>>> fe36b50864467e05b1fe0ec1758fa035225210fc

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

<<<<<<< HEAD
        //cv::GaussianBlur(new_image, gray_img, cv::Size(blur_window_size, blur_window_size), 0, 0);
=======
        //cv::GaussianBlur(gray_img, gray_img, cv::Size(blur_window_size, blur_window_size), 0, 0);
>>>>>>> fe36b50864467e05b1fe0ec1758fa035225210fc

        cv::aruco::detectMarkers(inputImage, board.dictionary, markerCorners, markerIds);
        cv::aruco::detectMarkers(gray_img, Dictionary, corners, ids );

        if(ids.size() > 0) {
            cv::Vec3d rvec, tvec;
            int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix, distcoefs, rvec, tvec);
        }


        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.2, camera_matrix, distcoefs, rvecs, tvecs); //0.1 length of the markers to be detected

            for (int i = 0; i < ids.size(); i++ )
            {
                cv::aruco::drawAxis(new_image, camera_matrix, distcoefs, rvecs[i], tvecs[i], 0.1); //0.1 length of the drawn axis
                broadcast(rvecs[i], tvecs[i], c);
            }

        }

        return new_image;
<<<<<<< HEAD
    }

    cv::Mat pose_board(cv::Mat image, double c[3])
    {
        cv::Mat new_image, grey_image;
        image.copyTo(new_image);
        cv::cvtColor(new_image, grey_image, cv::COLOR_BGR2GRAY);

        cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(2, 2, 0.04, 0.01, Dictionary);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(grey_image, Dictionary, corners, ids);

        if(ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(new_image, corners, ids);
            cv::Vec3d rvecs, tvecs;
            int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix, distcoefs, rvecs, tvecs);

            if (valid > 0)
            {
                cv::aruco::drawAxis(new_image, camera_matrix, distcoefs, rvecs, tvecs, 0.1);
                broadcast(rvecs, tvecs, c);
            }

        }

        return new_image;

    }

    void update_cam_pose(const geometry_msgs::PoseStamped& msg)
    {
        cam_x = msg.pose.position.x;
        cam_y = msg.pose.position.y;
        cam_z = msg.pose.position.z;
    }

    void update_mrkr_pose(const geometry_msgs::PoseStamped& msg)
    {
        mrkr_x = msg.pose.position.x;
        mrkr_y = msg.pose.position.y;
        mrkr_z = msg.pose.position.z;
=======
>>>>>>> fe36b50864467e05b1fe0ec1758fa035225210fc
    }

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs, double c[3])
    {
        cp_x = c[0];
        cp_y = c[1];
        cp_z = c[2];
        
        static tf::TransformBroadcaster br;

        transform.setOrigin(tf::Vector3(tvecs[0], tvecs[1], tvecs[2]));
        q.setRPY(rvecs[0], rvecs[1], rvecs[2]);
        transform.setRotation(q);
        
        x_ = transform.inverse().getOrigin().x();
        y_ = transform.inverse().getOrigin().y();
        z_ = transform.inverse().getOrigin().z();

<<<<<<< HEAD
        x_tr = -cp_y*x_ - cp_x*cp_z*y_ + cp_x;
        y_tr = cp_x*x_ - cp_y*cp_z*y_ + cp_y;
        z_tr = -(-cp_y*cp_y - cp_x*cp_x)*y_ + cp_z;

        mod = sqrt(x_tr*x_tr + y_tr*y_tr + z_tr*z_tr);

        x = x_tr/mod;
        y = y_tr/mod;
        z = z_tr/mod;
=======
        double x_tr = -cp_y*x_ - cp_x*cp_z*y_ + cp_x*z_;
        double y_tr = cp_x*x_ - cp_y*cp_z*y_ + cp_y*z_;
        double z_tr = -(-cp_y*cp_y - cp_x*cp_x)*y_ + cp_z*z_;

        double x_pr = x_tr/z_tr;
        double y_pr = y_tr/z_tr;
        double z_pr = 1;

        double mod = sqrt(x_pr*x_pr + y_pr*y_pr + z_pr*z_pr);

        double x = x_pr/mod;
        double y = y_pr/mod;
        double z = z_pr/mod;
>>>>>>> fe36b50864467e05b1fe0ec1758fa035225210fc
        
        p_x = x*z_;
        p_y = y*z_;
        p_z = z*z_;

        est_dist = sqrt((p_x*p_x + p_y*p_y + p_z*p_z));

        abs_cam = nh.subscribe("/vrpn_client_node/fisheyed_camera/pose", 1000, &arucoMarker::update_cam_pose, this);
        abs_mrkr = nh.subscribe("/vrpn_client_node/fisheyed_marker/pose", 1000, &arucoMarker::update_mrkr_pose, this);

        if (abs_cam && abs_mrkr)
        {
            abs_x = mrkr_x - cam_x;
            std::cout << abs_x << std::endl;
            abs_y = mrkr_y - cam_y;
            abs_z = mrkr_z - cam_z;

            abs_dist = sqrt((abs_x*abs_x + abs_y*abs_y + abs_z*abs_z));

            error = abs(abs_dist - est_dist);
            accuracy = (error/abs_dist)*100;

           ROS_INFO_STREAM("Error: "<< error << " Accuracy: " << accuracy << " Absolute_dist :" << abs_dist);

        }

        transform2.setOrigin(tf::Vector3(p_x, p_y, p_z));
        transform2.setRotation(transform.inverse().getRotation());

        br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "fisheyed_camera", "fisheyed_marker_pred"));

    }

};

#endif // POSE_ESTIMATE_H

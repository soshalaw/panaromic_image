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

    cv::Mat distcoefs = cv::Mat::zeros(5, 1, CV_64FC1);
    cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    ros::NodeHandle nh;
    ros::Publisher pub;

    tf::Transform transform;
    tf::Quaternion q;

    double x, y, z, cp_x, cp_y, cp_z, p_x, p_y, p_z, x_tr, y_tr, z_tr, x_pr, y_pr, z_pr, mod, modz_x, modz_y, modx, mody;
    double x_, y_, z_, z_x, z_y, z_z, omega, phi;

    double perimeter;      //perimeter of the marker in pixels
    double pixels_per_sqr;    // number of pixels in a square of the marker
    double sqr_numbr = 6;        //number of squares in the marker

    cv::Mat camera_matrix;
    tf::Matrix3x3 m;

public:

    arucoMarker(cv::Mat Camera_matrix, double Phi, double Omega)
    {
        camera_matrix = Camera_matrix;
        pub = nh.advertise<geometry_msgs::Pose>("pose",1000);
        phi = Phi;
        omega = Omega;
    }

    cv::Mat pose_marker(cv::Mat image, double c[3], std::vector<int> id, double marker_len)
    {
        cv::Mat new_image, gray_img;
        image.copyTo(new_image);
        cv::cvtColor(new_image, gray_img, cv::COLOR_BGR2GRAY);

        std::vector<int> ids, ids_to_est;
        std::vector<std::vector<cv::Point2f>> corners, corners_to_est;
        std::vector<cv::Vec3d> rvecs, tvecs;

        cp_x = c[0];
        cp_y = c[1];
        cp_z = c[2];

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
                    pixels_per_sqr = perimeter/sqr_numbr;
                    cv::aruco::drawAxis(new_image, camera_matrix, distcoefs, rvecs[i], tvecs[i], 0.1); //0.1 length of the drawn axis
                    broadcast(rvecs[i], tvecs[i]);

                    ROS_INFO_STREAM("Pixel density : " << pixels_per_sqr);
                }
            }          
        }

        return new_image;
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
                broadcast(rvecs, tvecs);
            }
        }
        return new_image;
    }

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs)
    {
        static tf::TransformBroadcaster br;
        geometry_msgs::Pose pose;

        //transformation from the virtual camera frame to sensor frame
           
        x_pr = tvecs[0]/tvecs[2];
        y_pr = tvecs[1]/tvecs[2];
        z_pr = tvecs[2];

        modx = sqrt(cp_y*cp_y + cp_x*cp_x);
        mody = sqrt((cp_x*cp_z)*(cp_x*cp_z) + (cp_y*cp_z)*(cp_y*cp_z) + (cp_y*cp_y + cp_x*cp_x)*(cp_y*cp_y + cp_x*cp_x));

        x_tr = cp_y*x_pr/modx + cp_x*cp_z*y_pr/mody + cp_x;
        y_tr = -cp_x*x_pr/modx + cp_y*cp_z*y_pr/mody + cp_y;
        z_tr = -(cp_y*cp_y + cp_x*cp_x)*y_pr/mody + cp_z;

        mod = sqrt(x_tr*x_tr + y_tr*y_tr + z_tr*z_tr);

        x_ = x_tr/mod;
        y_ = y_tr/mod;
        z_ = z_tr/mod;
        
        p_x = x_*z_pr;
        p_y = y_*z_pr;
        p_z = z_*z_pr;

        if (omega == 0 & phi == 0)
        {
            x = p_x;
            y = p_y;
            z = p_z;
        }else
        {
            z_x = sin(phi)*sin(omega);
            z_y = cos(phi)*sin(omega);
            z_z = cos(omega);

            modz_x = sqrt(z_y*z_y + z_x*z_x);
            modz_y = sqrt((z_x*z_z)*(z_x*z_z) + (z_y*z_z)*(z_y*z_z) + (z_y*z_y + z_x*z_x)*(z_y*z_y + z_x*z_x));

            x = -z_y*p_x/modz_x - z_x*z_z*p_y/modz_y + z_x*p_z;
            y = z_x*p_x/modz_x - z_y*z_z*p_y/modz_y + z_y*p_z;
            z = (z_y*z_y + z_x*z_x)*p_y/modz_y + z_z*p_z;
        }

        //transformation from sensor frame to camera frame



        transform.setOrigin(tf::Vector3(x, y, z));
        get_orientation(x, y, z, rvecs);
        m.setRotation(q);
        transform.setRotation(q);

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.y = q[2];
        pose.orientation.w = q[3];

        pub.publish(pose);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fisheyed_camera", "fisheyed_marker_pred"));
    }

    void get_orientation(double x, double y, double z, cv::Vec3d rvec)
    {
        cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64FC1);
        //get the rotation matrix of the virtual camera ref frame

        double cam_roll = asin(z/sqrt(y*y + z*z));
        double cam_pitch = asin(z/sqrt(x*x + z*z));
        double cam_yaw = asin(y/sqrt(x*x + y*y));

        tf::Quaternion q;
        q.setRPY(cam_roll, cam_pitch, cam_yaw);
        q.normalize();

        tf::Matrix3x3 cam_m(q);

        //get the rotation matrix of the marker related to the virtual camera frame
        cv::Rodrigues(rvec, rot_mat);

        tf::Matrix3x3 rot_mat_(rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                               rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                               rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));

        m = cam_m*rot_mat_;
    }
};

#endif // POSE_ESTIMATE_H

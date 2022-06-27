#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include <array>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"

class pose_estimate
{

private:

    cv::Ptr<cv::aruco::Dictionary> Dictionary;

    ros::NodeHandle nh;
    ros::Publisher pub;

    tf::Transform transform;
    tf::Quaternion q;

    double x, y, z, cp_x, cp_y, cp_z, p_x, p_y, p_z, x_tr, y_tr, z_tr, x_pr, y_pr, z_pr, mod, modz_x, modz_y, modx, mody;
    double x_, y_, z_, z_x, z_y, z_z, z_1, y_1, x_1, omega, phi;

    double perimeter;      //perimeter of the marker in pixels
    double pixels_per_sqr;    // number of pixels in a square of the marker
    double sqr_numbr;        //number of squares in the marker

    cv::Mat camera_matrix;
    cv::Mat distcoefs;
    tf::Matrix3x3 m;

public:

    pose_estimate(cv::Mat Camera_matrix, cv::Mat Distcoefs, double Phi, double Omega);

    cv::Mat pose_marker(cv::Mat image, std::array<double, 3> c, std::vector<int> id, double marker_len);

    void broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs);

    void get_orientation(double x, double y, double z, cv::Vec3d rvec);

};

#endif // POSE_ESTIMATE_H

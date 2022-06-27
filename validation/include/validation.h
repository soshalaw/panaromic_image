#ifndef VALIDATION_H
#define VALIDATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class validation
{

private:
    double cam_x, cam_y, cam_z, mrkr_x, mrkr_y, mrkr_z, est_x, est_y, est_z, mod;
    double mrkr_qx, mrkr_qy, mrkr_qz, mrkr_qw;
    double error_x, accuracy_x, error_y, accuracy_y, error_z, accuracy_z;
    double roll, pitch, yaw;
    double abs_x, abs_y, abs_z, abs_relx, abs_rely, abs_relz;
    double camera_qx, camera_qy, camera_qz, camera_qw, cam_roll, cam_pitch, cam_yaw;
    double dist, est_dist, error_dist, accuracy_dist;

    ros::NodeHandle nh;
    ros::Subscriber abs_cam;
    ros::Subscriber est_mrkr;
    ros::Subscriber abs_mrkr;
    ros::Subscriber camera_pose;

public:
    validation();

    void update_cam_pose(const geometry_msgs::PoseStamped& msg);

    void update_mrkr_pose(const geometry_msgs::PoseStamped& msg);

    void update_mrkr_pose_est(const geometry_msgs::Pose& msg);

    void update_camera_pose(const geometry_msgs::PoseStamped& msg);

    void validate_results();
};
#endif // VALIDATION_H

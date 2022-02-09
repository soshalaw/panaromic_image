#ifndef VALIDATION_H
#define VALIDATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


class validation
{

private:
    double cam_x, cam_y, cam_z, mrkr_x, mrkr_y, mrkr_z, est_x, est_y, est_z, mod;
    double error_x, accuracy_x, error_y, accuracy_y, error_z, accuracy_z;
    double abs_x, abs_y, abs_z;

    ros::NodeHandle nh;
    ros::Subscriber abs_cam;
    ros::Subscriber est_mrkr;
    ros::Subscriber abs_mrkr;

public:
    validation() {

            abs_cam = nh.subscribe("/vrpn_client_node/fisheyed_camera/pose", 1000, &validation::update_cam_pose, this);
            abs_mrkr = nh.subscribe("/vrpn_client_node/fisheyed_marker/pose", 1000, &validation::update_mrkr_pose, this);
            est_mrkr = nh.subscribe("/pose", 1000, &validation::update_mrkr_pose_est, this);
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

    }

    void update_mrkr_pose_est(const geometry_msgs::Pose& msg)
    {
        est_x = msg.position.x;
        est_y = msg.position.y;
        est_z = msg.position.z;

        validate_results();
    }

    void validate_results()
    {
        if (abs_cam && abs_mrkr)
        {
            abs_y = -(mrkr_x - cam_x);
            abs_x = mrkr_y - cam_y;
            abs_z = mrkr_z - cam_z;

            error_x = abs(abs_x - est_x);
            accuracy_x = 100 - abs(error_x/abs_x)*100;

            error_y = abs(abs_y - est_y);
            accuracy_y = 100 - abs(error_y/abs_y)*100;

            error_z = abs(abs_z - est_z);
            accuracy_z = 100 - abs(error_z/abs_z)*100;

            ROS_INFO_STREAM(" Error x: "<< error_x << " Accuracy x: " << accuracy_x );
            ROS_INFO_STREAM(" Error y: "<< error_y << " Accuracy y: " << accuracy_y );
            ROS_INFO_STREAM(" Error z: "<< error_z << " Accuracy z: " << accuracy_z );

            ROS_INFO_STREAM(" abs x: "<< abs_x << " abs y: " << abs_y << " abs z: " << abs_z );
            ROS_INFO_STREAM(" ets x: "<< est_x << " est y: " << est_y << " est z: " << est_z );
        }
    }

};
#endif // VALIDATION_H

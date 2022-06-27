#include "validation.h"


validation::validation() {

        //camera_pose = nh.subscribe("/vrpn_client_node/camera/pose", 1000, &validation::update_camera_pose, this);
        abs_cam = nh.subscribe("/vrpn_client_node/fisheyed_camera/pose", 1000, &validation::update_cam_pose, this);
        abs_mrkr = nh.subscribe("/vrpn_client_node/fisheyed_marker/pose", 1000, &validation::update_mrkr_pose, this);
        est_mrkr = nh.subscribe("/pose", 1000, &validation::update_mrkr_pose_est, this);
        ROS_INFO_STREAM("Initializing");
}

void validation::update_cam_pose(const geometry_msgs::PoseStamped& msg)
{
    cam_x = msg.pose.position.x;
    cam_y = msg.pose.position.y;
    cam_z = msg.pose.position.z;

    mrkr_qx = msg.pose.orientation.x;
    mrkr_qy = msg.pose.orientation.y;
    mrkr_qz = msg.pose.orientation.z;
    mrkr_qw = msg.pose.orientation.w;
}

void validation::update_mrkr_pose(const geometry_msgs::PoseStamped& msg)
{
    mrkr_x = msg.pose.position.x;
    mrkr_y = msg.pose.position.y;
    mrkr_z = msg.pose.position.z;
}

void validation::update_mrkr_pose_est(const geometry_msgs::Pose& msg)
{
    est_x = msg.position.x;
    est_y = msg.position.y;
    est_z = msg.position.z;

    validate_results();
}

void validation::update_camera_pose(const geometry_msgs::PoseStamped& msg)
{
    camera_qx = msg.pose.orientation.x;
    camera_qy = msg.pose.orientation.y;
    camera_qz = msg.pose.orientation.z;
    camera_qw = msg.pose.orientation.w;

    tf::Quaternion q2(camera_qx, camera_qy, camera_qz, camera_qw);
    tf::Matrix3x3 m2(q2);
    m2.setRPY(cam_roll, cam_pitch, cam_yaw);
    ros::param::set("/omega", cam_roll);
}

void validation::validate_results()
{
    if (abs_cam && abs_mrkr)
    {
        abs_x = mrkr_x - cam_x;
        abs_y = mrkr_y - cam_y;
        abs_z = mrkr_z - cam_z;

        tf::Quaternion q(mrkr_qx, mrkr_qy, mrkr_qz, mrkr_qw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        abs_relx = abs_x*cos(yaw) + abs_y*sin(yaw);
        abs_rely = - abs_x*sin(yaw) + abs_y*cos(yaw);
        abs_relz = abs_z;

        dist = sqrt(abs_relx*abs_relx + abs_rely*abs_rely + abs_relz*abs_relz);
        est_dist = sqrt(est_x*est_x + abs_rely*abs_rely + est_z*est_z);

        error_x = abs(abs_relx - est_x);
        accuracy_x = 100 - abs(error_x/abs_relx)*100;

        error_y = abs(abs_rely - est_y);
        accuracy_y = 100 - abs(error_y/abs_rely)*100;

        error_z = abs(abs_relz - est_z);
        accuracy_z = 100 - abs(error_z/abs_relz)*100;

        error_dist = abs(dist - est_dist);
        accuracy_dist = 100 - abs(error_dist/dist)*100;

        ROS_INFO_STREAM(" Error x: "<< error_x << " Accuracy x: " << accuracy_x );
        ROS_INFO_STREAM(" Error y: "<< error_y << " Accuracy y: " << accuracy_y );
        ROS_INFO_STREAM(" Error z: "<< error_z << " Accuracy z: " << accuracy_z );

        ROS_INFO_STREAM("  ");
        ROS_INFO_STREAM(" abs rel x: "<< abs_relx << " abs rel y: " << abs_rely << " abs rel z: " << abs_relz );
        ROS_INFO_STREAM(" ets x: "<< est_x << " est y: " << est_y << " est z: " << est_z );
        ROS_INFO_STREAM(" ets dist: "<< est_dist << " dist: " << dist);
        ROS_INFO_STREAM("  ");
    }
}

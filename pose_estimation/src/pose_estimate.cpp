#include "pose_estimate.h"

pose_estimate::pose_estimate(cv::Mat Camera_matrix, cv::Mat Distcoefs, double Phi, double Omega)
{
    sqr_numbr = 5;

    camera_matrix = Camera_matrix;
    distcoefs = Distcoefs;
    pub = nh.advertise<geometry_msgs::Pose>("pose",1000);
    phi = Phi;
    omega = Omega;

    Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

}

cv::Mat pose_estimate::pose_marker(cv::Mat image, std::array<double, 3> c, std::vector<int> id, double marker_len)
{
    cv::Mat new_image, gray_img;
    image.copyTo(new_image);
    cv::cvtColor(new_image, gray_img, cv::COLOR_BGR2GRAY);

    std::vector<int> ids, ids_to_est;
    std::vector<std::vector<cv::Point2f>> corners, corners_to_est;
    std::vector<cv::Vec3d> rvecs, tvecs;

    cp_x = c.at(0);
    cp_y = c.at(1);
    cp_z = c.at(2);

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

               // ROS_INFO_STREAM("Pixel density : " << pixels_per_sqr);
            }
        }
    }

    return new_image;
}

void pose_estimate::broadcast(cv::Vec3d rvecs, cv::Vec3d tvecs)
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

    //transformation from sensor frame to camera frame

    if (omega == 0 & phi == 0)
    {
        x = p_x;
        y = p_y;
        z = p_z;

        get_orientation(cp_x, cp_y, cp_z, rvecs);
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

        x_1 = -z_y*cp_x/modz_x - z_x*z_z*cp_y/modz_y + z_x*cp_z;
        y_1 = z_x*cp_x/modz_x - z_y*z_z*cp_y/modz_y + z_y*cp_z;
        z_1 = (z_y*z_y + z_x*z_x)*cp_y/modz_y + z_z*cp_z;

        get_orientation(x_1, y_1, z_1, rvecs);
    }

    transform.setOrigin(tf::Vector3(x, y, z));
    m.getRotation(q);
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

void pose_estimate::get_orientation(double x, double y, double z, cv::Vec3d rvec)
{
    cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64FC1);
    //get the rotation matrix of the virtual camera ref frame

    double cam_roll = acos(z/sqrt(y*y + z*z));
    double cam_pitch = acos(z/sqrt(x*x + z*z));
    double cam_yaw = acos(y/sqrt(x*x + y*y));

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


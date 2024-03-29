
#include "image_converter.h"

image_converter::image_converter()
: it_(nh_)
{
    image_sub = it_.subscribe("usb_cam/image", 1 , &image_converter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);

    theta_min =  -CV_PI/6;
    theta_max =  CV_PI/6;
    delta_min = CV_PI/2 + CV_PI/8;
    delta_max = CV_PI/2 + 3*CV_PI/8;
    omega = CV_PI/4;
    phi = CV_PI/2;

    marker_len = 0.2;

    camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    distcoefs = cv::Mat::zeros(5, 1, CV_64FC1);

    id = {0};

}

image_converter::~image_converter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void image_converter::imageCb(const sensor_msgs::ImageConstPtr& msg)  //function to convert data to cv::Mat
{
    cv::Mat frame;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exceprion: %s",e.what());
        return;
    }

    img = cv_ptr->image;

    frame = panoramic.slice(img, c, theta_min, theta_max, delta_min, delta_max);

    panoramic.def_camera_matrix(camera_matrix, distcoefs, theta_min, theta_max, delta_min, delta_max);

    new_image = estimate.pose_marker(frame, c, id, marker_len);

    cv::imshow(OPENCV_WINDOW, new_image);

    int k = cv::waitKey(1);

    if (k%256 == 32)
    {
        std::string name = "/home/tue-me-minicar-laptop-02/internship/camera_calibration/camera_01/validation_/validation_9/open_cv_img" + std::to_string(img_counter) + ".png" ;
        cv::imwrite(name, frame);
        img_counter++;
    }
}


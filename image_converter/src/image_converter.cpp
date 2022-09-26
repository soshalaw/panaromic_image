
#include "image_converter.h"

image_converter::image_converter()
: it_(nh_)
{
    image_sub = it_.subscribe("usb_cam/image", 1 , &image_converter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);

    ros::param::get("delta_min", delta_min);
    ros::param::get("delta_max", delta_max);

    ros::param::get("invpol", invpol);
    ros::param::get("mapcoef", pol);
    ros::param::get("yc", yc);
    ros::param::get("xc", xc);
    ros::param::get("c", c_);
    ros::param::get("d", d);
    ros::param::get("e", e);

    ros::param::get("theta_min", theta_min);
    ros::param::get("theta_max", theta_max);

    ros::param::get("alpha_min", alpha_min);
    ros::param::get("alpha_max", alpha_max);
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

        img = cv_ptr->image;

        //frame = panorama.panaroma(img,delta_min,delta_max, xc, yc, c_, d, e, invpol);

        //frame = panorama.slice(img,c,theta_min,theta_max,alpha_min,alpha_max,xc,yc,c_,d,e,invpol);

        cv::resize(img, frame, cv::Size(img.cols/2,img.rows/2));

        cv::imshow(OPENCV_WINDOW, frame);
        int k = cv::waitKey(1);

        if (k%256 == 32)
        {
            std::string name = "/home/corelaptop02/internship/camera_calibration/camera_perspective/open_cv_img" + std::to_string(img_counter) + ".png" ;
            //std::string name = "/home/corelaptop02/Thesis/results/cam6panorama.png";
            cv::imwrite(name, img);
            img_counter++;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exceprion: %s",e.what());
        return;
    }
}

void image_converter::panorama_merged() //merging the images of the two cameras into one matrix
{
    cv::Mat new_img, res;



    //create new image matrix of size (img1_rows,2*img1_cols) assuming both images are of same size

    //new_img = cv::Mat::zeros(img1.rows, (img1.cols*2+img2.cols), img1.type());

    // copy the camera1 image to the first set of columns and camera2 image to the last set

    //img1.copyTo(new_img(cv::Rect(0,0,img1.cols,img1.rows)));
    //img2.copyTo(new_img(cv::Rect(img1.cols,0,img1.cols,img1.rows)));;

    //resize the image to fit the monitor screen for visualization

    //cv::resize(new_img, res, cv::Size(new_img.cols, new_img.rows));

    cv::imshow(OPENCV_WINDOW, res);

    cv::waitKey(1);

}


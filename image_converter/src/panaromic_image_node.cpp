#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>


#include "image_converter.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "panaromic_image_node");
    bridge brdg;
    ros::spin();
    return 0;
}


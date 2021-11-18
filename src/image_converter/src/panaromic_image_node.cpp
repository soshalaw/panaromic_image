#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include "ocam_functions.h"
#include "image_converter.h"


int main()
{
    ros::init(argc, argv, "panaromic_image_node");
    bridge brdg;
    brdg.show_image();
    ros::spin();
    return 0;
}


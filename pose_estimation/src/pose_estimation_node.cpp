#include <iostream>
#include <ros/ros.h>


#include "image_converter.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "panaromic_image_node");
    image_converter brdg;
    ros::spin();
    return 0;
}


#include <ros/ros.h>

#include "validation.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "validation_node");
    validation valid;
    ros::spin();
    return 0;
}

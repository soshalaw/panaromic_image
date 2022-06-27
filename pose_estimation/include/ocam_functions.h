
#include <array>
#include "math.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

class ocam_functions
{

private:
    cv::Mat img, ImgPointsx, ImgPointsy;  // definition of matrices for the output image and remapping

    double x, y, z, cos_alpha, x_, y_, z_;
    double planer_coords[3];
    double cyl_coords[3];
    double points2D[2];
    double cp_x, cp_y, cp_z, modx, mody;

    std::vector<double> invpol;
    std::vector<double> pol;
    double yc;
    double xc;

    double c;
    double d;
    double e;

    int H_res; // length of the output image

    double pixel_length;  //length of a pixel in mm extracted from the camera specs
    double foc_len;

    int mode;

public:

    ocam_functions();

    void world2cam(double point2D[2], double point3D[3]);

    cv::Mat slice(cv::Mat M, std::array<double,3> &c, double theta_min, double theta_max, double delta_min, double delta_max);

    cv::Mat panaroma(cv::Mat M);

    void def_camera_matrix(cv::Mat camera_matrix, cv::Mat distcoefs, double theta_min, double theta_max, double delta_min, double delta_max);
};

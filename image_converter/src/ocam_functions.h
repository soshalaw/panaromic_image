/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "math.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

class paranomic
{

private:
    cv::Mat img, ImgPointsx, ImgPointsy;  // definition of matrices for the output image and remapping

    double x, y, z, cos_alpha, x_, y_, z_;
    double planer_coords[3];
    double cyl_coords[3];
    double points2D[2];
    double cp_x, cp_y, cp_z, modx, mody;

    //calibration data camera 01
    std::vector<double> invpol = {-1.075233654325322e+02, 4.704007234612547e+02, -7.039759405818603e+02, 2.838316240089585e+02, -9.038676504203400e+02, 1.606691263137671e+03};
    std::vector<double> pol = {1.145882288545091e+03 ,0 ,-3.630427146836845e-04 ,1.047936476714481e-07 ,-1.000973064316358e-10};
    double yc = 1.307765250016426e+03;
    double xc = 1.647330746606595e+03;

    double c = 1;
    double d = 0;
    double e = 0;

    int H_res = 1024; // length of the output image

    int mode = 0;

public:

    paranomic() {
    }

    void world2cam(double point2D[2], double point3D[3])
    {
     double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
     double theta       = atan(point3D[2]/norm);
     int length_invpol  = 6;
     double t, t_i;
     double rho, x, y;
     double invnorm;
     int i;

      if (norm != 0)
      {
        invnorm = 1/norm;
        t  = theta;
        rho = invpol[0];
        t_i = 1;

        for (i = 1; i < length_invpol; i++)
        {
          rho *= t;
          rho += invpol[i];
        y = point3D[1]*invnorm*rho;
        }

        x = point3D[0]*invnorm*rho;

        point2D[0] = x*c + y*d + xc;
        point2D[1] = x*e + y   + yc;
      }
      else
      {
        point2D[0] = xc;
        point2D[1] = yc;
      }
    }

    cv::Mat slice(cv::Mat M, double c[3], double theta_min, double theta_max, double delta_min, double delta_max)
    {
        double alpha = theta_max - theta_min;

        double gamma = delta_max - delta_min;

        double theta = 0;

        int V_res = tan(gamma/2)*H_res/tan(alpha/2);

        img.create(V_res, H_res,M.type());
        ImgPointsx.create(img.size(), CV_32FC1);
        ImgPointsy.create(img.size(), CV_32FC1);

        if (mode == 1)
        {
            c[0] = cp_x = sin(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
            c[1] = cp_y = cos(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
            c[2] = cp_z = cos(delta_min + (gamma)/2);
        }else
        {
            c[0] = cp_x = sin(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
            c[1] = cp_y = cos(delta_min + (gamma)/2);
            c[2] = cp_z = cos(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
        }

        modx = sqrt(cp_y*cp_y + cp_x*cp_x);
        mody = sqrt((cp_x*cp_z)*(cp_x*cp_z) + (cp_y*cp_z)*(cp_y*cp_z) + (cp_y*cp_y + cp_x*cp_x)*(cp_y*cp_y + cp_x*cp_x));

        for(int i = 0 ; i < V_res; i++)
        {
            y_ = -tan(gamma/2) + i*2*tan(gamma/2)/V_res;

            for(int j = 0; j < H_res; j++)
            {
                x_ = -tan(alpha/2) + j*2*tan(alpha/2)/H_res;

                x = cp_y*x_/modx + cp_x*cp_z*y_/mody + cp_x;
                y = -cp_x*x_/modx + cp_y*cp_z*y_/mody + cp_y;
                z = -(cp_y*cp_y + cp_x*cp_x)*y_/mody + cp_z;

                planer_coords[0] = x/sqrt(pow(x,2) + pow(y,2) + pow(z,2));
                planer_coords[1] = y/sqrt(pow(x,2) + pow(y,2) + pow(z,2));
                planer_coords[2] = z/sqrt(pow(x,2) + pow(y,2) + pow(z,2));

                world2cam(points2D, planer_coords);

                ImgPointsx.at<float>(i,j) = points2D[0];
                ImgPointsy.at<float>(i,j) = points2D[1];
            }
        }

        cv::remap(M, img, ImgPointsx, ImgPointsy, 1);

        return img;
    }

    cv::Mat panaroma(cv::Mat M)
    {
        double theta_min = 0;
        double theta_max = CV_PI*2;

        double delta_min = CV_PI/6;
        double delta_max = CV_PI/2;
        double x, y, z;

        double h_m = cos(delta_min) - cos(delta_max);
        int V_res = h_m*H_res/(theta_max - theta_min);

        img.create(V_res, H_res,M.type());
        ImgPointsx.create(img.size(), CV_32FC1);
        ImgPointsy.create(img.size(), CV_32FC1);

        for(int i = 0; i < V_res; i++)
        {
            z = cos(delta_min + i*(delta_max - delta_min)/V_res);

            for(int j = 0; j < H_res; j++)
            {
                x = cos(theta_min + (theta_max - theta_min)*j/H_res)*sin(delta_min + (delta_max - delta_min)*i/V_res);
                y = -sin(theta_min + (theta_max - theta_min)*j/H_res)*sin(delta_min + (delta_max - delta_min)*i/V_res);

                cyl_coords[0] = x/sqrt(pow(x,2) + pow(y,2));
                cyl_coords[1] = y/sqrt(pow(x,2) + pow(y,2));
                cyl_coords[2] = z/sqrt(pow(x,2) + pow(y,2));

                world2cam(points2D, cyl_coords);

                ImgPointsx.at<double>(i,j) = points2D[0];
                ImgPointsy.at<double>(i,j) = points2D[1];
            }
        }

        cv::remap(M, img, ImgPointsx, ImgPointsy, 1);
        return img;
    }

};


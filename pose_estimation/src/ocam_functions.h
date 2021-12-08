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
public:
    paranomic() {
    }

    double invpol[6] = {-48.7332773169590,232.972008795315,-390.397549899376,214.983251794334,-308.310044095912,513.809204056670};
    double pol[5] = {382.404319709623,0,-0.00113703769137079,1.18804593219259e-06,-3.83168603418075e-09};
    double yc = 404.531114757043;
    double xc = 519.031000184018;
    double c = 1;
    double d = 0;
    double e = 0;
    double width = 768;
    double length = 1024*1.5;

    /*int get_ocam_model(struct ocam_model *myocam_model, char *filename)
    {
     double *pol        = myocam_model->pol;
     double *invpol     = myocam_model->invpol;
     double *xc         = &(myocam_model->xc);
     double *yc         = &(myocam_model->yc);
     double *c          = &(myocam_model->c);
     double *d          = &(myocam_model->d);
     double *e          = &(myocam_model->e);
     int    *width      = &(myocam_model->width);
     int    *height     = &(myocam_model->height);
     int *length_pol    = &(myocam_model->length_pol);
     int *length_invpol = &(myocam_model->length_invpol);
     FILE *f;
     char buf[CMV_MAX_BUF];
     int i;

     //Open file
     if(!(f=fopen(filename,"r")))
     {
       printf("File %s cannot be opened\n", filename);
       return -1;
     }

     //Read polynomial coefficients
     fgets(buf,CMV_MAX_BUF,f);
     fscanf(f,"\n");
     fscanf(f,"%d", length_pol);
     for (i = 0; i < *length_pol; i++)
     {
         fscanf(f," %lf",&pol[i]);
     }

     //Read inverse polynomial coefficients
     fscanf(f,"\n");
     fgets(buf,CMV_MAX_BUF,f);
     fscanf(f,"\n");
     fscanf(f,"%d", length_invpol);
     for (i = 0; i < *length_invpol; i++)
     {
         fscanf(f," %lf",&invpol[i]);
     }

     //Read center coordinates
     fscanf(f,"\n");
     fgets(buf,CMV_MAX_BUF,f);
     fscanf(f,"\n");
     fscanf(f,"%lf %lf\n", xc, yc);

     //Read affine coefficients
     fgets(buf,CMV_MAX_BUF,f);
     fscanf(f,"\n");
     fscanf(f,"%lf %lf %lf\n", c,d,e);

     //Read image size
     fgets(buf,CMV_MAX_BUF,f);
     fscanf(f,"\n");
     fscanf(f,"%d %d", height, width);

     fclose(f);
     return 0;
    }*/


    //------------------------------------------------------------------------------
    void world2cam(float point2D[2], float point3D[3])
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
        }

        x = point3D[0]*invnorm*rho;
        y = point3D[1]*invnorm*rho;

        point2D[0] = x*c + y*d + xc;
        point2D[1] = x*e + y   + yc;
      }
      else
      {
        point2D[0] = xc;
        point2D[1] = yc;
      }

    }


    cv::Mat slice(cv::Mat M)
    {
        double theta_min = -CV_PI/4;
        double theta_max = CV_PI/4;

        double delta_min = CV_PI/4;
        double delta_max = CV_PI/2;
        float x, y, z, cos_alpha;

        int H_res = length/2;
        int V_res = (delta_max - delta_min)*H_res/(theta_max - theta_min);

        cv::Mat img, ImgPointsx, ImgPointsy;
        float planer_coords[3];

        img.create(V_res, H_res,M.type());
        ImgPointsx.create(img.size(), CV_32FC1);
        ImgPointsy.create(img.size(), CV_32FC1);

        float cp_x = cos(theta_min + (theta_max - theta_min)/2)*sin(delta_min + (delta_max - delta_min)/2);
        float cp_y = -sin(theta_min + (theta_max - theta_min)/2)*sin(delta_min + (delta_max - delta_min)/2);
        float cp_z = cos(delta_min + (delta_max - delta_min)/2);

        float points2D[2];


        for(int i = 0 ; i < V_res; i++)
        {
            z = cos(delta_min + i*(delta_max - delta_min)/V_res);

            for(int j = 0; j < H_res; j++)
            {
                x = cos(theta_min + (theta_max - theta_min)*j/H_res)*sin(delta_min + (delta_max - delta_min)*i/V_res);
                y = -sin(theta_min + (theta_max - theta_min)*j/H_res)*sin(delta_min + (delta_max - delta_min)*i/V_res);

                cos_alpha = cp_x*x + cp_y*y + cp_z*z;

                planer_coords[0] = x/cos_alpha;
                planer_coords[1] = y/cos_alpha;
                planer_coords[2] = z/cos_alpha;

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
        float x, y, z;

        int H_res = length;
        double h_m = cos(delta_min) - cos(delta_max);
        int V_res = h_m*H_res/(theta_max - theta_min);

        cv::Mat img, ImgPointsx, ImgPointsy;
        float cyl_coords[3];

        img.create(V_res, H_res,M.type());
        ImgPointsx.create(img.size(), CV_32FC1);
        ImgPointsy.create(img.size(), CV_32FC1);

        float points2D[2];


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

                ImgPointsx.at<float>(i,j) = points2D[0];
                ImgPointsy.at<float>(i,j) = points2D[1];
            }

        }

        cv::remap(M, img, ImgPointsx, ImgPointsy, 1);
        return img;
    }

};
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
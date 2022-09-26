
#include "ocam_functions.h"

ocam_functions::ocam_functions()
{
    ros::param::get("H_res", H_res); // length of the output image

    ros::param::get("pixel_length", pixel_length);  //length of a pixel in mm extracted from the camera specs
    //foc_len = pol[0]*pixel_length;

    mode = 0;
}

void ocam_functions::world2cam(double point2D[2], double point3D[3], double xc, double yc, double c, double d, double e, std::vector<double> invpol)
{
     double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
     double theta       = atan(point3D[2]/norm);
     int length_invpol  = static_cast<int>(invpol.size());
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

cv::Mat ocam_functions::slice(cv::Mat M, std::array<double,3> &c, double theta_min_deg, double theta_max_deg, double alpha_min_deg, double alpha_max_deg, double xc, double yc, double c_, double d, double e, std::vector<double> invpol)
{
    double theta_min, theta_max, alpha_min, alpha_max;

    theta_max = CV_PI*theta_max_deg/180;
    theta_min = CV_PI*theta_min_deg/180;


    alpha_max = CV_PI*alpha_max_deg/180;
    alpha_min = CV_PI*alpha_min_deg/180;

    {
        double alpha = theta_max - theta_min;

        double gamma = alpha_max - alpha_min;

        double theta = 0;

        int V_res = static_cast<int>(tan(gamma/2)*H_res/tan(alpha/2));

        img.create(V_res, H_res,M.type());
        ImgPointsx.create(img.size(), CV_32FC1);
        ImgPointsy.create(img.size(), CV_32FC1);

        if (mode == 1)
        {
            c.at(0) = cp_x = sin(theta_min + (alpha)/2)*sin(alpha_min + (gamma)/2);
            c.at(1) = cp_y = cos(theta_min + (alpha)/2)*sin(alpha_min + (gamma)/2);
            c.at(2) = cp_z = cos(alpha_min + (gamma)/2);
        }else
        {
            c.at(0) = cp_x = sin(theta_min + (alpha)/2)*sin(alpha_min + (gamma)/2);
            c.at(1) = cp_y = cos(alpha_min + (gamma)/2);
            c.at(2) = cp_z = cos(theta_min + (alpha)/2)*sin(alpha_min + (gamma)/2);
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

                world2cam(points2D, planer_coords, xc, yc, c_, d, e, invpol);

                ImgPointsx.at<float>(i,j) = static_cast<float>(points2D[0]);
                ImgPointsy.at<float>(i,j) = static_cast<float>(points2D[1]);
            }
        }

        cv::remap(M, img, ImgPointsx, ImgPointsy, 1);

        return img;
    }
}

cv::Mat ocam_functions::panaroma(cv::Mat M, double delta_min_deg, double delta_max_deg, double xc, double yc, double c_, double d, double e, std::vector<double> invpol)
{
    double x, y, z, delta_min, delta_max;

    delta_max = CV_PI*delta_max_deg/180;
    delta_min = CV_PI*delta_min_deg/180;

    double h_m = tan(delta_min) + tan(delta_max);
    int V_res = int(h_m*H_res/CV_PI);

    img.create(V_res, H_res,M.type());
    ImgPointsx.create(img.size(), CV_32FC1);
    ImgPointsy.create(img.size(), CV_32FC1);

    for(int i = 0; i < V_res; i++)
    {
        y = -tan(delta_min) + h_m*i/V_res;

        for(int j = 0; j < H_res; j++)
        {
            x = cos(CV_PI*j/H_res);
            z = sin(CV_PI*j/H_res);;

            cyl_coords[0] = x;
            cyl_coords[1] = y;
            cyl_coords[2] = z;

            world2cam(points2D, cyl_coords, xc, yc, c_, d, e, invpol);

            ImgPointsx.at<float>(i,j) = static_cast<float>(points2D[0]);
            ImgPointsy.at<float>(i,j) = static_cast<float>(points2D[1]);
        }
    }

    cv::remap(M, img, ImgPointsx, ImgPointsy, 1);
    return img;
    }

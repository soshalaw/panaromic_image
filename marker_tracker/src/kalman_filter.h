#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H


#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>


class kalman_filter
{
private:
    //measured translations and rotations

    int nStates;
    int nMeasurements;
    int nInputs;
    double dt;

    cv::KalmanFilter KF;
    cv::Mat prediction;
    cv::Mat estimated;
    cv::Mat measurements = cv::Mat::zeros(6, 1, CV_64F);
    cv::Vec3d measured_eulers;

public:
    kalman_filter(int NStates, int NMeasurements, int NInputs, double Dt)
    {
        nStates = NStates;
        nMeasurements = NMeasurements;
        nInputs = NInputs;
        dt = Dt;

        KF = cv::KalmanFilter(nStates, nMeasurements, nInputs, CV_64F);

        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));       // set process noise
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1));   // set measurement noise
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));             // error covariance

        // position
        KF.transitionMatrix.at<double>(0,3) = dt;
        KF.transitionMatrix.at<double>(1,4) = dt;
        KF.transitionMatrix.at<double>(2,5) = dt;
        KF.transitionMatrix.at<double>(3,6) = dt;
        KF.transitionMatrix.at<double>(4,7) = dt;
        KF.transitionMatrix.at<double>(5,8) = dt;
        KF.transitionMatrix.at<double>(0,6) = 0.5*dt*dt;
        KF.transitionMatrix.at<double>(1,7) = 0.5*dt*dt;
        KF.transitionMatrix.at<double>(2,8) = 0.5*dt*dt;

        // orientation
        KF.transitionMatrix.at<double>(9,12) = dt;
        KF.transitionMatrix.at<double>(10,13) = dt;
        KF.transitionMatrix.at<double>(11,14) = dt;
        KF.transitionMatrix.at<double>(12,15) = dt;
        KF.transitionMatrix.at<double>(13,16) = dt;
        KF.transitionMatrix.at<double>(14,17) = dt;
        KF.transitionMatrix.at<double>(9,15) = 0.5*dt*dt;
        KF.transitionMatrix.at<double>(10,16) = 0.5*dt*dt;
        KF.transitionMatrix.at<double>(11,17) = 0.5*dt*dt;

        KF.measurementMatrix.at<double>(0,0) = 1;  // x
        KF.measurementMatrix.at<double>(1,1) = 1;  // y
        KF.measurementMatrix.at<double>(2,2) = 1;  // z
        KF.measurementMatrix.at<double>(3,9) = 1;  // roll
        KF.measurementMatrix.at<double>(4,10) = 1; // pitch
        KF.measurementMatrix.at<double>(5,11) = 1; // yaw
    }

    void fillMeasurements(cv::Vec3d translation_measured, cv::Vec3d rotation_measured)
    {
        // Convert rotation matrix to euler angles

        measured_eulers = rot2euler(rotation_measured);

        // Set measurement to predict
        measurements.at<double>(0) = translation_measured[0]; // x
        measurements.at<double>(1) = translation_measured[1]; // y
        measurements.at<double>(2) = translation_measured[2]; // z
        measurements.at<double>(3) = measured_eulers[0];      // roll
        measurements.at<double>(4) = measured_eulers[1];      // pitch
        measurements.at<double>(5) = measured_eulers[2];      // yaw
    }

    void updateKalmanFilter(cv::Vec3d &translation_estimated, cv::Mat &rotation_estimated)
    {
        // First predict, to update the internal statePre variable
        prediction = KF.predict();

        // The "correct" phase that is going to use the predicted value and our measurement
        estimated = KF.correct(measurements);

        // Estimated translation
        translation_estimated[0] = estimated.at<double>(0);
        translation_estimated[1] = estimated.at<double>(1);
        translation_estimated[2] = estimated.at<double>(2);

        // Estimated rotation euler angles
        cv::Mat eulers_estimated(3, 1, CV_64F);
        eulers_estimated.at<double>(0) = estimated.at<double>(9);
        eulers_estimated.at<double>(1) = estimated.at<double>(10);
        eulers_estimated.at<double>(2) = estimated.at<double>(11);

        // Convert estimated quaternion to rotation matrix
        rotation_estimated = euler2rot(eulers_estimated);
    }

    cv::Vec3d rot2euler(cv::Vec3d rvecs)
    {
        double sy, x, y, z;
        bool singular;
        cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);

        cv::Rodrigues(rvecs, R);

        sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

        singular = sy < 1e-6; // If

        if(singular)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }

        return cv::Vec3d(x, y, z);
    }

    cv::Mat euler2rot(cv::Vec3d e)
    {
        cv::Mat R = (cv::Mat_<double>(3,3) <<
                     cos(e[1])*cos(e[2]), -cos(e[1])*sin(e[2]), sin(e[1]),
                sin(e[0])*sin(e[1])*cos(e[2])+cos(e[0])*sin(e[2]), cos(e[0])*cos(e[2])-sin(e[0])*sin(e[1])*sin(e[2]), -sin(e[0])*cos(e[1]),
                sin(e[0])*sin(e[2])-cos(e[0])*sin(e[2])*cos(e[3]), cos(e[0])*sin(e[1])*sin(e[2])+sin(e[0])*cos(e[2]), cos(e[0])*cos(e[1]));

        return R;
    }

};

#endif // KALMAN_FILTER_H

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.h
 * Author: akp
 *
 * Created on 4. April 2018, 02:40
 */

#pragma once

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "util/FrameShell.h"
#include <chrono>
#include <boost/date_time.hpp>

#include <eigen3/Eigen/Eigen>

namespace dso
{
 
    /*
     
    This file implements a (linear) Kalman Filter, which estimates the
    optimal camera trajectory
     
     */
    
    class KalmanFilter
    {
    public:
        KalmanFilter();
        

        void init(float initx = 0.0, float inity = 0.0, float initz = 0.0,
                  float initvx = 0.0, float initvy = 0.0, float initvz = 0.0,
                  float sigmax = 10.0, float sigmay = 10.0, float sigmaz = 10.0,
                  float sigmavx = 1.0, float sigmavy = 1.0, float sigmavz = 1.0,
                  float qx = 0.0003, float qy = 0.0003, float qz = 0.0003,
                  float qvx = 0.0001, float qvy = 0.0001, float qvz = 0.0001, 
                  float rx = 10.0f, float ry = 10.0f, float rz = 10.0f,
                  float rvx = 1.0f, float rvy = 1.0f, float rvz = 1.0f);

        bool initialized;
        bool isInitialized();
        
        float initx, inity, initz;
        float initvx, initvy, initvz;
        float sigmax, sigmay, sigmaz;
        float sigmavx, sigmavy, sigmavz;
        float qx, qy, qz;
        float qvx, qvy, qvz;
        float rx, ry, rz;
        float rvx, rvy, rvz;
        
        
        Eigen::Matrix<float,6,1> state; // State vector: x, y, z, velocityX, velocityY, velocityZ
        Eigen::Matrix<float,6,6> sigma; // "P"-Matrix (state covariance / uncertainty of state)

        Eigen::Matrix<float,6,6> Q; // Possible noise in the predicted new state (process noise)
        Eigen::Matrix<float,3,3> R_GPS; // Possible noise in the measured new GPS state (observation noise)
        Eigen::Matrix<float,3,3> R_DSO; // Possible noise in the measured new DSO state (observation noise)
        
        unsigned long timestampLast;
        Eigen::Vector3f dsoTranslationPrevious;
        Eigen::Quaternionf dsoOrientation;

        // Next timestep x_{t+1} = g(x_t,u) 
        // and update uncertainty
        void predictionStep(const Eigen::Matrix<float,6,1>& currentState); 
        void predict(float timestep);

        // compare expected and measured values, update state and uncertainty
        void correctionStep(const Eigen::Vector3f& measurement);
        void updateGPS(const Eigen::Vector3f& measurement); // GPS x-y-z position
        void updateDSO(const Eigen::Vector3f& measurement, const Eigen::Quaternionf orientation, float timestep); // Velocities
        
        /*
         Helper Functions
         */
        unsigned long getCurrentTimestamp();
        unsigned long getLastTimestamp();
        void setLastTimestampToCurrent();
        double getDeltaTimeInSeconds();
        
        
        const double  a = 6378137.0;              //WGS-84 semi-major axis
        //const double  a = 0.0006378137;              //TODO: WRONG (for visualization only)
        const double e2 = 6.6943799901377997e-3;  //WGS-84 first eccentricity squared
        const double a1 = 4.2697672707157535e+4;  //a1 = a*e2
        const double a2 = 1.8230912546075455e+9;  //a2 = a1*a1
        const double a3 = 1.4291722289812413e+2;  //a3 = a1*e2/2
        const double a4 = 4.5577281365188637e+9;  //a4 = 2.5*a2
        const double a5 = 4.2840589930055659e+4;  //a5 = a1+a3
        const double a6 = 9.9330562000986220e-1;  //a6 = 1-e2
        
        void ecef_to_geo( double &ecef_x, double &ecef_y, double &ecef_z, double &geo_lat, double &geo_lon, double &geo_alt );
        void geo_to_ecef(double &geo_lat, double &geo_lon, double &geo_alt, double &ecef_x, double &ecef_y, double &ecef_z);
        
        SE3 getStateAsSE3();

    protected:

    
    };

}


#endif /* KALMANFILTER_H */


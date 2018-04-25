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
        inline KalmanFilter()
        {
            // Constructor is not used but init();
        }

        void init(float initx = 0.0, float inity = 0.0, float initz = 0.0,
                  float initvx = 0.0, float initvy = 0.0, float initvz = 0.0,
                  float sigmax = 0.0, float sigmay = 0.0, float sigmaz = 0.0,
                  float sigmavx = 0.0, float sigmavy = 0.0, float sigmavz = 0.0,
                  float qx = 0.0003, float qy = 0.0003, float qz = 0.0003,
                  float qvx = 0.0001, float qvy = 0.0001, float qvz = 0.0001, 
                  float rx = 0.3, float ry = 0.3, float rz = 0.3,
                  float rvx = 0.1, float rvy = 0.1, float rvz = 0.1);

    
        Eigen::Matrix<float,6,1> state; // State vector: x, y, z, velocityX, velocityY, velocityZ
        Eigen::Matrix<float,6,6> sigma; // "P"-Matrix (state covariance / uncertainty of state)

        Eigen::Matrix<float,6,6> Q; // Possible noise in the predicted new state (process noise)
        Eigen::Matrix<float,2,2> R; // Possible noise in the measured new state (observation noise)

        // Next timestep x_{t+1} = g(x_t,u) 
        // and update uncertainty
        void predictionStep(const Eigen::Matrix<float,6,1>& currentState); 
        void predict(float timestep);

        // compare expected and measured values, update state and uncertainty
        void correctionStep(const Eigen::Vector3f& measurement);
        void update(const Eigen::Vector2f& measurement);
        
        SE3 getStateAsSE3();

    protected:

    
    };

}


#endif /* KALMANFILTER_H */


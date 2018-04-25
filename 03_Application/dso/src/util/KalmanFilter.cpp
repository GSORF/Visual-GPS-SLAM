/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.cpp
 * Author: akp
 *
 * Created on 4. April 2018, 02:49
 */

#include "KalmanFilter.h"
#include "NumType.h"


// State:
// x: World position in X
// y: World position in Y
// z: World position in Z
// vx: Velocity in X
// vy: Velocity in Y
// vz: Velocity in Z

namespace dso
{
    
    using namespace Eigen;

    
    void KalmanFilter::init(float initx, float inity, float initz,
                            float initvx, float initvy, float initvz,
                            float sigmax, float sigmay, float sigmaz,
                            float sigmavx, float sigmavy, float sigmavz,
                            float qx, float qy, float qz,
                            float qvx, float qvy, float qvz, 
                            float rx, float ry, float rz,
                            float rvx, float rvy, float rvz )
    {
        state = Eigen::Matrix<float,6,1>::Zero(); state(0) = initx; state(1) = inity; state(2) = initz; state(3) = initvx; state(4) = initvy; state(5) = initvz;
        sigma = Eigen::Matrix<float,6,6>::Zero(); sigma(0,0) = sigmax ; sigma(1,1) = sigmay; sigma(2,2) = sigmaz; sigma(3,3) = sigmavx ; sigma(4,4) = sigmavy; sigma(5,5) = sigmavz;
        
        Q = Eigen::Matrix<float,6,6>::Zero();     Q(0,0) = qx; Q(1,1) = qy; Q(2,2) = qz; Q(3,3) = qvx; Q(4,4) = qvy; Q(5,5) = qvz;
        R = Eigen::Matrix<float,2,2>::Zero();     R(0,0) = rx; R(1,1) = ry; // R(2,2) = rz; R(3,3) = rvx; R(4,4) = rvy; R(5,5) = rvz;

    }
    void KalmanFilter::predictionStep(const Eigen::Matrix<float,6,1>& currentState)
    {
        /*
         For example in an Extended Kalman Filter:
          
         ( x )   ( x + cos(yaw)*current_x - sin(yaw)*current_y )
         ( y ) = ( y + sin(yaw)*current_x + cos(yaw)*current_y  )
         (yaw) = (yaw + current_yaw)
          
         //To normalize the angle, atan2 can be used. like this ( atan2(sin(yaw), cos(yaw)) ) 
         
         //Then build matrix G (dg / dx) with above new data:
         
             ( 1     0   -sin(yaw)*current_x - cos(yaw)*current_y )
         G = ( 0     1    cos(yaw)*current_x - sin(yaw)*current_y )
             ( 0     0                       1                    )
          
         // Then update the sigma matrix:
         
         sigma = G * sigma * G.transpose() + Q
         
         */

    }

    void KalmanFilter::predict(float timestep)
    {
        /*
         
         This is the implementation based on a linear Kalman Filter
         
         // We need the State Transition Matrix (Phi) which determines what happens with the state vector
         // at each timestep. In this case of using the constant velocity model it looks like this:
         
         [x']  = [ 1   0   0  Timestep      0       0      ]  [x]
         [y']  = [ 0   1   0      0     Timestep    0      ]  [y]
         [z']  = [ 0   0   1      0         0     Timestep ]  [z]
         [vx'] = [ 0   0   0      1         0       0      ]  [vx]
         [vy'] = [ 0   0   0      0         1       0      ]  [vy]
         [vz'] = [ 0   0   0      0         0       1      ]  [vz]
         
         Furthermore we add a second term for control commands to a robot.
         This would be the control matrix B times the control vector "u"
         However, we don't have that in our case, do we?!
         
          
         Then we need to update the state covariance (the uncertainty in the state)
          
         This is done by calculating P = Phi * P * Phi.transpose() + Q
         
         */
        
        // State transition matrix:
        Eigen::Matrix<float,6,6> Phi = Eigen::Matrix<float,6,6>::Zero();
        Phi(0,0) = Phi(1,1) = Phi(2,2) = Phi(3,3) = Phi(4,4) = Phi(5,5) = 1;
        Phi(0,3) = Phi(1,4) = Phi(2,5) = timestep;
        
        state = Phi * state;
        
        // State covariance matrix
        
        sigma = Phi * sigma * Phi.transpose() + Q;
        
    }



    void KalmanFilter::correctionStep(const Eigen::Vector3f& measurement)
    {
        
        /*
         
         This is an example from an extended kalman filter.
          
         We basically get a new measurement and want to use this to "push" our
         current state vector towards the measurement.
         
         In order to accomplish this, we need to know a mesurement matrix H,
         which tells us what components of the state vector the incoming
         measurement contains. 
         
         As we are linearizing in the extended kalman filter, we need to determine
         dh / dx
          
         So we can fill the matrix H like this:
         Matrix3f H;
         H << cos(deltaYaw), sin(deltaYaw), -sin(deltaYaw)*deltaX + cos(deltaYaw)*deltaY,
             -sin(deltaYaw), cos(deltaYaw), -cos(deltaYaw)*deltaX - sin(deltaYaw)*deltaY,
                   0       ,       0      ,                      1;
         
         Then we can calculate the Kalman Gain by:
          
         K = sigma * H.transpose() * ( (H*sigma*H.transpose() + R) ).inverse();
         
         Finally, we can simply update the pose estimate:
         
         state = state + K*errorInTheEstimate;
         sigma = (I - K*H)*sigma;
         
        */
        
    }
    
    void KalmanFilter::update(const Eigen::Vector2f& measurement)
    {
        /*
        
        This implementation is based on a linear Kalman Filter which assumes a constant velocity motion model
        
         
         
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(H, np.dot(self.P, H.T)) + R))
        y = z - np.dot(H, self.x)
        
        self.x = self.x + np.dot(K, y)
        K0 = np.eye(self.L) - np.dot(K, H)
        self.P = np.dot(K0, np.dot(self.P, K0.T))
        return self.x, self.P 
         
         
         
         */
        Eigen::Matrix<float,2,6> H_GPS;
        H_GPS <<  1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0;
        
        
        Eigen::Matrix<float, 6,2> K1 = sigma * H_GPS.transpose();
        Eigen::Matrix<float, 2,2> K2 = (H_GPS * (sigma * H_GPS.transpose()) + R).inverse();
        
        //Eigen::Matrix<float,6,2> K = ( sigma * H_GPS.transpose() ) * (H_GPS * (sigma * H_GPS.transpose()) + R).inverse();
        Eigen::Matrix<float,6,2> K = K1 * K2;
        Eigen::Vector2f y = measurement - H_GPS * state;
        
        // Apply the calculated correction to the current state:
        state = (state + K*y);
        Eigen::Matrix<float,6,6> K0 = Eigen::Matrix<float,6,6>::Identity() - K*H_GPS;
        sigma = K0 * sigma * K0.transpose();

        
        
    }

    SE3 KalmanFilter::getStateAsSE3()
    {
        SE3 outputPose;
        
        outputPose.translation() = Eigen::Vector3d(state[0], state[1], state[2]);
        
        return outputPose;
    }


}
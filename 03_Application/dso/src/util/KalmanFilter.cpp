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

#include <complex>
#include <valarray>

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
    
    KalmanFilter::KalmanFilter()
    {
        this->initialized = false;
        this->timestampLast = getCurrentTimestamp();
    }

    bool KalmanFilter::isInitialized()
    {
        return this->initialized;
    }
    
    void KalmanFilter::init(float initx, float inity, float initz,
                            float initvx, float initvy, float initvz,
                            float sigmax, float sigmay, float sigmaz,
                            float sigmavx, float sigmavy, float sigmavz,
                            float qx, float qy, float qz,
                            float qvx, float qvy, float qvz, 
                            float rx, float ry, float rz,
                            float rvx, float rvy, float rvz )
    {
        this->initx = initx;
        this->inity = inity;
        this->initz = initz;
        this->initvx = initvx;
        this->initvy = initvy;
        this->initvz = initvz;
        this->sigmax = sigmax;
        this->sigmay = sigmay;
        this->sigmaz = sigmaz;
        this->sigmavx = sigmavx;
        this->sigmavy = sigmavy;
        this->sigmavz = sigmavz;
        this->qx = qx;
        this->qy = qy;
        this->qz = qz;
        this->qvx = qvx;
        this->qvy = qvy;
        this->qvz = qvz;
        this->rx = rx;
        this->ry = ry;
        this->rz = rz;
        this->rvx = rvx;
        this->rvy = rvy;
        this->rvz = rvz;
        
        
        
        state = Eigen::Matrix<float,6,1>::Zero(); state(0) = initx; state(1) = inity; state(2) = initz; state(3) = initvx; state(4) = initvy; state(5) = initvz;
        sigma = Eigen::Matrix<float,6,6>::Zero(); sigma(0,0) = sigmax ; sigma(1,1) = sigmay; sigma(2,2) = sigmaz; sigma(3,3) = sigmavx ; sigma(4,4) = sigmavy; sigma(5,5) = sigmavz;
        
        Q = Eigen::Matrix<float,6,6>::Zero();     Q(0,0) = qx; Q(1,1) = qy; Q(2,2) = qz; Q(3,3) = qvx; Q(4,4) = qvy; Q(5,5) = qvz;
        R_GPS = Eigen::Matrix<float,3,3>::Zero();     R_GPS(0,0) = rx; R_GPS(1,1) = ry; R_GPS(2,2) = rz;
        R_DSO = Eigen::Matrix<float,3,3>::Zero();     R_DSO(0,0) = rvx; R_DSO(1,1) = rvy; R_DSO(2,2) = rvz;

        
        dsoTranslationPrevious = Eigen::Vector3f::Zero();
        dsoOrientation = Eigen::Quaternionf::Identity();
        
        this->initialized = true;
    }
    void KalmanFilter::predictionStep(const Eigen::Matrix<float,6,1>& currentState)
    {
        if(!this->initialized) return;
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
        if(!this->initialized) return;
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
        
        // Process noise matrix (recalulate based on timestep value):
        float T = timestep;
        float T2 = T*T;
        float T3 = T2*timestep;
        Eigen::Matrix<float,6,6> Q_currentTimestep = Q; // Also called Q_k
        Q_currentTimestep.Zero();
        
        /*
         From Blender-Python implementation:
         
        Qk = np.array([[sigma_w_v*T3/3 + sigma_w_x*T, sigma_w_v*T2/2,   0,                            0,                0,                            0                 ],
                       [sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0,                0,                            0                 ],
                       [0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2,   0,                            0                 ],
                       [0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0                 ],
                       [0,                            0,                0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2    ],
                       [0,                            0,                0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T       ]], dtype=float) 
        */
        
        // Process noise for positions:
        Q_currentTimestep(0,0) = this->sigmavx*T3/3 + this->sigmax*T;
        Q_currentTimestep(1,1) = this->sigmavy*T3/3 + this->sigmay*T;
        Q_currentTimestep(2,2) = this->sigmavz*T3/3 + this->sigmaz*T;
        
        // Process noise for velocities:
        Q_currentTimestep(3,3) = this->sigmavx*T;
        Q_currentTimestep(4,4) = this->sigmavy*T;
        Q_currentTimestep(5,5) = this->sigmavz*T;
        
        // Control matrix
        Eigen::Matrix<float,6,6> B = Eigen::Matrix<float,6,6>::Zero();
        // Control vector
        Eigen::Matrix<float,6,1> control = Eigen::Matrix<float,6,1>::Zero();
        
        // Predicted (a-priori) state estimate (vector)
        state = Phi * state + B * control;
        
        // Predicted (a priori) estimate state covariance (matrix)
        sigma = Phi * sigma * Phi.transpose() + Q_currentTimestep;
        
    }



    void KalmanFilter::correctionStep(const Eigen::Vector3f& measurement)
    {
        if(!this->initialized) return;
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
    
    void KalmanFilter::updateGPS(const Eigen::Vector3f& measurement)
    {
        if(!this->initialized) return;
        /*
        
        This implementation is based on a linear Kalman Filter which assumes a constant velocity motion model
        
         
         
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(H, np.dot(self.P, H.T)) + R))
        y = z - np.dot(H, self.x)
        
        self.x = self.x + np.dot(K, y)
        K0 = np.eye(self.L) - np.dot(K, H)
        self.P = np.dot(K0, np.dot(self.P, K0.T))
        return self.x, self.P 
         
         
         
         */
        Eigen::Matrix<float,3,6> H_GPS;
        H_GPS <<  1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0;
        
        
        Eigen::Matrix<float, 6,3> K1 = sigma * H_GPS.transpose();
        Eigen::Matrix<float, 3,3> K2 = (H_GPS * (sigma * H_GPS.transpose()) + R_GPS).inverse();
        
        //Eigen::Matrix<float,6,2> K = ( sigma * H_GPS.transpose() ) * (H_GPS * (sigma * H_GPS.transpose()) + R).inverse();
        Eigen::Matrix<float,6,3> K = K1 * K2; // Kalman Gain
        Eigen::Vector3f y = measurement - H_GPS * state; // Innovation
        
        // Apply the calculated correction to the current state:
        state = (state + K*y);
        Eigen::Matrix<float,6,6> K0 = Eigen::Matrix<float,6,6>::Identity() - K*H_GPS;
        sigma = K0 * sigma * K0.transpose();

        
        
    }
    
    void KalmanFilter::updateDSO(const Eigen::Vector3f& translation, const Eigen::Quaternionf orientation, float timestep)
    {
        if(!this->initialized) return;
        
        // Calculate velocity vector
        Eigen::Vector3f velocity = (dsoTranslationPrevious - translation) / timestep;
        
        dsoTranslationPrevious = translation;
        dsoOrientation = orientation;
        
        Eigen::Matrix<float,3,6> H_DSO;
        H_DSO <<  0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 1;
        
        // TODO: Add Orientation into state vector
        
        Eigen::Matrix<float, 6,3> K1 = sigma * H_DSO.transpose();
        Eigen::Matrix<float, 3,3> K2 = (H_DSO * (sigma * H_DSO.transpose()) + R_DSO).inverse();
        
        //Eigen::Matrix<float,6,3> K = ( sigma * H_GPS.transpose() ) * (H_GPS * (sigma * H_GPS.transpose()) + R).inverse();
        Eigen::Matrix<float,6,3> K = K1 * K2; // Kalman Gain
        Eigen::Vector3f y = velocity - H_DSO * state; // Innovation
        
        // Apply the calculated correction to the current state:
        state = (state + K*y);
        Eigen::Matrix<float,6,6> K0 = Eigen::Matrix<float,6,6>::Identity() - K*H_DSO;
        sigma = K0 * sigma * K0.transpose();
        
        
    }
    
    unsigned long KalmanFilter::getCurrentTimestamp()
    {
        return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    unsigned long KalmanFilter::getLastTimestamp()
    {
        return this->timestampLast;
    }

    void KalmanFilter::setLastTimestampToCurrent()
    {
        this->timestampLast = getCurrentTimestamp();
    }

    double KalmanFilter::getDeltaTimeInSeconds()
    {
        return fabs( (double)(this->getCurrentTimestamp() - this->getLastTimestamp()) ) / 1000.0;
    }
    

    /*
     CONVERT to and from Geodetic (lat, lon, height) to ECEF coordinates (x, y, z)
     Ported to C++ from: http://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
     
     */
    
    //Convert Earth-Centered-Earth-Fixed (ECEF) to lat, Lon, Altitude
    //Input is a three element array containing x, y, z in meters
    //Returned array contains lat and lon in radians, and altitude in meters
    void KalmanFilter::ecef_to_geo( double &ecef_x, double &ecef_y, double &ecef_z, double &geo_lat, double &geo_lon, double &geo_alt )
    {
        double zp, w2, w, r2, r, s2, c2, u, v, s, ss, c, g, rg, rf, f, m, p;
        
        zp = fabs( ecef_z );
        w2 = ecef_x*ecef_x + ecef_y*ecef_y;
        w = sqrt(w2);
        r2 = w2 + ecef_z*ecef_z;
        r = sqrt( r2 );
        geo_lon = atan2( ecef_y, ecef_x );       //Lon (final)
        s2 = ecef_z*ecef_z/r2;
        c2 = w2/r2;
        u = a2/r;
        v = a3 - a4/r;
        if( c2 > 0.3 ){
            s = ( zp/r )*( 1.0 + c2*( a1 + u + s2*v )/r );
            geo_lat = asin( s );      //Lat
            ss = s*s;
            c = sqrt( 1.0 - ss );
        }
        else{
            c = ( w/r )*( 1.0 - s2*( a5 - u - c2*v )/r );
            geo_lat = acos( c );      //Lat
            ss = 1.0 - c*c;
            s = sqrt( ss );
        }
        g = 1.0 - e2*ss;
        rg = a/sqrt( g );
        rf = a6*rg;
        u = w - rg*c;
        v = zp - rf*s;
        f = c*u + s*v;
        m = c*v - s*u;
        p = m/( rf/g + f );
        geo_lat = geo_lat + p;      //Lat
        geo_alt = f + m*p/2.0;     //Altitude
        if( ecef_z < 0.0 ){
            geo_lat *= -1.0;     //Lat
        }
        
    }
    
    //Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
    //Input is a three element array containing lat, lon (rads) and alt (m)
    //Returned array contains x, y, z in meters
    void KalmanFilter::geo_to_ecef(double &geo_lat, double &geo_lon, double &geo_alt, double &ecef_x, double &ecef_y, double &ecef_z)
    {
        double n = a/sqrt( 1 - e2* sin( geo_lat )*sin( geo_lat ) );
        ecef_x = ( n + geo_alt )*cos( geo_lat )*cos( geo_lon );    //ECEF x
        ecef_y = ( n + geo_alt )*cos( geo_lat )*sin( geo_lon );    //ECEF y
        ecef_z = ( n*(1 - e2 ) + geo_alt )*sin( geo_lat );          //ECEF z
        
    }
    
    
    
    
    
    SE3 KalmanFilter::getStateAsSE3()
    {
        SE3 outputPose;
        
        outputPose.translation() = Eigen::Vector3d(state[0], state[1], state[2]);
        outputPose.so3().setQuaternion(this->dsoOrientation.cast<double>());
        
        return outputPose;
    }


}
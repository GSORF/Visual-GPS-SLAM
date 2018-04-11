/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

// For outputting as .csv file
#include <iostream>
#include <fstream>

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{

class SampleOutputWrapper : public Output3DWrapper
{
	
public:
	std::ofstream posesCSV;
        std::ofstream hessiansCSV;
        std::ofstream pointCloudOBJ;
        inline SampleOutputWrapper()
        {
		
		posesCSV.open("/home/akp/cameraPoses.csv");
                hessiansCSV.open("/home/akp/cameraHessians.csv");
                pointCloudOBJ.open("/home/akp/cameraPointcloud.obj");
                pointCloudOBJ << "o DSO_Pointcloud\n";
                
            printf("OUT: Created SampleOutputWrapper\n");
        }

        virtual ~SampleOutputWrapper()
        {
		posesCSV.close();
                hessiansCSV.close();
                pointCloudOBJ.close();
            printf("OUT: Destroyed SampleOutputWrapper\n");
		
        }

        virtual void publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity)
        {
            /*
            printf("OUT: got graph with %d edges\n", (int)connectivity.size());

            int maxWrite = 5;

            for(const std::pair<uint64_t,Eigen::Vector2i> &p : connectivity)
            {
                int idHost = p.first>>32;
                int idTarget = p.first & ((uint64_t)0xFFFFFFFF);
                printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n", idHost, idTarget, p.second[0], p.second[1]);
                maxWrite--;
                if(maxWrite==0) break;
            }
            */
        }



        virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib)
        {
            // Used for acumulating model:
            
            for(FrameHessian* f : frames)
            {
                /*
                printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
                       f->frameID,
                       final ? "final" : "non-final",
                       f->shell->incoming_id,
                       f->shell->timestamp,
                       (int)f->pointHessians.size(), (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
                std::cout << f->shell->camToWorld.matrix3x4() << "\n";
                */
                double timestep_ms = (1.0 / 25.0) * 1000.0;
                hessiansCSV << f->shell->id * timestep_ms << ",";
                
                //int maxWrite = 5;
                for(PointHessian* p : f->pointHessians)
                {
                    /*
                    printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
                           p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals );
                    */
                    
                    //skip for final!=false (according to Output3DWrapper.h)
                    if(final==false)
                    {
                        // Create 3D Vector of point in Image Space
                        Eigen::Vector4d imagePoint(p->u,p->v,p->idepth_scaled, 1.0);
                        // Transform to camera space (use inverse of calibration matrix):
                        // TODO: Don't know how to do that... ...yet.
                        /////HCalib->fxl();
                        // Transform to world space:
                        Eigen::Matrix4d cam2World( f->shell->camToWorld.matrix() );
                        Eigen::Vector4d worldPoint = cam2World * imagePoint;

                        pointCloudOBJ << "v " << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << "\n";
                        pointCloudOBJ.flush();
                    }


                    hessiansCSV << p->idepth_hessian << ", ";

                    //Flush because Destructor is never called...
                    hessiansCSV.flush();
            
                    
                    //maxWrite--;
                    //if(maxWrite==0) break;
                }
            }
            
            
            
            

            
            
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib)
        {
            
            // First show timestamp (based on 25 fps) in milliseconds:
            posesCSV << frame->id * (1.0 / 25.0) * 1000.0 << ",";
            
            // TODO Adam: change back to camToWorld (not predicted), because this is only for testing if mapping works
            
            // Translation:
            posesCSV << frame->camToWorld_predicted.inverse().translation().row(0) << ","
                    << frame->camToWorld_predicted.inverse().translation().row(1) << ","
                    << frame->camToWorld_predicted.inverse().translation().row(2) << ",";
            // Quaternion:
            posesCSV << frame->camToWorld_predicted.inverse().unit_quaternion().w() << ","
                    << frame->camToWorld_predicted.inverse().unit_quaternion().x() << ","
                    << frame->camToWorld_predicted.inverse().unit_quaternion().y() << ","
                    << frame->camToWorld_predicted.inverse().unit_quaternion().z()<< "\n";
            posesCSV.flush(); //Flush because Destructor is never called...


            
            /*
            printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
                   frame->incoming_id,
                   frame->timestamp,
                   frame->id);
            
            // Matrix3x4 from Sophus library is of type SE3
            // which - in turn - internally is a Matrix<Scalar,3,4>
            // Conversion to Translation and Quaternion is below.
            // First show timestamp (based on 25 fps) in milliseconds:
            double timestep_ms = (1.0 / 25.0) * 1000.0;
            
            Eigen::Affine3d dsoPose(frame->camToWorld.matrix3x4());
            // Matrix converting from DSO space to Blender space (will be used later for visualization):
            // x = -x
            // y =  z
            // z = -y
            Eigen::Matrix3d linearD2B(3,3);
            linearD2B << -1,  0,  0,
                          0,  0,  1,
                          0, -1,  0;
            Eigen::Affine3d dso2Blender;
            dso2Blender.setIdentity();
            dso2Blender.linear() = linearD2B;

            // Transform the pose back to Blender:
            Eigen::Affine3d blenderCam2World = dso2Blender * dsoPose;
            Eigen::Affine3d blenderWorld2Cam = blenderCam2World.inverse(); //Inverse = World to Camera

            Eigen::Quaterniond q(blenderWorld2Cam.linear());
            Eigen::Vector3d t(blenderWorld2Cam.translation());
            
            
            posesCSV << frame->id * timestep_ms << ",";
            // Translation:
            posesCSV << t.x() << ","
                    << t.y() << ","
                    << t.z() << ",";
            // Quaternion:
            posesCSV << q.w() << ","
                    << q.x() << ","
                    << q.y() << ","
                    << q.z() << "\n";

            //Flush because Destructor is never called...
            posesCSV.flush();
            
            */
            
            
            
            
        }


        virtual void pushLiveFrame(FrameHessian* image)
        {
            /*
            // can be used to get the raw image / intensity pyramid.
            printf("OUT: pushLiveFrame %d (time %f, internal ID %d). CameraToWorld:\n",
                   image->shell->incoming_id,
                   image->shell->timestamp,
                   image->shell->id);
            
            FrameShell* shell = image->shell;
            
            std::cout << shell->camToWorld.matrix3x4() << "\n";
            std::cout << "camToTrackingRef: " << shell->camToTrackingRef.matrix3x4() << "\n";
            */
            
            
            
        }

        virtual void pushDepthImage(MinimalImageB3* image)
        {
            // can be used to get the raw image with depth overlay.
        }
        virtual bool needPushDepthImage()
        {
            return false;
        }

        virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF )
        {
            /*
            printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). CameraToWorld:\n",
                   KF->frameID,
                   KF->shell->incoming_id,
                   KF->shell->timestamp,
                   KF->shell->id);
            std::cout << KF->shell->camToWorld.matrix3x4() << "\n";
            
            int maxWrite = 5;
            for(int y=0;y<image->h;y++)
            {
                for(int x=0;x<image->w;x++)
                {
                    if(image->at(x,y) <= 0) continue;

                    printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x,y,image->at(x,y));
                    
                    maxWrite--;
                    if(maxWrite==0) break;
                }
                if(maxWrite==0) break;
            }
            */
            
            
        }


};



}



}

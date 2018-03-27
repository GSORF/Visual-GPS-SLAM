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
	std::ofstream csvFile;
        inline SampleOutputWrapper()
        {
		
		csvFile.open("/home/akp/cameraPoses.csv");
                
            printf("OUT: Created SampleOutputWrapper with CSV Writer after 3am!\n");
        }

        virtual ~SampleOutputWrapper()
        {
		csvFile.close();
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
            /*
            for(FrameHessian* f : frames)
            {
                printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
                       f->frameID,
                       final ? "final" : "non-final",
                       f->shell->incoming_id,
                       f->shell->timestamp,
                       (int)f->pointHessians.size(), (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
                std::cout << f->shell->camToWorld.matrix3x4() << "\n";


                int maxWrite = 5;
                for(PointHessian* p : f->pointHessians)
                {
                    printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
                           p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals );
                    maxWrite--;
                    if(maxWrite==0) break;
                }
            }
            
            */
        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib)
        {
            printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
                   frame->incoming_id,
                   frame->timestamp,
                   frame->id);
            
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

            Eigen::Affine3d dsoPose(frame->camToWorld.matrix3x4());
            
            Eigen::Affine3d blenderCam2World = dso2Blender * dsoPose;
            Eigen::Affine3d blenderWorld2Cam = blenderCam2World.inverse(); //Inverse = World to Camera

            Eigen::Quaterniond q(blenderWorld2Cam.linear());
            Eigen::Vector3d t(blenderWorld2Cam.translation());
            
            std::cout << frame->camToWorld.matrix3x4() << "\n";
            std::cout << "camToTrackingRef: " << frame->camToTrackingRef.matrix3x4() << "\n";
            // Matrix3x4 from Sophus library is of type SE3
            // which - in turn - internally is a Matrix<Scalar,3,4>
            // Conversion to Translation and Quaternion is below.
            // First show timestamp (based on 25 fps) in milliseconds:
            csvFile << frame->id * (1.0 / 25.0) * 1000.0 << ",";
            // Translation:
            csvFile << t.x() << ","
                    << t.y() << ","
                    << t.z() << ",";
            // Quaternion:
            csvFile << q.w() << ","
                    << q.x() << ","
                    << q.y() << ","
                    << q.z() << "\n";
            csvFile.flush(); //Flush because Destructor is never called...
        }


        virtual void pushLiveFrame(FrameHessian* image)
        {
            // can be used to get the raw image / intensity pyramid.
            printf("OUT; pushLiveFrame\n");
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

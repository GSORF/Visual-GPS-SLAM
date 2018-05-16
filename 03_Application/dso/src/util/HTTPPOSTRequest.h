/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   HTTPPOSTRequest.h
 * Author: akp
 *
 * Created on 18. April 2018, 20:25
 */

#include <boost/asio.hpp>
#include <curl/curl.h>
#include "util/FrameShell.h"

#include <iostream>
#include <sstream>
#include <string>

#include <chrono>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>


#ifndef HTTPPOSTREQUEST_H
#define HTTPPOSTREQUEST_H


namespace dso
{
 

class HTTPPOSTRequest
{
public:
    
    inline HTTPPOSTRequest()
    {
        
    }
    
    inline HTTPPOSTRequest(std::string host, std::string protocol)
    {
        init(host, protocol);
    }
    
    inline void init(std::string host, std::string protocol)
    {
        this->host = host;
        this->protocol = protocol;
        this->timestamp = 0;
        this->active = true;

        // Create new project:
        double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        this->projectName = std::to_string((unsigned long)now);
        this->projectDescription = "DSO Project from computer \"" + boost::asio::ip::host_name() + "\"";
        std::cout << "PROJECT NAME=" << this->projectName << " - PROJECT DESCRIPTION=" << this->projectDescription << std::endl;
        std::cout << "TIMESTAMP = " << now;
        
    }
    inline void initProject(std::string name, std::string description)
    {
        this->projectName = name;
        this->projectDescription = description;
        
    }
    
    inline void sendPOST(std::string message)
    {
        // If sending POST requests has been deactivated, return
        if(!this->active)
            return;
        
        //message = "action=addproject&projectname=1524088429&projectdescription=HelloWorld";
        
        std::cout << "SENT POST REQUEST with message: " << message << std::endl;
        
        // Create TCP Stream Object
        boost::asio::ip::tcp::iostream streamPOST;

        // Connect to the server via HTTP port 80
        streamPOST.connect(this->host, this->protocol);

        // Construct HTTP POST request:
        streamPOST << "POST /demo/insert.php HTTP/1.1\r\n";
        streamPOST << "Host: " << this->host << "\r\n";
        streamPOST << "Accept: */*\r\n";
        //streamPOST << "Accept-Encoding: gzip, deflate, br\r\n";
        //streamPOST << "Accept-Language: en-US,en;q=0.5\r\n";
        //streamPOST << "Connection: keep-alive\r\n\r\n";
        streamPOST << "Connection: close\r\n";
        streamPOST << "Content-Type: application/x-www-form-urlencoded; charset=utf-8\r\n";
        streamPOST << "Content-Length: "<< message.length() << "\r\n";
        streamPOST << "\r\n";
        streamPOST << message;
        streamPOST.flush();
        
        std::cout << streamPOST.rdbuf() << std::endl;


        
    }
    
    
    inline void addProject()
    {
        // Example: "action=addproject&projectname=32452351&projectdescription=HelloWorld"
        
        std::string message;
        
        message += "action=addproject";
        message += "&projectname=" + this->projectName;
        message += "&projectdescription=" + urlencode(this->projectDescription);
        
        boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);
    }
    
    inline void addFilteredGPS(unsigned long timestamp, double latitude, double longitude, double accuracy, double bearing, double speed, double altitude, int satellites)
    {
        // Example: "action=addfilteredgps&projectname=" + txtProjectName.value + "&gpsfilteredtimestamp=" + txtFilteredGPSTimestamp.value + "&gpsfilteredlatitude=" + txtFilteredGPSLatitude.value + "&gpsfilteredlongitude=" + txtFilteredGPSLongitude.value + "&gpsfilteredaccuracy=" + txtFilteredGPSAccuracy.value + "&gpsfilteredbearing=" + txtFilteredGPSBearing.value + "&gpsfilteredspeed=" + txtFilteredGPSSpeed.value + "&gpsfilteredaltitude=" + txtFilteredGPSAltitude.value + "&gpsfilteredsatellites=" + txtFilteredGPSSatellites.value;"
        
        std::string message;
        
        message += "action=addfilteredgps";
        message += "&projectname=" + this->projectName;
        message += "&gpsfilteredtimestamp=" + std::to_string(timestamp);
        message += "&gpsfilteredlatitude=" + std::to_string(latitude);
        message += "&gpsfilteredlongitude=" + std::to_string(longitude);
        message += "&gpsfilteredaccuracy=" + std::to_string(accuracy);
        message += "&gpsfilteredbearing=" + std::to_string(bearing);
        message += "&gpsfilteredspeed=" + std::to_string(speed);
        message += "&gpsfilteredaltitude=" + std::to_string(altitude);
        message += "&gpsfilteredsatellites=" + std::to_string(satellites);
        
        boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);
    }
    
    inline void addRawGPS(unsigned long timestamp, double latitude, double longitude, double accuracy, double bearing, double speed, double altitude, int satellites)
    {
        // Example: "action=addrawgps&projectname=" + txtProjectName.value + "&gpsrawtimestamp=" + txtRawGPSTimestamp.value + "&gpsrawlatitude=" + txtRawGPSLatitude.value + "&gpsrawlongitude=" + txtRawGPSLongitude.value + "&gpsrawaccuracy=" + txtRawGPSAccuracy.value + "&gpsrawbearing=" + txtRawGPSBearing.value + "&gpsrawspeed=" + txtRawGPSSpeed.value + "&gpsrawaltitude=" + txtRawGPSAltitude.value + "&gpsrawsatellites=" + txtRawGPSSatellites.value"
        
        std::string message;
        
        message += "action=addrawgps";
        message += "&projectname=" + this->projectName;
        message += "&gpsrawtimestamp=" + std::to_string(timestamp);
        message += "&gpsrawlatitude=" + std::to_string(latitude);
        message += "&gpsrawlongitude=" + std::to_string(longitude);
        message += "&gpsrawaccuracy=" + std::to_string(accuracy);
        message += "&gpsrawbearing=" + std::to_string(bearing);
        message += "&gpsrawspeed=" + std::to_string(speed);
        message += "&gpsrawaltitude=" + std::to_string(altitude);
        message += "&gpsrawsatellites=" + std::to_string(satellites);
        
        boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);
    }
    
    inline void addCameraPose(unsigned long timestamp, Eigen::Vector3d translation, Eigen::Quaterniond quaternion)
    {
        // Example: "action=addpose&projectname=" + txtProjectName.value + "&posetimestamp=" + txtPoseTimestamp.value + "&posex=" + txtPoseX.value + "&posey=" + txtPoseY.value + "&posez=" + txtPoseZ.value + "&poseqw=" + txtPoseQW.value + "&poseqx=" + txtPoseQX.value + "&poseqy=" + txtPoseQY.value + "&poseqz=" + txtPoseQZ.value"
        
        std::string message;
        
        message += "action=addpose";
        message += "&projectname=" + this->projectName;
        message += "&posetimestamp=" + std::to_string(timestamp);
        message += "&posex=" + std::to_string( (double)translation[0] );
        message += "&posey=" + std::to_string( (double)translation[1] );
        message += "&posez=" + std::to_string( (double)translation[2] );
        message += "&poseqw=" + std::to_string(quaternion.w());
        message += "&poseqx=" + std::to_string(quaternion.x());
        message += "&poseqy=" + std::to_string(quaternion.y());
        message += "&poseqz=" + std::to_string(quaternion.z());
        
        
        boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);
    }
    
    inline void addPointCloud(std::string pointString)
    {
        // Example: "action=addpointcloud&projectname=" + txtProjectName.value + "&pointstring=" + txtPointCloud.value"
        
        std::string message;
        
        message += "action=addpointcloud";
        message += "&projectname=" + this->projectName;
        message += "&pointstring=" + pointString;
        
        // TODO: Add back (currently commented due to performance reasons)
        //boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);

    }
    
    inline void addEverything(std::string name,
                              std::string description,
                              unsigned long timestamp,
                              double filteredLatitude, double filteredLongitude, double filteredAccuracy, double filteredBearing, double filteredSpeed, double filteredAltitude, int filteredSatellites,
                              double rawLatitude, double rawLongitude, double rawAccuracy, double rawBearing, double rawSpeed, double rawAltitude, int rawSatellites,
                              Eigen::Vector3d translation, Eigen::Quaterniond quaternion,
                              std::string pointString)
    {
        
        /*
         Example:
         * 
         var data = "action=addeverything";
		data += "&projectname=" + txtProjectName.value + "&projectdescription=" + txtProjectDescription.value;
		data += "&gpsfilteredtimestamp=" + txtFilteredGPSTimestamp.value + "&gpsfilteredlatitude=" + txtFilteredGPSLatitude.value + "&gpsfilteredlongitude=" + txtFilteredGPSLongitude.value + "&gpsfilteredaccuracy=" + txtFilteredGPSAccuracy.value + "&gpsfilteredbearing=" + txtFilteredGPSBearing.value + "&gpsfilteredspeed=" + txtFilteredGPSSpeed.value + "&gpsfilteredaltitude=" + txtFilteredGPSAltitude.value + "&gpsfilteredsatellites=" + txtFilteredGPSSatellites.value;
		data += "&gpsrawtimestamp=" + txtRawGPSTimestamp.value + "&gpsrawlatitude=" + txtRawGPSLatitude.value + "&gpsrawlongitude=" + txtRawGPSLongitude.value + "&gpsrawaccuracy=" + txtRawGPSAccuracy.value + "&gpsrawbearing=" + txtRawGPSBearing.value + "&gpsrawspeed=" + txtRawGPSSpeed.value + "&gpsrawaltitude=" + txtRawGPSAltitude.value + "&gpsrawsatellites=" + txtRawGPSSatellites.value;
		data += "&posetimestamp=" + txtPoseTimestamp.value + "&posex=" + txtPoseX.value + "&posey=" + txtPoseY.value + "&posez=" + txtPoseZ.value + "&poseqw=" + txtPoseQW.value + "&poseqx=" + txtPoseQX.value + "&poseqy=" + txtPoseQY.value + "&poseqz=" + txtPoseQZ.value;
		data += "&pointstring=" + txtPointCloud.value;
         
         */

        std::string message;
        
        message += "action=addeverything";
        message += "&projectname=" + this->projectName;
        message += "&projectdescription=" + urlencode(this->projectDescription);
        
        message += "&gpsfilteredtimestamp=" + std::to_string(timestamp);
        message += "&gpsfilteredlatitude=" + std::to_string(filteredLatitude);
        message += "&gpsfilteredlongitude=" + std::to_string(filteredLongitude);
        message += "&gpsfilteredaccuracy=" + std::to_string(filteredAccuracy);
        message += "&gpsfilteredbearing=" + std::to_string(filteredBearing);
        message += "&gpsfilteredspeed=" + std::to_string(filteredSpeed);
        message += "&gpsfilteredaltitude=" + std::to_string(filteredAltitude);
        message += "&gpsfilteredsatellites=" + std::to_string(filteredSatellites);
        
        message += "&gpsrawtimestamp=" + std::to_string(timestamp);
        message += "&gpsrawlatitude=" + std::to_string(rawLatitude);
        message += "&gpsrawlongitude=" + std::to_string(rawLongitude);
        message += "&gpsrawaccuracy=" + std::to_string(rawAccuracy);
        message += "&gpsrawbearing=" + std::to_string(rawBearing);
        message += "&gpsrawspeed=" + std::to_string(rawSpeed);
        message += "&gpsrawaltitude=" + std::to_string(rawAltitude);
        message += "&gpsrawsatellites=" + std::to_string(rawSatellites);
        
        message += "&posetimestamp=" + std::to_string(timestamp);
        message += "&posex=" + std::to_string( (double)translation[0] );
        message += "&posey=" + std::to_string( (double)translation[1] );
        message += "&posez=" + std::to_string( (double)translation[2] );
        message += "&poseqw=" + std::to_string(quaternion.w());
        message += "&poseqx=" + std::to_string(quaternion.x());
        message += "&poseqy=" + std::to_string(quaternion.y());
        message += "&poseqz=" + std::to_string(quaternion.z());
        
        message += "&pointstring=" + pointString;
        
        boost::thread httpThread(&HTTPPOSTRequest::sendPOST, this, message);
    }
    
    /*
     Source: http://www.zedwood.com/article/cpp-urlencode-function
     Creative Commons CC-By-SA 3.0 
     */
    std::string urlencode(const std::string &s)
    {
        static const char lookup[]= "0123456789abcdef";
        std::stringstream e;
        for(int i=0, ix=s.length(); i<ix; i++)
        {
            const char& c = s[i];
            if ( (48 <= c && c <= 57) ||//0-9
                 (65 <= c && c <= 90) ||//abc...xyz
                 (97 <= c && c <= 122) || //ABC...XYZ
                 (c=='-' || c=='_' || c=='.' || c=='~') 
            )
            {
                e << c;
            }
            else
            {
                e << '%';
                e << lookup[ (c&0xF0)>>4 ];
                e << lookup[ (c&0x0F) ];
            }
        }
        return e.str();
    }

    inline void setActive()
    {
        this->active = true;
    }
    
    inline void setInactive()
    {
        this->active = false;
    }
    
    
    
    
    
    
  
    
    
/*
 * EXAMPLE HTTP REQUEST (note that key-value pairs are separated by a new line)
 * 
Content-Length	
87
Content-type	
application/x-www-form-urlencoded
Host	
master.kalisz.co
Referer	
https://master.kalisz.co/someURL/foo/bar.php
User-Agent	
Mozilla/5.0 (X11; Ubuntu; Linu…) Gecko/20100101 Firefox/59.0
*/
             
    
    
    
protected:

    std::string host;
    std::string protocol;
    bool active; // Determines if POST requests are being sent
    
    std::string projectName;
    std::string projectDescription;
    
    // TODO: Dummy timestamp
    unsigned long timestamp;
};



}

#endif /* HTTPPOSTREQUEST_H */


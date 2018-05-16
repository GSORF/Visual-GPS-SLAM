/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UDPServer.h
 * Author: akp
 *
 * Created on 8. Mai 2018, 00:17
 */

#ifndef UDPSERVER_H
#define UDPSERVER_H

#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include <chrono>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

using boost::asio::ip::udp;

namespace dso
{
    
    
    class UDPServer {
    public:
        
        // Constructor: Initialize to listen on UDP port 2016
        UDPServer(boost::asio::io_service &io_service): socket_(io_service, udp::endpoint(udp::v4(), 2016))
        {
            
            printf("\n----->>>> UDP SERVER CREATED\n");
            timestampLastUpdate_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            
        
            start_receive();
            
            this->hasMeasurement = false;
            //start();
            
        };
    
        void start()
        {
            /*
            if(io_service.stopped())
            {
                io_service.run();
            }
            */
        }
        void stop()
        {
            /*
            if(!io_service.stopped())
            {
                io_service.stop();        
            }
            */
        }
        
        bool getMeasurement(double &latitude, double &longitude, double &altitude, double &accuracy)
        {
            bool success = false;
            if (!this->hasMeasurement) return success;
            
            // Lock mutex to synchronize threads and avoid racing conditions
            mtx_.lock();
            
            double timestampCurrent = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            
            // TODO: Check for age of measurement (in milliseconds). Set to 2 Seconds for now (too long still?)
            if( fabs(timestampLastUpdate_ - timestampCurrent) < 2000.0 )
            {
                // If the measurement is not too old, return it back

                latitude = this->latitude_;
                longitude = this->longitude_;
                altitude = this->altitude_;
                accuracy = this->accuracy_;
                
                success = true;
            }
            
            // Unlock mutex to let go of thread
            mtx_.unlock();
            
            
            
            
            if(success)
                return true;
            else
                return false;
            
        }

        
        virtual ~UDPServer();
    private:
        
        std::string make_daytime_string()
        {
          using namespace std; // For time_t, time and ctime;
          time_t now = time(0);
          return ctime(&now);
        }

        void start_receive()
        {
          socket_.async_receive_from(
              boost::asio::buffer(recv_buffer_), remote_endpoint_,
              boost::bind(&UDPServer::handle_receive, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred));
        }
        
        void handle_receive(const boost::system::error_code& error,
            std::size_t bytes_transferred)
        {
          // Ignore anything that is larger than our buffer
          if (!error || error == boost::asio::error::message_size)
          {
            /*
             The datagram contains a message like:
             1.) "VGPSSLAM_FindVGPSServer"
             2.) "VGPSSLAM_VGPSServerIsHere";
             3.) "GPS#" + location.getLatitude() + "#" + location.getLongitude() + "#" + location.getAccuracy() 
            */
              
            // Complicated way to just get a string from boost...  
            std::string received;
            std::copy(recv_buffer_.begin(), recv_buffer_.begin()+bytes_transferred, std::back_inserter(received));
            std::cout << "Received from UDP: " << received << std::endl;
            
            
            // Internal UDP Logic - determine what to do with the received datagram
            if(received == msgFindServer)
            {
                // Broadcast message to get IP from GPS receiver was received (smartphone)
                boost::shared_ptr<std::string> message(
                                new std::string(msgServerIsHere));
            
                socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                boost::bind(&UDPServer::handle_send, this, message,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
                
            }
            else if(received == msgServerIsHere)
            {
                // We ignore messages that tell us where the server is, because we ARE the server ;)
                
            }
            else if(received.compare(0, msgPayloadStart.length(), msgPayloadStart) == 0)
            {
                // Lock mutex to synchronize threads and avoid racing conditions
                mtx_.lock();

                // Parse String to get individual parts of gps data:
                std::vector<std::string> string_parts;
                boost::split(string_parts, received, boost::is_any_of("#"));

                if(string_parts.size() == 4)
                {
                    // There are four parts: "GPS", "latitude", "longitude", "accuracy"
                    // E.g.: GPS#49.4144637#11.1298899#20.903
                    
                    // Update timestamp for last update
                    this->timestampLastUpdate_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    // Update gps data
                    this->latitude_ = std::stod(string_parts.at(1));
                    this->longitude_ = std::stod(string_parts.at(2));
                    this->altitude_ = 0;
                    this->accuracy_ = std::stod(string_parts.at(3));
                    
                    this->hasMeasurement = true;
                }
                
                // Unlock mutex to let go of thread
                mtx_.unlock();
                
                /*
                 Example Code for sending a message (this is not used)
                
                boost::shared_ptr<std::string> message(
                new std::string(make_daytime_string()));
                
                socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                boost::bind(&UDPServer::handle_send, this, message,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
                */
            }
            
            
            // Continue to receive UDP datagrams
            start_receive();

          }
        }

        
        void handle_send(boost::shared_ptr<std::string> /*message*/,
                         const boost::system::error_code& /*error*/,
                         std::size_t /*bytes_transferred*/)
        {
            //This function is invoked after the service request has been completed. 
        }
        

        udp::socket socket_;
        udp::endpoint remote_endpoint_;
        boost::array<char, 1024> recv_buffer_;

        double latitude_;
        double longitude_;
        double altitude_;
        double accuracy_;
        double timestampLastUpdate_;
        
        const std::string msgFindServer = "VGPSSLAM_FindVGPSServer";
        const std::string msgServerIsHere = "VGPSSLAM_VGPSServerIsHere";
        const std::string msgPayloadStart = "GPS";
        
        bool hasMeasurement;
        
        boost::mutex mtx_;
        //boost::thread udpThread_;
    };

}
#endif /* UDPSERVER_H */


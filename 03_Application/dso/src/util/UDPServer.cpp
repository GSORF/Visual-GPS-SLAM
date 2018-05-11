/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UDPServer.cpp
 * Author: akp
 * 
 * Created on 8. Mai 2018, 00:17
 */


#include "UDPServer.h"

namespace dso
{
    
    
    
    
    UDPServer::~UDPServer() {
        printf("\n----->>>> UDP SERVER DESTROYED\n");
        
        socket_.close();
    }

}
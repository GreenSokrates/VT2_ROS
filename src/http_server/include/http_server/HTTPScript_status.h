/*
 * HTTPScript.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef HTTP_SCRIPT_STATUS_H_
#define HTTP_SCRIPT_STATUS_H_

#include <cstdlib>
#include <string>
#include <vector>
#include <http_server/HTTPScript.h>
#include <ros/ros.h>

using namespace std;

/**
 * This is the abstract http script superclass that needs to be derived
 * by application specific http scripts.
 * @see HTTPServer
 */
class HTTPScript_status : public HTTPScript {

    public:

                        HTTPScript_status(){};
        virtual         ~HTTPScript_status(){};
        string  call(vector<string> names, vector<string> values);
};

#endif /* HTTP_SCRIPT_STATUS_H_ */

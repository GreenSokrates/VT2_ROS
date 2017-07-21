/*
 * HTTPScript.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef HTTP_SCRIPT_MONTAGE_H_
#define HTTP_SCRIPT_MONTAGE_H_

#include <cstdlib>
#include <string>
#include <vector>
#include <http_server/HTTPScript.h>
#include <cell_core/montage_service.h>
#include <ros/ros.h>

using namespace std;

/**
 * This is the abstract http script superclass that needs to be derived
 * by application specific http scripts.
 * @see HTTPServer
 */
class HTTPScript_montage : public HTTPScript {

    public:

                        HTTPScript_montage(){};
        virtual         ~HTTPScript_montage(){};
        string  call(vector<string> names, vector<string> values);
        bool callService(double Offset_, int Ausgabestelle_);
};

#endif /* HTTP_SCRIPT_MONTAGE_H_ */

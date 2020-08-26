/*
 * HTTPScript.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef HTTP_SCRIPT_H_
#define HTTP_SCRIPT_H_

#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

/**
 * This is the abstract http script superclass that needs to be derived
 * by application specific http scripts.
 * @see HTTPServer
 */
class HTTPScript {

    public:

                        HTTPScript();
        virtual         ~HTTPScript();
        virtual string  call(vector<string> names, vector<string> values);
};

#endif /* HTTP_SCRIPT_H_ */

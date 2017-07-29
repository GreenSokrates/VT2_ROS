/*
 * HTTPClient.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 16.03.2016
 *      Author: Marcel Honegger
 */

#ifndef HTTP_CLIENT_H_
#define HTTP_CLIENT_H_

#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <unistd.h>
#include <netdb.h>
#include <stdint.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>

using namespace std;

/**
 * The <code>HTTPClient</code> class allows to retrieve the contents of a given URL.
 * <br/>
 * An object of this class offers a <code>get()</code> method that accepts an URL
 * as a text string, and returns the contents of that URL as a text string.
 * <br/><br/>
 * The following examples show how to use this object and how the URL may be given:
 * <pre><code>
 *   HTTPClient httpClient();    <span style="color:#008000">// creates a http client object</span>
 *
 *   <span style="color:#0000FF">try</span> {
 *
 *     string response = httpClient.get(<span style="color:#800080">"http://192.168.1.1:80/index.html"</span>);  <span style="color:#008000">// URL given with IP address</span>
 *     string anotherResponse = httpClient.get(<span style="color:#800080">"www.zhaw.ch"</span>);                <span style="color:#008000">// URL given as a domain name</span>
 *
 *   } <span style="color:#0000FF">catch</span> (Exception& e) {
 *     cerr << e.what() << endl;
 *   }
 * </code></pre>
 * The <code>get()</code> method optionally allows to keep the connection to the server alive,
 * which has significant performance advantages when a given URL needs to be read continuously,
 * i.e. many times per second.
 * <br/>
 * Note that the <code>get()</code> method may throw a runtime error, when the connection to the server
 * cannot be established, or when the contents of the URL cannot be read.
 */
class HTTPClient {

    public:
        
                    HTTPClient();
                    HTTPClient(uint32_t bufferSize);
        virtual     ~HTTPClient();
        string      get(string url);
        string      get(string url, bool keepAlive);
        string 		get(string url, bool keepAlive, uint32_t readTimeout);
        
    private:
        
        static const uint32_t   BUFFER_SIZE = 4096;         // default size of the input/read buffer, in [bytes]
        static const string     DEFAULT_PATH;               // default path to contents
        static const uint16_t   PORT_NUMBER = 80;           // default port number of the server
        static const uint32_t   RECEIVE_TIMEOUT = 60000;    // initial receive timeout in [ms] (60000 = one minute)
        static const uint32_t   READ_TIMEOUT = 200;         // continuous read timeout in [ms]
        
        uint32_t    bufferSize;
        string      hostname;
        uint16_t    portNumber;
        int32_t     clientSocket;
};

#endif /* HTTP_CLIENT_H_ */

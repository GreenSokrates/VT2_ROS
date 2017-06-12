/*
 * HTTPServer.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <string>
#include <vector>
#include <list>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <exception>
#include <stdexcept>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <stdint.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "Thread.h"

using namespace std;

class HTTPScript;
class HTTPClientThread;

/**
 * The <code>HTTPServer</code> class implements a simple webserver that is able to
 * transmit files over an ethernet connection and allows to call scripts that are
 * registered with the server.
 * <br/>
 * An http server can be created and started as follows:
 * <pre><code>
 *   HTTPServer* httpServer = new HTTPServer(8080);  <span style="color:#008000">// creates a server on port 8080</span>
 *   httpServer->start();                            <span style="color:#008000">// starts the server thread</span>
 *   ...
 * </code></pre>
 * Apart from delivering static files, this http server allows to execute application
 * specific code implemented as http scripts. These scripts are objects derived from
 * the <code>HTTPScript</code> superclass.
 * <br/>
 * An example of an application specific script is given below:
 * <pre><code>
 * class MyHTTPScript : public HTTPScript {
 *     public:
 *                  MyHTTPScript();
 *         virtual  ~MyHTTPScript();
 *         string   call(vector<string> names, vector<string> values);
 * };
 * 
 * string MyHTTPScript::call(vector<string> names, vector<string> values) {
 *   
 *   string response;
 *   
 *   response += "  <h2>";
 *   for (uint32_t i = 0; i < min(names.size(), values.size()); i++) {
 *     response += "  <p>"+names[i]+"="+values[i]+"</p>";
 *   }
 *   response += "  </h2>";
 * 
 *   return response;
 * }
 * </code></pre>
 * This script returns the parameters that were passed to it by the http server.
 * <br/>
 * Before this script can be used, it needs to be registered with the http server
 * with the <code>add()</code> method as follows:
 * <pre><code>
 *   httpServer->add("myScript", new MyHTTPScript());
 * </code></pre>
 * When the <code>call()</code> method of the script is called by the http server,
 * it receives two string vectors: a vector with the names of the arguments passed
 * in the URL, and a vector with the corresponding values of the arguments.
 * <br/>
 * An example of an http request calling this script is as follows:
 * <pre><code>
 *   http://192.168.1.1:8080/cgi-bin/myScript?x=0.5&y=-0.1&z=0.2
 * </code></pre>
 * The vectors of arguments passed to the <code>call()</code> method are then
 * {'x', 'y', 'z'} for the names and {'0.5', '-0.1', '0.2'} for the values.
 * <br/>
 * The response of the <code>call()</code> method is a <code>string</code> object
 * which is placed within an xhtml page, which in turn is returned by the http
 * server to the requesting http client.
 * @see HTTPScript
 */
class HTTPServer : public Thread {
    
    friend class HTTPClientThread;
    
    public:
        
                        HTTPServer();
                        HTTPServer(uint16_t portNumber);
                        HTTPServer(uint16_t portNumber, string path, string indexPage);
        virtual         ~HTTPServer();
        void            add(string name, HTTPScript* httpScript);
    
    private:
        
        static const size_t         STACK_SIZE = 64*1024;       // stack size of server thread in [bytes]
        static const uint16_t       PORT_NUMBER = 80;           // default port number of this server
        static const uint32_t       BUFFER_SIZE = 8192;         // size of the input/read buffer, in [bytes]
        static const uint32_t       BIND_RETRY_DELAY = 1000;    // delay before retrying to bind, in [ms]
        static const uint32_t       READ_RETRY_DELAY = 10;      // delay before retrying to read input, in [ms]

        uint16_t                    portNumber;
        string                      path;
        string                      indexPage;
        vector<string>              httpScriptNames;
        vector<HTTPScript*>         httpScripts;
        list<HTTPClientThread*>     httpClients;

        string          urlDecoder(string url);
        void            run();
};

class HTTPClientThread : public Thread {
    
    public:
        
                        HTTPClientThread(HTTPServer* httpServer, int32_t clientSocket);
        virtual         ~HTTPClientThread();
        void            run();
    
    private:
        
        HTTPServer*     httpServer;
        int32_t         clientSocket;
};

#endif /* HTTP_SERVER_H_ */

/*
 * HTTPClient.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 16.03.2016
 *      Author: Marcel Honegger
 */

#include <http_server/HTTPClient.h>

using namespace std;

const string HTTPClient::DEFAULT_PATH = "index.html"; // default path to contents

template <class T>
inline string type2String(const T &t)
{

    stringstream out;
    out << t;

    return out.str();
}

/**
 * Creates a <code>HTTPClient</code> object.
 */
HTTPClient::HTTPClient()
{

    bufferSize = BUFFER_SIZE;
    hostname = "";
    portNumber = PORT_NUMBER;
    clientSocket = -1;
}

/**
 * Creates a <code>HTTPClient</code> object.
 * @param bufferSize the size of the read buffer in [bytes].
 */
HTTPClient::HTTPClient(uint32_t bufferSize)
{

    this->bufferSize = bufferSize;
    this->hostname = "";
    this->portNumber = PORT_NUMBER;
    this->clientSocket = -1;
}

/**
 * Deletes the <code>HTTPClient</code> object and releases all allocated resources.
 */
HTTPClient::~HTTPClient()
{

    if (clientSocket)
        close(clientSocket);
}

/**
 * This method allows to retrieve the contents of a given URL.
 * Note that this method throws a runtime error in case of a failure.
 * @param url a given URL to retrieve the contents from.
 * @return a string object with the contents of the given URL.
 */
string HTTPClient::get(string url)
{

    return get(url, false, READ_TIMEOUT);
}

/**
 * This method allows to retrieve the contents of a given URL.
 * Note that this method throws a runtime error in case of a failure.
 * @param url a given URL to retrieve the contents from.
 * @param keepAlive a flag to determine if the connection to the server should be kept alive.
 * @return a string object with the contents of the given URL.
 */
string HTTPClient::get(string url, bool keepAlive)
{

    return get(url, keepAlive, READ_TIMEOUT);
}

/**
 * This method allows to retrieve the contents of a given URL.
 * Note that this method throws a runtime error in case of a failure.
 * @param url a given URL to retrieve the contents from.
 * @param keepAlive a flag to determine if the connection to the server should be kept alive.
 * @param readTimeout the time that this function waits for data, given in [ms].
 * @return a string object with the contents of the given URL.
 * 
 */
string HTTPClient::get(string url, bool keepAlive, uint32_t readTimeout)
{

    // parse url string

    size_t index = 0;
    if ((index = url.find("http://")) != string::npos)
    {
        if (url.size() > index + 7)
        {
            url = url.substr(index + 7); // remove head of url string
        }
        else
        {
            // leave url string as is
        }
    }
    else
    {
        // leave url string as is
    }

    string hostname;
    if ((index = url.find("/")) != string::npos)
    {
        hostname = url.substr(0, index);
        if (url.size() > index + 1)
        {
            url = url.substr(index + 1);
        }
        else
        {
            url = DEFAULT_PATH;
        }
    }
    else
    {
        hostname = url;
        url = DEFAULT_PATH;
    }

    uint16_t portNumber = PORT_NUMBER;
    if ((index = hostname.find(":")) != string::npos)
    {
        if (hostname.size() > index + 1)
        {
            portNumber = static_cast<uint16_t>(strtol(hostname.substr(index + 1).c_str(), NULL, 10));
        }
        else
        {
            // leave default port number
        }
        hostname = hostname.substr(0, index);
    }
    else
    {
        // leave default port number
    }

    url = "GET /" + url + " HTTP/1.1\r\nHost: " + hostname + ":" + type2String(portNumber) + "\r\n\r\n";

    // check if previous connection can be reused

    if (!keepAlive || (hostname.compare(this->hostname) != 0) || (portNumber != this->portNumber))
    {

        this->hostname = hostname;
        this->portNumber = portNumber;

        if (clientSocket)
        {
            close(clientSocket);
            clientSocket = -1;
        }

        // establish new connection to server */

        addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = PF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo *res0;

        if (getaddrinfo(hostname.c_str(), "http", &hints, &res0) != 0)
            throw runtime_error("HTTPClient: no such host");

        clientSocket = -1;
        for (addrinfo *res = res0; res; res = res->ai_next)
        {
            clientSocket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
            if (clientSocket < 0)
            {
                // do nothing, go to next iteration
            }
            else if (connect(clientSocket, res->ai_addr, res->ai_addrlen) < 0)
            {
                close(clientSocket);
                clientSocket = -1;
            }
            else
            {
                break; // terminate iteration, since connection could be established
            }
        }

        freeaddrinfo(res0);

        if (clientSocket < 0)
            throw runtime_error("HTTPClient: connecting error");
    }

    // send request

    if (write(clientSocket, url.c_str(), url.size()) < 0)
        throw runtime_error("HTTPClient: error writing to socket");

    // receive response

    string response;

    timeval tv;
    tv.tv_sec = RECEIVE_TIMEOUT / 1000;
    tv.tv_usec = (RECEIVE_TIMEOUT % 1000) * 1000;
    setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

    char buffer[bufferSize];
    bzero(buffer, bufferSize);
    ssize_t size = read(clientSocket, buffer, bufferSize - 1); // cout << "HTTPClient: read first..." << endl;

    if (size < 0)
    {

        // couldn't read from socket

        throw runtime_error("HTTPClient: error reading from socket");
    }
    else
    {

        // reconfigure the receive timeout of the socket

        tv.tv_sec = readTimeout / 1000;
        tv.tv_usec = (readTimeout % 1000) * 1000;
        setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

        // read while data is available

        while (size > 0)
        {

            response += string(buffer, static_cast<size_t>(size));
            bzero(buffer, bufferSize);
            size = read(clientSocket, buffer, bufferSize - 1); //cout << "HTTPClient: read again..." << endl;
        }
    }

    if (!keepAlive)
    {
        close(clientSocket);
        clientSocket = -1;
    }

    return response;
}

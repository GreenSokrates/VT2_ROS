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
#include <ros/ros.h>
#include <http_server/HTTPScript.h>
#include <http_server/HTTPNode.h>
#include <cell_core/status_msg.h>

using namespace std;

/**
 * This is the abstract http script superclass that needs to be derived
 * by application specific http scripts.
 * @see HTTPServer
 */
class HTTPScript_status : public HTTPScript
{

protected:
  ros::NodeHandle nh;
  ros::Subscriber StatusSub;

public:
  HTTPScript_status();
  virtual ~HTTPScript_status(){};
  string call(vector<string> names, vector<string> values);
  void statusCallback(const cell_core::status_msg &msg);
  bool idle_;
  bool error_;
};

#endif /* HTTP_SCRIPT_STATUS_H_ */

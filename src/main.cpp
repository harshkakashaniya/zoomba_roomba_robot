/************************************************************************
 MIT License

 Copyright (c) 2018 Harsh Kakashaniya

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @file    main.cpp
 *  @author  Harsh Kakashaniya
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, ROS tutorials.
 *
 *  @Description DESCRIPTION
 *
 *  This file is used to publish message to turtle bot to avoid obstacle and
 *   move around
 *
 */

#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
 {

ros::init(argc, argv, "smart");

ros::NodeHandle nh;
ros::Publisher smart_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  geometry_msgs::Twist command;
  ros::Rate loop_rate(5.0);
   // Here you build your twist message
   command.linear.x = 1;
   command.linear.y = 0;
   command.linear.z = 0;

   command.angular.x = 0;
   command.angular.y = 0;
   command.angular.z = 22/7;

   while(ros::ok() )
   {
       smart_pub.publish(command);

       // Time between messages, so you don't blast out an thousands of
       // messages in your 3 secondperiod
       loop_rate.sleep();
   }

   return 0;
    }

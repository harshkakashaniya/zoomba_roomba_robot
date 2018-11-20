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

// required libraries added
#include <sstream>
#include <iostream>
// ROS package added
#include "ros/ros.h"
// type of message we are publishing to
#include "geometry_msgs/Twist.h"
// class from which commands are executed
#include "../include/Walker.hpp"

int main(int argc, char **argv) {
ros::init(argc, argv, "smart");
Walker Bot;  // Member of class Walker
Bot.Motion();  // method for robot movement
  return 0;
}

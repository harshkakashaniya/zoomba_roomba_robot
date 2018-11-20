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
 *  @file    Walker.hpp
 *  @author  Harsh Kakashaniya
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, ROS tutorials.
 *
 *  @Description DESCRIPTION
 *
 *  This file is used to define methods and variables for sensor data and command robot to move
 *
 */
  #ifndef INCLUDE_WALKER_HPP_
  #define INCLUDE_WALKER_HPP_
// ROS package
#include <ros/ros.h>
// For moving robot according to our wish publishing data
#include <geometry_msgs/Twist.h>
// For having scan of laser sensor
#include "sensor_msgs/LaserScan.h"


class Walker {
 public:
	/**
    *   @brief Default constructor for Walker
    *
    */
  Walker();
	/**
    *   @brief Default disstructor for Walker
    *
    */

  ~Walker();
	/**
   *   @brief Method to scan laser and find distance of obstacles.
   *
   *   @param laser sensor data.
   */
  void LaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);
	/**
   *   @brief Method to move robot according to laser output
   *
   *   @param none
   */
  void Motion();

 private:
  ros::NodeHandle nh;  // to handle ros commands
  bool movement;  // to assist motion with laser output
  ros::Subscriber laserSubscribe;  // object to subscribe laser data
  ros::Publisher velocityPublish;  // object to publish robot movement data
  float SetRange;  // range of object from camera permissible
  geometry_msgs::Twist msg;  // message to publish
};

#endif  // INCLUDE_WALKER_HPP_

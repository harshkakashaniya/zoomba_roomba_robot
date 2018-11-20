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
 *  @file    Walker.cpp
 *  @author  Harsh Kakashaniya
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, ROS tutorials.
 *
 *  @Description DESCRIPTION
 *
 *  This file is used to implement sensor data and command robot to move
 *
 */
#include "../include/Walker.hpp"
/**
  *   @brief Default constructor for Walker
  *
  */
Walker::Walker() {
  SetRange = 1;  // set range of 1 m for trial
  laserSubscribe = nh.subscribe < sensor_msgs::LaserScan
      > ("/scan", 10, &Walker::LaserScan, this);  // subscribe LaserScan

  ROS_INFO("Range of %f m is set to avoid obstacles.", SetRange);
  movement = true;
}
/**
 *   @brief Method to scan laser and find distance of obstacles.
 *
 *   @param laser sensor data.
 */
void Walker::LaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
  float minDis = scan->ranges[0];  // Let first element be minimum distance
  // for loop for minimum distance
    for (int i = 0; i < scan->ranges.size(); i++) {
      float currentVal = scan->ranges[i];
    // comparison of values
    if (currentVal < minDis) {
        minDis = currentVal;  // minimum value found in array
    }
  }
  // obstacle near by signal
  if (minDis < SetRange) {
    ROS_WARN("Crash! chances, Distance = %f", minDis);  // distance less than
    movement = false;
    } else {
    ROS_INFO("Working Fine!! Go straight %f", minDis);  // every thing ok
    movement = true;
    }
}

/**
 *   @brief Method to move robot according to laser output
 *
 *   @param none
 */
void Walker::Motion() {
ros::Rate loop_rate(5.0);
// publish velocity
velocityPublish = nh.advertise<geometry_msgs::Twist>(
"cmd_vel_mux/input/teleop", 1000);
geometry_msgs::Twist command;
// Here you build your twist message


  while (ros::ok()) {
    if (movement) {
      ROS_INFO("Translating %f", command.linear.x);
      command.linear.x = 1;
      command.angular.z = 0;
      } else {
      ROS_INFO("Rotating %f", command.linear.x);
      command.linear.x = 0;
      command.angular.z = 0.8;
      }
     velocityPublish.publish(command);
     ros::spinOnce();
     loop_rate.sleep();
}
}
/**
  *   @brief Default disstructor for Walker
  *
  */
Walker::~Walker() {
  geometry_msgs::Twist command;
  command.linear.x = 0;
  command.linear.y = 0;
  command.linear.y = 0;
  command.angular.z = 0;
  command.angular.y = 0;
  command.angular.x = 0;
}

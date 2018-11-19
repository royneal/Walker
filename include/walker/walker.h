/**
 * @file    walker.h
 * @author  Royneal Rayess
 * @copyright MIT License (c) 2018 Royneal Rayess
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @brief DESCRIPTION
 * Class for demonstrating obstacle avoidance using turtlebot and gazebo
 *
 */

#pragma once


#include <ros/console.h>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


/**
 * @brief      A class for implementing obstacle  avoidance
 */
class Walker {
    public:
        Walker();
        void Explore();
        void SettxRate(const int& rate);
        void LmsCallbck(const sensor_msgs::LaserScanConstPtr& scan);
        geometry_msgs::Twist MoveFwd(const double& fwdvel);
        geometry_msgs::Twist Rotate(const double& deg);
        geometry_msgs::Twist RotCW(const double& deg);
        geometry_msgs::Twist RotCCW(const double& deg);

    private:
        ros::NodeHandle nh_;  // ros handle
        ros::Publisher pub_;  // ros publisher object
        ros::Subscriber sub_;  // ros subscriber object
        static int msg_rate_;
        int rotation_direction_;
        geometry_msgs::Twist velocity_;
};

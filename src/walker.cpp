/**
 * @file    walker.cpp
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
 * this file is the implementation of Class walker, it advertises and suscribes to
 * the appropriate topics to make the turtle bot move around and avoid obstacles 
 *
 */

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include "walker/walker.h"
#include "geometry_msgs/Twist.h"

// msg_rate_ is declared as a const in header file
// it has to be declared again here and initialized
// before it can be used.
int Walker::msg_rate_ = 1;

/**
 * @brief      default class constructor, advertises to velocity topic
 */
Walker::Walker() {
    pub_ = nh_.advertise<geometry_msgs::Twist>
                    ("/mobile_base/commands/velocity", 10);
    sub_ = nh_.subscribe("/scan", 1000, &Walker::LmsCallbck, this);
    velocity_.linear.x = 0;
    velocity_.linear.y = 0;
    velocity_.linear.z = 0;
    velocity_.angular.x = 0;
    velocity_.angular.y = 0;
    velocity_.angular.z = 0;
}

/**
 * @brief      method to set private member msg_rate_ to new rate
 * @param      rate holds new publishing rate requested by client
 *  
 */
void Walker::SettxRate(const int& rate) {
    msg_rate_ = rate;
}

/**
 * @brief      sets the message values to move robot forward
 * @param      fwdvel double sets the direction and velocity
 * @return     returns a standard gemometry_msgs::Twist message
 */
geometry_msgs::Twist Walker::MoveFwd(const double& fwdvel) {
     velocity_.linear.x = fwdvel;  // set fwd velocity of robot
     velocity_.angular.z =0;  // set rotation of robot to zero
     return velocity_;
}

/**
 * @brief      sets Twist message variables to rotate clockwise
 * @param      linear.x  double sets the direction and velocity
 * @param      angular.z double sets the roatation of robot
 * @return     returns a standard gemometry_msgs::Twist message
 */
geometry_msgs::Twist Walker::RotCW(const double& deg) {
     velocity_.linear.x = 0.3;  // set velocity of robot to 0.3
     velocity_.angular.z = -1;  // set rotation of robot to -1
    return velocity_;
}

/**
 * @brief      sets Twist message variables to counter clockwise
 * @param      linear.x  double sets the direction and velocity
 * @param      angular.z double sets the roatation of robot
 * @return     returns a standard gemometry_msgs::Twist message
 */
geometry_msgs::Twist Walker::RotCCW(const double& deg) {
     velocity_.linear.x = 0.3;  // set velocity of robot to 0.3
     velocity_.angular.z = 1;  // set rotation of robot to 1
    return velocity_;
}

/**
 * @brief      laser callback updates laer readings 
 * @param      pointer to lser scan data
 */
void Walker::LmsCallbck(const sensor_msgs::LaserScanConstPtr& scan) {
    auto min_dis = std::min_element(scan->ranges.begin(),
                                            scan->ranges.end());
    double trigger_threshold = 3;
    if (*min_dis <= trigger_threshold) {
        ROS_INFO_STREAM("Threshold Triggered");
        double size = scan->ranges.end() - scan->ranges.begin();
        auto iter = std::find(scan->ranges.begin(),
                                scan->ranges.end(), *min_dis);

        auto location = iter - scan->ranges.begin();

        if ((location) >= (size/2)) {
            ROS_INFO_STREAM("Obstacle On the left");
            rotation_direction_ = 1;  // rotate right for example
        }

        if ((location) < (size/2)) {
            ROS_INFO_STREAM("Obstacle On the right");
            rotation_direction_ = 2;  // rotate left for exampl
        }
    } else {
    rotation_direction_ = 0;  // no rotation
    }
}

/**
 * @brief      implements Obstackle avoidance 
 *     
 */
void Walker::Explore() {
    ros::Rate loop_rate(msg_rate_);  // rate at which messages get published
    pub_.publish(MoveFwd(0.2));  // publish message to topic

    while (ros::ok()) {
        ros::Rate loop_rate(msg_rate_);  // rate at which messages get published
        if (rotation_direction_ == 0) {
            ROS_INFO_STREAM("No Rotation");
            pub_.publish(MoveFwd(0.2));  // publish message to topic
        }
        while (rotation_direction_ == 1) {
            ROS_INFO_STREAM("Rotating CW");
            pub_.publish(RotCW(60));  // publish message to topic
            ros::spinOnce();
            loop_rate.sleep();
        }
       while (rotation_direction_ == 2) {
            ROS_INFO_STREAM("Rotating CCW");
            pub_.publish(RotCCW(-60));  // publish message to topic
            ros::spinOnce();
            loop_rate.sleep();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

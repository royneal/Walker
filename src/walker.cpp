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
 * this node is a client that allows changing the publishing rate of
 * the publisher_subscriber_node in this beginner_tutorials package.
 *
 */

#include <string>
#include "walker/walker.h"
#include "geometry_msgs/Twist.h"


// msg_rate_ is declared as a const in header file
// it has to be declared again here and initialized
// before it can be used. 

int Walker::msg_rate_ = 1; 
/**
 * @brief      default class constructor, advertises the chatter topic
 */
Walker::Walker() {
    pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
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

geometry_msgs::Twist Walker::MoveFwd(const double& fwdvel) {
     velocity_.linear.x = fwdvel;  // velocity command message type 
     velocity_.linear.y = fwdvel;
     return velocity_;
     
}

geometry_msgs::Twist Walker::Rotate(const double& deg) {
    velocity_.angular.z = deg;
    return velocity_;
}

/**
 * @brief      method to publis messages on chatter topic
 * @param      msg string to be published on topic
 */
void Walker::Explore(const int& vel) {
    ros::Rate loop_rate(msg_rate_);  // rate at which messages get published
     
    while (ros::ok()) {
        ros::Rate loop_rate(msg_rate_);  // rate at which messages get published
        pub_.publish(MoveFwd(2));  // publish message to topic

        ros::spinOnce();
        loop_rate.sleep();
    }
}
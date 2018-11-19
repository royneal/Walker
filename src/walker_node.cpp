/**
 * @file    walker_node.cpp
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
 * main node program  that instantiates a walker object which controls the 
 * turtle bot and implements obstacle avoidance. 
 *
 */
#include <string>
#include "walker/walker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker_node");
    ros::NodeHandle nh("~");
    int rate;
    int stat = 0;
    nh.getParam("txstat", stat);
    if (nh.getParam("txrate", rate)) {
      ROS_INFO("Publishing rate set to: %d", rate);
    } else {
      ROS_WARN("Failed to get param 'txrate' setting to Default = 1");
    }

    Walker turtle_bot;  // create a publisher object
    turtle_bot.SettxRate(rate);

    int velocity = 10;  // message to publish
    if (stat == 1) {
      ROS_INFO("Publisher is enabled");
      turtle_bot.Explore();  // publish velocity to turtlebot
    } else {
      ROS_WARN_STREAM("stat = " << stat);
      ROS_WARN("Publisher is disabled");
    }
    return 0;
}

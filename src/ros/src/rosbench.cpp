/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * Copyright (C) 2013, Georiga Tech Research Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "rosbench.h"
#include "ipcbench/ros_timespec.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"

#include <sstream>
#include <signal.h>
#include <time.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int publish(int argc, char **argv, int freq)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<ipcbench::ros_timespec>("chatter", 1000);

    ros::Rate loop_rate(freq);

    int count = 0;
    while (ros::ok())
    {
        //std_msgs::String msg;
        struct timespec ts;
        clock_gettime( CLOCK_MONOTONIC, &ts );
        ipcbench::ros_timespec rts;
        rts.sec = ts.tv_sec;
        rts.nsec = ts.tv_nsec;

        // std::stringstream ss;
        // ss << "hello world " << count;
        // msg.data = ss.str();
        // ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(rts);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


    return 0;
}


void chatterCallback(const ipcbench::ros_timespec::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%lu, %u]", msg->sec, msg->nsec );
    get_delta( msg->sec, msg->nsec );
}

int subscribe(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    return 0;
}

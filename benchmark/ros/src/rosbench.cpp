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

int publish(int argc, char **argv, int freq)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<ipcbench::ros_timespec>("chatter", 1000);

    ros::Rate loop_rate(freq);

    int count = 0;
    while (ros::ok())
    {
        struct timespec ts;
        clock_gettime( CLOCK_MONOTONIC, &ts );
        ipcbench::ros_timespec rts;
        rts.sec = ts.tv_sec;
        rts.nsec = ts.tv_nsec;

        chatter_pub.publish(rts);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

static int s_emit;
static uint64_t s_discarded = 0;
static uint64_t s_discard = 0;
static mqd_t s_mq;

void chatterCallback(const ipcbench::ros_timespec::ConstPtr& msg)
{
    if( s_emit && s_discarded++ > s_discard ) {
        get_delta( msg->sec, msg->nsec, s_mq );
    }
}

int subscribe(int argc, char **argv, int num, int emit, enum rosbench_transport t, uint64_t discard, mqd_t mq)
{
    s_emit = emit;
    s_discard = discard;
    s_mq = mq;

    char buf[32]={0};
    {
        strcpy(buf, "listener_");
        size_t n = strlen(buf);
        snprintf(buf+n, sizeof(buf)-n-1, "%d", num);
    }

    ros::init(argc, argv, buf );

    ros::NodeHandle node;
    ros::Subscriber sub;

    switch(t) {
    case ROS_TCP:
        sub = node.subscribe("chatter", 1000, chatterCallback, ros::TransportHints().tcp());
        break;
    case ROS_UDP:
        sub = node.subscribe("chatter", 1000, chatterCallback, ros::TransportHints().udp());
        break;
    default:
        abort();
    }

    ros::spin();

    return 0;
}

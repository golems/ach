/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROSBENCH_H
#define ROSBENCH_H

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <mqueue.h>

#ifdef __cplusplus
extern "C" {
#endif



static double ticks_delta(struct timespec t0, struct timespec t1) {
    double dsec =  ((double)t1.tv_sec -  (double)t0.tv_sec);
    double dnsec = ((double)t1.tv_nsec - (double)t0.tv_nsec) / 1e9;
    return dsec + dnsec;// - overhead;
}



#define STACK_SIZE (8*1024)

static void make_realtime( int priority ) {
    char prefault[STACK_SIZE];
    memset( prefault,0,sizeof(prefault) );

    if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
        fprintf(stderr, "Couldn't lock pages in memory: %s\n",
                strerror(errno) );
        abort();
    }
    struct sched_param sp;
    sp.sched_priority = priority; /* 99 is max priority on linux */
    if( sched_setscheduler( 0, priority ? SCHED_FIFO : SCHED_OTHER, &sp) < 0 ) {
        fprintf(stderr, "Couldn't set scheduling priority: %s\n",
                strerror(errno) );
        abort();
    }
}

enum rosbench_transport {
    ROS_TCP,
    ROS_UDP,
};

int publish(int argc, char **argv, int freq);
int subscribe(int argc, char **argv, int num, int emit, enum rosbench_transport t, uint64_t discard, mqd_t mq);

#ifdef __cplusplus
}
#endif

#endif // ROSBENCH_H

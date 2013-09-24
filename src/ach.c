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

#ifdef HAVE_CONFIG
#include "config.h"
#endif //HAVE_CONFIG

#include <ipcbench.h>
#include <ach.h>

static ach_channel_t channel;

#define CHAN_NAME "ipcbench"

static void s_init(void) {
    /* create channel */
    int r = ach_unlink(CHAN_NAME);               /* delete first */
    assert( ACH_OK == r || ACH_ENOENT == r);
    r = ach_create(CHAN_NAME, 32,
                   sizeof(struct timespec), NULL );
    assert(ACH_OK == r);

}

static void s_destroy(void) {
    ach_unlink(CHAN_NAME);
}

static void s_init_send_recv(void) {
    int r = ach_open(&channel, CHAN_NAME, NULL );
    if( ACH_OK != r ) abort();
}

static void s_send( const struct timespec *ts ) {
    int r = ach_put( &channel, ts, sizeof(*ts) );
    printf("sending: %lu, %lu\n", ts->tv_sec, ts->tv_nsec);
    if( ACH_OK != r ) abort();
}



static void s_recv( struct timespec *ts ) {
    size_t fs;
    ach_status_t r = ach_get( &channel, ts, sizeof(*ts),
                              &fs, NULL, ACH_O_WAIT | ACH_O_LAST );
    if( ! (ACH_OK==r || ACH_MISSED_FRAME == r) ) {
        fprintf(stderr, "error: %s\n", ach_result_to_string(r) );
        abort();
    }
    if( sizeof(*ts) != fs ) {
        abort();
    }
}


static void s_destroy_send_recv(void) {
    ach_close(&channel);
}


struct ipcbench_vtab ipc_bench_vtab_ach = {
    .init = s_init,
    .init_send = s_init_send_recv,
    .init_recv = s_init_send_recv,
    .send = s_send,
    .recv = s_recv,
    .destroy_send = s_destroy_send_recv,
    .destroy_recv = s_destroy_send_recv,
    .destroy = s_destroy,
};



/*
void send(void)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        abort();

    ipcbench_lcm_timestamp_t msg;
    msg.secs = 0;
    msg.nsecs = 0;

    while(1) {
        struct timespec ts;
        clock_gettime( CLOCK_MONOTONIC, &ts );
        msg.secs = ts.tv_sec;
        msg.nsecs = ts.tv_nsec;
        ipcbench_lcm_timestamp_t_publish(lcm, "EXAMPLE", &msg);
        sleep(1);
    }

    lcm_destroy(lcm);
}


void listen(void)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        abort();


    while(1)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return;
}

*/

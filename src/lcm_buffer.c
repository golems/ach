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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <poll.h>
#include "ipcbench_lcm_timestamp_t.h"

static void
listen_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const ipcbench_lcm_timestamp_t * msg, void * user)
{
    (void)rbuf;
    (void)user;
    (void)channel;
    printf("receive %lu\n", msg->secs );
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    /* Create */
    lcm_t *lcm_snd = lcm_create(NULL);
    lcm_t *lcm_rcv = lcm_create(NULL);

    /* Subscribe */
    ipcbench_lcm_timestamp_t_subscription_t *lsub =
        ipcbench_lcm_timestamp_t_subscribe(lcm_rcv, "overflow", &listen_handler, NULL);
    ipcbench_lcm_timestamp_t_subscription_set_queue_capacity( lsub, 1);

    /* Overflow socket buffer */
    ipcbench_lcm_timestamp_t msg = {.secs=0, .nsecs=0};

    while( msg.secs++ < 1000 ) {
        ipcbench_lcm_timestamp_t_publish(lcm_snd, "overflow", &msg);
    }
    printf("sent %lu messages\n", msg.secs - 1 );

    /* See what we get */
    {
        int fd = lcm_get_fileno(lcm_rcv);

        struct pollfd pfd[] = {{.fd = fd, .events = POLLIN} };

        int k;
        while( (k = poll( pfd, 1, 0)) ) {
            if( k < 0 ) {
                perror("poll");
                abort();
            }
            lcm_handle(lcm_rcv);
        }
    }

    lcm_destroy(lcm_rcv);
    lcm_destroy(lcm_snd);
}

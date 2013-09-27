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
#include <sys/fcntl.h>
#include <mqueue.h>

static mqd_t fd;

static void s_open(int flag) {
    struct mq_attr attr = {.mq_maxmsg = 10,
                           .mq_msgsize = sizeof(struct timespec),
                           .mq_flags = 0};
    if( (fd = mq_open("/ipcbench", O_CREAT|flag, 0666, &attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }
}

static void s_init_send(void) {
    s_open( O_WRONLY );
}

static void s_init_recv(void) {
    s_open( O_RDONLY );
}

static void s_destroy_send_recv(void) {
    if( close(fd) ) {
        perror( "error closing mq" );
    }
}

static void s_send( const struct timespec *ts ) {
    if( mq_send(fd, (char*)ts, sizeof(*ts), 0) ) {
        perror( "could not send data mq" );
        abort();
    }
}

static void s_recv( struct timespec *ts ) {
    ssize_t r = mq_receive( fd, (char*)ts, sizeof(*ts), NULL );
    if( 0 > r ) {
        perror( "could not receive data mq" );
        abort();
    }
}

static void s_destroy( ) {
    mq_unlink("/ipcbench");
}

struct ipcbench_vtab ipc_bench_vtab_mq = {
    .init = s_destroy,
    .init_send = s_init_send,
    .init_recv = s_init_recv,
    .send = s_send,
    .recv = s_recv,
    .destroy_send = s_destroy_send_recv,
    .destroy_recv = s_destroy_send_recv,
    .destroy = s_destroy
};

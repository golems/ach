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

static int fd[2];


static void s_init(void) {
    if( pipe(fd) ) {
        perror( "could not create pipe" );
        abort();
    }
}

static void s_destroy(void) {
    if( close(fd[0]) || close(fd[1]) ) {
        perror( "error closing pipe" );
    }
}

static void s_send( const struct timespec *ts ) {
    ssize_t r = write(fd[1], ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not send data on pipe" );
        abort();
    }
}

static void s_recv( struct timespec *ts ) {
    ssize_t r = read(fd[0], ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not receive data on pipe" );
        abort();
    }
}

struct ipcbench_vtab ipc_bench_vtab_pipe = {
    .init = s_init,
    .send = s_send,
    .recv = s_recv,
    .destroy = s_destroy,
};

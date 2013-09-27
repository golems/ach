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
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define UNIX_PATH_MAX    108

static int sock;
static int csock;
static int fd;
static struct sockaddr_un addr = {0};
static struct sockaddr_un caddr = {0};
unsigned clen;

static void s_init(void) {
    unlink("/tmp/ipcbench.sock");
}

static void s_init_send(void) {
    sock = socket( PF_UNIX, SOCK_STREAM, 0 );
    if( sock < 0 ) {
        perror( "Could not create socket");
        abort();
    }

    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, UNIX_PATH_MAX, "/tmp/ipcbench.sock");


    if (connect(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("failed to connect");
        abort();
    }

    fd = sock;
}

static void s_init_recv(void) {
    sock = socket( PF_UNIX, SOCK_STREAM, 0 );
    if( sock < 0 ) {
        perror( "Could not create socket");
        abort();
    }

    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, UNIX_PATH_MAX, "/tmp/ipcbench.sock");


    if (bind(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("Failed to bind the server socket");
        abort();
    }

    if (listen(sock, 1) < 0) {
        perror("Failed to listen on server socket");
        abort();
    }

    if ((csock = accept(sock, (struct sockaddr *) &caddr, &clen)) < 0) {
        perror(" failed to accept connection");
        abort();
    }

    fd = csock;
}

static void s_send( const struct timespec *ts ) {
    ssize_t r = write(fd, ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not send data on pipe" );
        abort();
    }
}

static void s_recv( struct timespec *ts ) {
    ssize_t r = read(fd, ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not receive data on pipe" );
        abort();
    }
}

struct ipcbench_vtab ipc_bench_vtab_local = {
    .init = s_init,
    .init_send = s_init_send,
    .init_recv = s_init_recv,
    .send = s_send,
    .recv = s_recv
};

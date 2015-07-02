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

#define PORT 8070

static int *sock;
static int *csock;
static int *fd;
static struct sockaddr_in *addr;
static struct sockaddr_in *caddr;
unsigned clen;


static void s_init(void) {
    sock = (int*)calloc(ipcbench_cnt, sizeof(int));
    csock = (int*)calloc(ipcbench_cnt, sizeof(int));
    addr = (struct sockaddr_in*)calloc(ipcbench_cnt, sizeof(struct sockaddr_in));
    caddr = (struct sockaddr_in*)calloc(ipcbench_cnt, sizeof(struct sockaddr_in));
}

static void s_sock(void) {
    for( size_t i = 0; i < ipcbench_cnt; i ++ ) {
        sock[i] = socket( PF_INET, SOCK_STREAM, IPPROTO_TCP );
        if( sock[i] < 0 ) {
            perror( "Could not create socket");
            abort();
        }

        addr[i].sin_family = AF_INET;
        addr[i].sin_addr.s_addr = inet_addr("127.0.0.1");
        addr[i].sin_port = htons(PORT+i);
    }
}

static void s_init_send(void) {
    s_sock();

    for( size_t i = 0; i < ipcbench_cnt; i ++ ) {
        if (connect(sock[i], (struct sockaddr *) &addr[i], sizeof(addr[i])) < 0) {
            perror("failed to connect");
            abort();
        }
    }

    fd = sock;
}

static void s_init_recv(void) {
    s_sock();

    for( size_t i = 0; i < ipcbench_cnt; i ++ ) {

        u_int yes = 1;
        if( setsockopt(sock[i], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) ) {
            perror("setsockopt for receiver failed\n");
        }

        if (bind(sock[i], (struct sockaddr *) &addr[i], sizeof(addr[i])) < 0) {
            perror("Failed to bind the server socket");
            abort();
        }

        if (listen(sock[i], 1) < 0) {
            perror("Failed to listen on server socket");
            abort();
        }
    }

    for( size_t i = 0; i < ipcbench_cnt; i ++ ) {
        if ((csock[i] = accept(sock[i], (struct sockaddr *) &caddr[i], &clen)) < 0) {
            perror(" failed to accept connection");
            abort();
        }
        ipcbench_pfd[i].fd = csock[i];
    }

    fd = csock;
}

static void s_send( const struct timespec *ts ) {
    size_t i = pubnext();
    ssize_t r = write(fd[i], ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not send data on pipe" );
        abort();
    }
}

static void s_recv( struct timespec *ts ) {
    size_t i = pollin();
    ssize_t r = read(fd[i], ts, sizeof(*ts));
    if( sizeof(*ts) != r ) {
        perror( "could not receive data on pipe" );
        abort();
    }
}

struct ipcbench_vtab ipc_bench_vtab_tcp = {
    .init = s_init,
    .init_send = s_init_send,
    .init_recv = s_init_recv,
    .send = s_send,
    .recv = s_recv
};

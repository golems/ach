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

#define PORT 8070



int main(int argc, char **argv) {
    (void)argc; (void)argv;
    /* Create Sockets */
    int snd_sock = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );
    if( snd_sock < 0 ) {
        perror( "Could not create socket");
        abort();
    }

    int rcv_sock = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );
    if( rcv_sock < 0 ) {
        perror( "Could not create socket");
        abort();
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(PORT);

    if (bind(rcv_sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("Failed to bind the socket");
        abort();
    }

    /* { */
    /*     size_t size = sizeof(int) * 1000; */
    /*     if (setsockopt (rcv_sock, SOL_SOCKET, SO_RCVBUF, */
    /*                     (char *) &size, sizeof (size) ) ) */
    /*     { */
    /*         perror ("setsockopt(SOL_SOCKET, SO_RCVBUF)"); */
    /*         abort(); */
    /*     } */
    /* } */

    /* Overflow socket buffer */
    int i = 0;

    while( i++ < 1000) {
        ssize_t r = sendto( snd_sock, &i, sizeof(i), 0,
                            (struct sockaddr *) &addr, sizeof(addr) );
        if( sizeof(i) != r ) {
            perror( "could not send data" );
            abort();
        }
    }
    printf("sent %d messages\n", i - 1 );

    /* See what we get */
    {
        struct pollfd pfd[] = {{.fd = rcv_sock, .events = POLLIN} };

        int k;
        while( (k = poll( pfd, 1, 0)) ) {
            if( k < 0 ) {
                perror("poll");
                abort();
            }
            ssize_t r = recvfrom( rcv_sock, &i, sizeof(i), 0,
                                  NULL, 0 );
            if( sizeof(i) != r ) {
                perror( "could not receive data on pipe" );
                abort();
            }
            printf("recv: %d\n", i);
        }
    }
}

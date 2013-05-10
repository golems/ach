/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
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

#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <ctype.h>
#include <signal.h>
#include <regex.h>
#include <assert.h>
#include <stdarg.h>
#include <errno.h>
#include <syslog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "ach.h"
#include "achutil.h"
#include "achd.h"

/*********
* Client *
*********/

static int socket_connect(void);
static int server_connect( struct achd_conn*);

static void sleep_till( const struct timespec *t0, int32_t ns );

void achd_client() {
    /* open log */
    openlog("achd-client", LOG_PID, LOG_DAEMON);

    /* First, do some checks to make sure we can process the request */
    struct achd_conn conn;
    memset(&conn, 0, sizeof(conn));

    /* Create request headers */
    conn.vtab = achd_get_vtab( cx.cl_opts.transport, cx.cl_opts.direction );
    assert( conn.vtab && conn.vtab->handler );

    if( cx.cl_opts.remote_chan_name ) {
        conn.send_hdr.chan_name = cx.cl_opts.remote_chan_name;
    } else {
        cx.error( ACH_BAD_HEADER, "No channel name given\n");
        assert(0);
    }

    assert( cx.cl_opts.transport );
    assert( cx.cl_opts.direction == ACHD_DIRECTION_PUSH ||
            cx.cl_opts.direction == ACHD_DIRECTION_PULL );
    conn.send_hdr.transport = cx.cl_opts.transport;

    sighandler_install();

    /* maybe detach */
    if( cx.detach ) {
        pid_t gp = ach_detach( ACH_PARENT_TIMEOUT_SEC );
        /* close stdout/stderr/stderr */
        if( close(STDOUT_FILENO) ) {
            ACH_LOG( LOG_ERR, "Couldn't close stdout: %s", strerror(errno) );
        }
        if( close(STDERR_FILENO) ) {
            ACH_LOG( LOG_ERR, "Couldn't close stderr: %s", strerror(errno) );
        }
        /* let GP know we're ok */
        if( kill( gp, SIGUSR1 ) ) {
            ACH_LOG( LOG_ERR, "Couldn't signal grandparent with status: %s\n", strerror(errno) );
        }
    }

    /* Start the show */

    /* Open initial connection */
    int fd = server_connect( &conn );
    if( fd < 0 ) fd = achd_reconnect( &conn );

    /* Allocate buffers */
    conn.pipeframe_size =
        conn.channel.shm->data_size / conn.channel.shm->index_cnt;
    conn.pipeframe = ach_pipe_alloc( conn.pipeframe_size );

    /* TODO: If we lose and then re-establish a connections, frames
     * may be missed or duplicated.
     *
     * This can be more easily fixed with a pushing client, since it
     * can maintain channel context and try to reconnect.  We just
     * need to buffer the last read frame till we reconnect, or re-get
     * it with ACH_O_COPY.  Other frames can remain buffered in the
     * channel.  If the client dies, though, we lose context.
     *
     * Fixing this for pulling requires the server process to persist
     * after lost connection.  That seems harder to implement.
     */

    /* Start running */
    if ( fd >= 0 && !cx.sig_received ) {
        ACH_LOG(LOG_INFO, "Client running\n");
        conn.vtab->handler( &conn );
        ACH_LOG(LOG_INFO, "Client done\n");
    }
}

static int server_connect( struct achd_conn *conn) {
    ACH_LOG( LOG_NOTICE, "Connecting to %s:%d\n", cx.cl_opts.remote_host, cx.port );
    conn->in = conn->out = -1;

    /* Note the time */
    clock_gettime( CLOCK_MONOTONIC, &conn->t0 );

    /* Connect to server */
    int fd = socket_connect();
    if (fd < 0) {
        return fd;
    } else {
        ACH_LOG( LOG_DEBUG, "Socket connected\n");
    }

    /* Write request */
    {
        conn->in = conn->out = fd;
        if( conn->vtab->connect ) conn->vtab->connect(conn);
        conn->in = conn->out = -1;
        enum ach_status r =
            achd_printf(fd,
                        "channel-name: %s\n"
                        "transport: %s\n"
                        "direction: %s\n"
                        ".\n",
                        conn->send_hdr.chan_name,
                        conn->send_hdr.transport,
                        ( (cx.cl_opts.direction == ACHD_DIRECTION_PULL) ?
                          "push" : "pull" )
                        /* remote end does the opposite */ );
        if( ACH_OK != r ) {
            ACH_LOG(LOG_DEBUG, "couldn't send headers\n");
            close(fd);
            return -1;
        } else {
            ACH_LOG(LOG_DEBUG, "headers sent\n");
        }
    }

    /* Get Response */
    conn->recv_hdr.status = ACH_BUG;
    {
        enum ach_status r = achd_parse_headers( fd, &conn->recv_hdr );
        if( ACH_OK != r ) {
            if( errno ) {
                cx.error( r, "Bad response from server: %s\n", strerror(errno) );
            } else {
                cx.error( r, "Bad response from server\n");
            }
            assert(0);
        } else if ( ACH_OK != conn->recv_hdr.status ) {
            if( conn->recv_hdr.message ) {
                cx.error( conn->recv_hdr.status, "Server error: %s\n", conn->recv_hdr.message );
                assert(0);
            } else {
                cx.error( conn->recv_hdr.status, "Bad response from server\n" );
                assert(0);
            }
        }
    }
    ACH_LOG(LOG_DEBUG, "Server response received\n");

    /* Try to create channel if needed */
    if( ! conn->channel.shm ) {
        ach_status_t r = ach_open(&conn->channel, cx.cl_opts.chan_name, NULL);
        if( ACH_ENOENT == r) {
            int frame_size = conn->recv_hdr.frame_size ? conn->recv_hdr.frame_size : ACH_DEFAULT_FRAME_SIZE;
            int frame_count = conn->recv_hdr.frame_count ? conn->recv_hdr.frame_count : ACH_DEFAULT_FRAME_COUNT;
            /* Fixme: should sanity check these counts */
            r = ach_create( cx.cl_opts.chan_name, (size_t)frame_count, (size_t)frame_size, NULL );
            if( ACH_OK != r )  cx.error( r, "Couldn't create channel\n");
            r = ach_open( &conn->channel, cx.cl_opts.chan_name, NULL );
            if( ACH_OK != r )  cx.error( r, "Couldn't open channel\n");
        } else if (ACH_OK != r ) {
            cx.error( r, "Couldn't open channel\n");
        }
        r = ach_flush(&conn->channel );
        if( ACH_OK != r )  cx.error( r, "Couldn't flush channel\n");
    }

    return conn->in = conn->out = fd;
}


int achd_reconnect( struct achd_conn *conn) {
    int fd = -1;
    while( cx.reconnect && fd < 0 && !cx.sig_received ) {
        ACH_LOG(LOG_DEBUG, "Reconnect attempt\n");
        sleep_till( &conn->t0, ACHD_RECONNECT_NS );
        fd = server_connect( conn );
    }

    return fd;
}

static int socket_connect() {
    /* Make socket */
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ACH_LOG(LOG_ERR, "Couldn't create socket: %s\n", strerror(errno));
        return sockfd;
    }

    /* Lookup Host */
    assert( cx.cl_opts.remote_host );
    struct hostent *server = gethostbyname( cx.cl_opts.remote_host );
    if( NULL == server ) {
        ACH_LOG(LOG_ERR, "Host '%s' not found\n", cx.cl_opts.remote_host);
        close(sockfd);
        return -1;
    }

    /* Make socket address */
    struct sockaddr_in serv_addr;
    memset( &serv_addr, 0, sizeof(serv_addr) );
    serv_addr.sin_family = AF_INET;
    memcpy( &serv_addr.sin_addr.s_addr,
            server->h_addr,
            (size_t)server->h_length);
    serv_addr.sin_port = htons(cx.port); /* Well, ipv4 only for now */

    /* Connect */
    if( connect(sockfd,  (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0 ) {
        ACH_LOG(LOG_ERR, "Couldn't connect: %s\n", strerror(errno));
        close(sockfd);
        return -1;
    }

    return sockfd;
}


static void sleep_till( const struct timespec *t0, int32_t ns ) {
    int64_t ns1 = t0->tv_nsec + ns;
    struct timespec t = {.tv_sec = t0->tv_sec + ns1 / 1000000000,
                         .tv_nsec = ns1 % 1000000000 };

    while(!cx.sig_received) {
        int r = clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME,
                             &t, NULL );
        if (r && !cx.sig_received && EINTR == errno ) {
            /* signalled */
            continue;
        } else if (!r) {
            /* time elapsed */
            break;
        } else {
            /* weird error */
            ACH_LOG( LOG_ERR, "clock_nanosleep failed: %s\n", strerror(errno) );
            break;
        }
        assert(0);
    }
}

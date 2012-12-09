/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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
#include <arpa/inet.h>
#include <netdb.h>

#include "ach.h"
#include "achutil.h"
#include "achd.h"

/* TODO: add SCTP, RDS Support */

struct udp_cx {
    int sock;
};

static void get_frame( struct achd_conn *conn );
static void put_frame( struct achd_conn *conn );



static void get_frame( struct achd_conn *conn ) {
    do {
        size_t frame_size = 0;
        ach_status_t r = ACH_BUG;
        if( 0 ) {
            /* parse command */
            /* if ( 0 == memcmp("next", cmd, 4) ) { */
            /*     r = ach_get(&chan, frame->data, max, &frame_size,  NULL, */
            /*                 ACH_O_WAIT ); */
            /* }else if ( 0 == memcmp("last", cmd, 4) ){ */
            /*     r = ach_get(&chan, frame->data, max, &frame_size,  NULL, */
            /*                 ACH_O_WAIT | ACH_O_LAST ); */
            /* } else if ( 0 == memcmp("poll", cmd, 4) ) { */
            /*     r = ach_get( &chan, frame->data, max, &frame_size, NULL, */
            /*                  ACH_O_COPY | ACH_O_LAST ); */
            /* } else { */
            /*     hard_assert(0, "Invalid command: %s\n", cmd ); */
            /* } */
        } else {
            /* push the data */
            /* TODO: getlast header is not right */
            r = ach_get( &conn->channel, conn->pipeframe->data, conn->pipeframe_size, &frame_size,  NULL,
                         ( (conn->recv_hdr.get_last /*|| is_freq*/) ?
                           (ACH_O_WAIT | ACH_O_LAST ) : ACH_O_WAIT) );
        }
        /* check return code */
        if( ACH_OVERFLOW == r ) {
            achd_log( LOG_NOTICE, "buffer too small, resizing to %" PRIuPTR "\n", frame_size);
            /* enlarge buffer and retry on overflow */
            assert(frame_size > conn->pipeframe_size );
            conn->pipeframe_size = frame_size;
            free(conn->pipeframe);
            conn->pipeframe = ach_pipe_alloc( conn->pipeframe_size );
        } else if (ACH_OK == r || ACH_MISSED_FRAME == r ) {
            ach_pipe_set_size( conn->pipeframe, frame_size );
            break;
        }else {
            /* abort on other errors */
            /* hard_assert( 0, "sub: ach_error: %s\n", */
            /*              ach_result_to_string(r) ); */
            assert(0);
        }
    }while( !cx.sig_received );
}

static void put_frame( struct achd_conn *conn ) {
    size_t cnt =  ach_pipe_get_size(conn->pipeframe);
    if( !cx.sig_received ) {
        ach_status_t r = ach_put( &conn->channel, conn->pipeframe->data, cnt );
        if( ACH_OK != r ) {
            cx.error( r, "Couldn't put frame, size %d\n", cnt );
        }
    }
}

int achd_connect_nop( struct achd_conn *conn ) {
    (void)conn;
    return 0;
}

int achd_udp_sender( struct achd_conn *conn ) {
    struct udp_cx *ucx;
    if( conn->cx ) {
        ucx = (struct udp_cx*)conn->cx;
    } else {
        conn->cx = ucx = (struct udp_cx*)calloc(1, sizeof(struct udp_cx));
    }

    ucx->sock = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );
    if( ucx->sock < 0 ) {
        cx.error(ACH_FAILED_SYSCALL, "Couldn't create UDP socket: %s\n", strerror(errno) );
    }

    return 0;
}


int achd_udp_receiver( struct achd_conn *conn ) {

    /* Create socket */
    achd_udp_sender(conn);
    struct udp_cx *ucx = (struct udp_cx*)conn->cx;
    assert(ucx);

    /* Bind */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = 0;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if( bind( ucx->sock, (struct sockaddr*)&addr, sizeof(addr) ) ) {
        cx.error( ACH_FAILED_SYSCALL, "Could not bind udp socket: %s\n", strerror(errno) );
    }

    /* Tell peer the port */
    struct sockaddr_in real_addr;
    memset(&real_addr, 0, sizeof(real_addr));
    socklen_t real_len = sizeof(real_addr);
    if( getsockname( ucx->sock, (struct sockaddr*)&real_addr, &real_len ) ) {
        cx.error( ACH_FAILED_SYSCALL, "Could find socket port: %s\n", strerror(errno) );
    } else {
        if( ACH_OK != achd_printf( conn->out, "remote-port: %d\n", ntohs(real_addr.sin_port) ) ) {
            cx.error(ACH_FAILED_SYSCALL, "Couldn't write remote-port: %s\n", strerror(errno) );
        }
    }


    return 0;
}


void achd_push_tcp( struct achd_conn *conn ) {
    /* Subscribe and Write */

    /* struct timespec period = {0,0}; */
    /* int is_freq = 0; */
    /* if(opt_freq > 0) { */
    /*     double p = 1.0 / opt_freq; */
    /*     period.tv_sec = (time_t)p; */
    /*     period.tv_nsec = (long) ((p - (double)period.tv_sec)*1e9); */
    /*     is_freq = 1; */
    /* } */

    /* read loop */
    while( !cx.sig_received ) {
        /* char cmd[4] = {0}; */
        /* if( opt_sync ) { */
        /*     /\* wait for the pull command *\/ */
        /*     size_t rc = fread( cmd, 1, 4, fin); */
        /*     hard_assert(4 == rc, "Invalid command read: %d\n", rc ); */
        /*     verbprintf(2, "Command %s\n", cmd ); */
        /* } */

        /* read the data */
        get_frame(conn);

        if( cx.sig_received ) break;

        /* stream send */
        int sent_frame = 0;
        do {
            size_t size = sizeof(ach_pipe_frame_t) - 1 + ach_pipe_get_size(conn->pipeframe);
            achd_log( LOG_DEBUG, "Writing frame, %" PRIuPTR " bytes total\n", size);
            ssize_t r = achd_write( conn->out, conn->pipeframe, size );
            if( r < 0 || (size_t)r != size ) {
                achd_log( LOG_ERR, "Couldn't write frame\n");
                if( cx.reconnect ) achd_reconnect(conn);
                else return;
            } else sent_frame = 1;
        } while( !sent_frame && !cx.sig_received && cx.reconnect );
        /* TODO: ACH_O_LAST handling */

        /* if( opt_sync ) { */
            /*     fsync( fileno(fout) ); /\* fails w/ sbcl, and maybe that's ok *\/ */
            /* } */
            /* verbprintf( 2, "Printed output\n"); */
        /* t0 = 0; */
        /* /\* maybe sleep *\/ */
        /* if( is_freq ) { */
        /*     assert( !opt_sync ); */
        /*     _relsleep(period); */
        /* } */
    }
}

void achd_pull_tcp( struct achd_conn *conn ) {
    /* Read and Publish Loop */
    while( !cx.sig_received ) {
        int got_frame = 0;
        uint64_t cnt = 0;
        do {
            /* get size */
            ssize_t s = achd_read(conn->in, conn->pipeframe, 16 );
            if( s <= 0 ) {
                achd_log(LOG_DEBUG, "Empty read: %s (%d)\n", strerror(errno), errno);
                achd_reconnect(conn);
            } else if( 16 != (ssize_t)s ) {
                achd_log(LOG_ERR, "Incomplete frame header\n");
                achd_reconnect(conn);
            } else if( memcmp("achpipe", conn->pipeframe->magic, 8) ) {
                achd_log(LOG_ERR, "Invalid frame header\n");
                achd_reconnect(conn);
            } else {
                cnt = ach_pipe_get_size( conn->pipeframe );
                /* TODO: sanity check that cnt is not something outrageous */
                /* make sure buf can hold it */
                if( (size_t)cnt > conn->pipeframe_size ) {
                    conn->pipeframe_size = cnt;
                    free( conn->pipeframe );
                    conn->pipeframe = ach_pipe_alloc( conn->pipeframe_size );
                    ach_pipe_set_size( conn->pipeframe, cnt );
                }
                /* get data */
                s = achd_read( conn->in, conn->pipeframe->data, (size_t)cnt );
                if( (ssize_t)cnt != s ) {
                    achd_log(LOG_ERR, "Incomplete frame data\n");
                    achd_reconnect(conn);
                } else {
                    got_frame = 1;
                }
            }
        } while( !got_frame && !cx.sig_received && cx.reconnect );
        if( !got_frame ) return;
        /* put data */
        put_frame(conn);
    }
}


/* TODO: Tell remote end when we're done
 *
 * Could use some type of keepalives for UDP connections. Maybe
 * periodically send 0-length messages over the socket to tell remote
 * end we're still alive
 */

void achd_push_udp( struct achd_conn *conn ) {
    struct udp_cx *ucx = (struct udp_cx*)conn->cx;
    assert(ucx);

    /* Find remote address */
    struct sockaddr_in addr_udp;
    memset( &addr_udp, 0, sizeof(addr_udp) );
    {
        struct sockaddr_in addr_tcp;
        socklen_t len = sizeof(addr_tcp);
        if( getpeername( conn->in, (struct sockaddr *) &addr_tcp, &len ) ) {
            cx.error( ACH_FAILED_SYSCALL, "Couldn't determine name of peer: %s\n", strerror(errno));
        } else {
            addr_udp.sin_addr = addr_tcp.sin_addr;
            addr_udp.sin_port = htons((in_port_t)conn->recv_hdr.remote_port);
        }
    }

    achd_log( LOG_INFO, "sending UDP to %s:%d\n",
              inet_ntoa(addr_udp.sin_addr), ntohs(addr_udp.sin_port) );

    while( !cx.sig_received ) {
        /* read the data */
        get_frame(conn);

        /* Send UDP packet */
        /* TODO: does O_NONBLOCK make sense? */
        /* TODO: handle exceeding of UDP MTU */
        /* TODO: warn on exceeding of ethernet MTU */
        size_t cnt = ach_pipe_get_size( conn->pipeframe );
        ssize_t r = -1;
        achd_log( LOG_DEBUG, "Sending %"PRIuPTR" UDP bytes\n", cnt );
        do {
            r = sendto( ucx->sock, conn->pipeframe->data, cnt, 0,
                        (struct sockaddr*) &addr_udp, sizeof(addr_udp) );
        } while( r < 0 && !cx.sig_received && EINTR == errno );

        if( r < 0 || (size_t)r != cnt ) {
            cx.error( ACH_FAILED_SYSCALL, "Couldn't send UDP message to %s:%d, %s (%d)\n",
                      inet_ntoa(addr_udp.sin_addr), ntohs(addr_udp.sin_port), strerror(errno), errno );
        }
    }
}

void achd_pull_udp( struct achd_conn *conn ) {
    struct udp_cx *ucx = (struct udp_cx*)conn->cx;
    assert(ucx);

    /* Find peer address */
    /* Only should accept data from the peer */
    struct sockaddr_in addr_tcp;
    {
        socklen_t len = sizeof(addr_tcp);
        if( getpeername( conn->in, (struct sockaddr *) &addr_tcp, &len ) ) {
            cx.error( ACH_FAILED_SYSCALL, "Couldn't determine name of peer: %s\n", strerror(errno));
        }

        struct sockaddr_in addr_udp;
        len = sizeof(addr_udp);
        if( getsockname( ucx->sock, (struct sockaddr *) &addr_udp, &len ) ) {
            cx.error( ACH_FAILED_SYSCALL, "Couldn't determine name of UDP receiver: %s\n", strerror(errno));
        } else {
            achd_log( LOG_INFO, "receiving UDP from %s on local port %d\n",
                      inet_ntoa(addr_tcp.sin_addr), ntohs( addr_udp.sin_port ) );
        }
    }

    /* Get the packets */
    while( !cx.sig_received ) {
        /* Read packet */
        struct sockaddr_in addr_udp;
        memset( &addr_udp, 0, sizeof(addr_udp) );
        socklen_t len = sizeof(addr_udp);
        ssize_t r = -1;
        do {
            /* TODO: handle too small buffers */
            r = recvfrom( ucx->sock, conn->pipeframe->data, conn->pipeframe_size,
                          0, (struct sockaddr*) &addr_udp, &len );
        } while( r < 0 && EINTR == errno && !cx.sig_received );

        if( cx.sig_received ) break;

        achd_log( LOG_DEBUG, "Received %" PRIdPTR " UDP bytes from %s\n",
                  r, inet_ntoa(addr_udp.sin_addr) );

        /* Put the frame */
        ach_pipe_set_size( conn->pipeframe, (size_t)r );
        put_frame(conn);
    }

}

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
#include <netdb.h>

#include "ach.h"
#include "achutil.h"
#include "achd.h"

void achd_push_tcp( struct achd_conn *conn ) {
    size_t pipeframe_size =
        conn->channel.shm->data_size / conn->channel.shm->index_cnt;
    ach_pipe_frame_t *pipeframe = ach_pipe_alloc( pipeframe_size );

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
        int got_frame = 0;
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
                r = ach_get( &conn->channel, pipeframe->data, pipeframe_size, &frame_size,  NULL,
                             ( (conn->request.get_last /*|| is_freq*/) ?
                               (ACH_O_WAIT | ACH_O_LAST ) : ACH_O_WAIT) );
            }
            /* check return code */
            if( ACH_OVERFLOW == r ) {
                achd_log( LOG_NOTICE, "buffer too small, resizing to %d\n", frame_size);
                /* enlarge buffer and retry on overflow */
                assert(frame_size > pipeframe_size );
                pipeframe_size = frame_size;
                free(pipeframe);
                pipeframe = ach_pipe_alloc( pipeframe_size );
            } else if (ACH_OK == r || ACH_MISSED_FRAME == r ) {
                got_frame = 1;
                ach_pipe_set_size( pipeframe, frame_size );
            }else {
                /* abort on other errors */
                /* hard_assert( 0, "sub: ach_error: %s\n", */
                /*              ach_result_to_string(r) ); */
                assert(0);
            }
        }while( !got_frame && !cx.sig_received );

        if( cx.sig_received ) break;

        /* stream send */
        int sent_frame = 0;
        do {
            size_t size = sizeof(ach_pipe_frame_t) - 1 + ach_pipe_get_size(pipeframe);
            achd_log( LOG_ERR, "Writing frame, %d bytes total\n", size);
            ssize_t r = achd_write( conn->out, pipeframe, size );
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
    size_t pipeframe_size =
        conn->channel.shm->data_size / conn->channel.shm->index_cnt;
    ach_pipe_frame_t *pipeframe = ach_pipe_alloc( pipeframe_size );
    /* Read and Publish Loop */
    while( !cx.sig_received ) {
        int got_frame = 0;
        uint64_t cnt = 0;
        do {
            /* get size */
            ssize_t s = achd_read(conn->in, pipeframe, 16 );
            if( s <= 0 ) {
                achd_log(LOG_DEBUG, "Empty read: %s (%d)\n", strerror(errno), errno);
                achd_reconnect(conn);
            } else if( 16 != (ssize_t)s ) {
                achd_log(LOG_ERR, "Incomplete frame header\n");
                achd_reconnect(conn);
            } else if( memcmp("achpipe", pipeframe->magic, 8) ) {
                achd_log(LOG_ERR, "Invalid frame header\n");
                achd_reconnect(conn);
            } else {
                cnt = ach_pipe_get_size( pipeframe );
                /* TODO: sanity check that cnt is not something outrageous */
                /* make sure buf can hold it */
                if( (size_t)cnt > pipeframe_size ) {
                    pipeframe_size = cnt;
                    free( pipeframe );
                    pipeframe = ach_pipe_alloc( pipeframe_size );
                }
                /* get data */
                s = achd_read( conn->in, pipeframe->data, (size_t)cnt );
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
        if( !cx.sig_received ) {
            ach_status_t r = ach_put( &conn->channel, pipeframe->data, cnt );
            if( ACH_OK != r ) {
                cx.error( r, "Couldn't put frame, size %d\n", cnt );
            }
            assert( r == ACH_OK );
        }
    }
}

void achd_push_udp( struct achd_conn *conn ) {
    /* check port set */
    assert(0); /* unimplemented */
}

void achd_pull_udp( struct achd_conn *conn ) {
    assert(0); /* unimplemented */
}

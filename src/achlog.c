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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <inttypes.h>
#include <syslog.h>
#include "ach.h"
#include "achutil.h"

static struct log_desc {
    const char *name;
    int last;
    double freq;
    ach_channel_t chan;
    FILE *fout;
} *log_desc = NULL;
static size_t n_log = 0;
static double opt_freq = 0;
static int opt_last = 0;

static void *worker( void *arg ) {
    struct log_desc *desc = (struct log_desc*)arg;

    /* write header */
    fprintf( desc->fout,
             "ACHLOG\n"
             "log-channel: %s\n"
             "log-version: 0\n"
             ".\n",
             desc->name
        );

    size_t max = 512;
    ach_pipe_frame_t *frame = ach_pipe_alloc( max );

    /* get frames */
    int canceled = 0;
    while( ! canceled ) {
        /* push the data */
        size_t frame_size;
        ach_status_t r = ach_get( &desc->chan, frame->data, max, &frame_size,  NULL,
                                  ACH_O_WAIT | ((opt_last ) ? ACH_O_LAST : 0) );
        switch(r) {
        case ACH_OVERFLOW:
            /* enlarge buffer and retry on overflow */
            assert(frame_size > max );
            max = frame_size;
            free(frame);
            frame = ach_pipe_alloc( max );
            continue;
        case ACH_MISSED_FRAME:
        case ACH_OK:
        {
            ach_pipe_set_size( frame, frame_size );
            size_t size = sizeof(ach_pipe_frame_t) - 1 + frame_size;
            size_t s = fwrite( frame, 1, size, desc->fout );
            if( s != size ) {
                ACH_LOG( LOG_ERR, "Could not write frame to %s, %"PRIuPTR" written instead of %"PRIuPTR"\n",
                         desc->name, s, size );
                canceled = 1;
            }
        }
        break;
        case ACH_CANCELED:
            canceled = 1;
            break;
        default:
            ACH_LOG( LOG_ERR, "Could not get frame from %s: %s\n",
                     desc->name, strerror(errno) );
            canceled = 1;
            break;
        }
    }

    /* sync */
    if( fflush(desc->fout) ) {
        ACH_LOG( LOG_ERR, "Could not flush file %s: %s\n",
                 desc->name, strerror(errno) );
    }
    return arg;
}



static void help() {
    puts( "Usage: achlog [OPTIONS] channels...\n"
          "Log ach channels to files"
          "\n"
          "Options:\n"
          "  -?,                  Show help\n"
          "\n"
          "Examples:\n"
          "  achlog foo bar       Log channels foo and bar\n"
          "\n"
          "Report bugs to <ntd@gatech.edu>"
        );
    exit(EXIT_SUCCESS);
}

static void posarg( char *arg ) {
    if( 0 == strcmp("--help", arg) ) help();

    log_desc = (struct log_desc*)realloc( log_desc, (1+n_log)*sizeof(log_desc[0]) );
    log_desc[n_log++].name = arg;
}


int main( int argc, char **argv ) {
    int c;
    while( (c = getopt( argc, argv, "lnh?V")) != -1 ) {
        switch(c) {
        case 'v':
            ach_verbosity ++;
            break;
        case 'l':
            opt_last = 1;
            break;
        case 'n':
            opt_last = 0;
            break;
        /* case 'f': */
        /*     opt_freq = atof(optarg); */
        /*     break; */
        case 'V':   /* version     */
            ach_print_version("achpipe.bin");
            exit(EXIT_SUCCESS);
        case '?':
        case 'h':
        case 'H':
            help();
        default:
            posarg(optarg);
        }
    }
    while( optind < argc ) {
        posarg(argv[optind++]);
    }
    if( 0 == n_log ) ACH_DIE("No channels to log\n");

    /* Open Channels */
    size_t i;
    for( i = 0; i < n_log; i ++ ) {
        ach_status_t r = ach_open(&log_desc[i].chan, log_desc[i].name, NULL);
        if( ACH_OK != r ) {
            ACH_DIE( "Could not open channel %s: %s\n",
                     log_desc[i].name, ach_result_to_string(r) );
        }
        /* Open log file */
        log_desc[i].fout = fopen(log_desc[i].name, "w");
        if( NULL == log_desc[i].fout ) {
            ACH_DIE( "Could not open log file for %s: %s\n",
                     log_desc[i].name, strerror(errno) );
        }
    }


    /* Block Signals */
    int sigs[] = {SIGTERM, SIGINT, 0};
    ach_sig_block_dummy( sigs );

    /* Create Workers */
    pthread_t thread[n_log];
    for( i = 0; i < n_log; i ++ ) {
        int r = pthread_create( thread+i, NULL, worker, (void*)(log_desc+i) );
        if( r ) ACH_DIE( "Couldn't start worker thread: %s\n", strerror(r) );
    }

    /* Wait for Signal */
    ach_sig_wait( sigs );

    /* Cancel workers */
    ach_cancel_attr_t cattr;
    ach_cancel_attr_init( &cattr );
    cattr.async_unsafe = 1;
    for( i = 0; i < n_log; i ++ ) {
        ach_cancel( &log_desc[i].chan, &cattr );
    }

    /* Join worker threads */
    for( i = 0; i < n_log; i ++ ) {
        int r = pthread_join( thread[i], NULL );
        if( r ) ACH_DIE( "Couldn't join worker thread: %s\n", strerror(r) );
    }

    return 0;
}

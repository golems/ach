/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
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
#include <unistd.h>
#include <limits.h>
#include <pwd.h>
#include "ach.h"
#include "ach_private_posix.h"
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
static int opt_gzip = 0;

static struct timespec now_ach, now_real;
const char *now_real_str = "\n";

/* SUSv2 guarantees that "Host names are limited to 255 bytes".
 * POSIX.1-2001 guarantees that "Host names (not including the
 * terminating null byte) are limited to HOST_NAME_MAX bytes".  On
 * Linux, HOST_NAME_MAX is defined with the value 64, which has been
 * the limit since Linux 1.0 (earlier kernels imposed a limit of 8
 * bytes). */
static char host[HOST_NAME_MAX+1] = {0};
const struct passwd *passwd;

static FILE *filter( const char *program, const char *channel, const char *suffix );

static void *worker( void *arg ) {
    struct log_desc *desc = (struct log_desc*)arg;

    /* write header */
    fprintf( desc->fout,
             "ACHLOG\n"
             "channel-name: %s\n"
             "log-version: 0\n"
             "log-time-ach: %lu.%09lu\n"
             "log-time-real: %lu.%09lu # %s"
             "local-host: %s\n",
             desc->name,
             now_ach.tv_sec, now_ach.tv_nsec,
             now_real.tv_sec, now_real.tv_nsec, now_real_str,
             host );
    if( passwd ) {
        fprintf( desc->fout,
                 "user: %s # %s\n",
                 passwd->pw_name, passwd->pw_gecos
            );
    }
    fputs( ".\n", desc->fout );

    if( fflush(desc->fout) ) {
        ACH_LOG( LOG_ERR, "Could not flush file %s: %s\n",
                 desc->name, strerror(errno) );
    }

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

static void posarg( char *arg ) {
    log_desc = (struct log_desc*)realloc( log_desc, (1+n_log)*sizeof(log_desc[0]) );
    log_desc[n_log++].name = arg;
}

int main( int argc, char **argv ) {
    /* Check if we're running under achcop */
    if( getenv("ACHCOP") ) {
        ach_pid_notify = getppid();
    }

    int c;
    while( (c = getopt( argc, argv, "zlnh?V")) != -1 ) {
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
        case 'z':
            opt_gzip = 1;
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
            puts( "Usage: achlog [OPTIONS] channels...\n"
                  "Log ach channels to files"
                  "\n"
                  "Options:\n"
                  "  -?,                  Show help\n"
                  "  -z,                  Filter output through gzip\n"
                  "\n"
                  "Examples:\n"
                  "  achlog foo bar       Log channels foo and bar\n"
                  "\n"
                  "Report bugs to <ntd@gatech.edu>"
                );
            exit(EXIT_SUCCESS);
        default:
            posarg(optarg);
        }
    }
    while( optind < argc ) {
        posarg(argv[optind++]);
    }
    if( 0 == n_log ) ACH_DIE("No channels to log\n");

    /* Block Signals */
    /* Have to block these before forking so ctrl-C doesn't kill the
     * gzip */
    int sigs[] = {SIGTERM, SIGINT, 0};
    ach_sig_block_dummy( sigs );

    /* Open Channels */
    size_t i;
    for( i = 0; i < n_log; i ++ ) {
        ach_status_t r = ach_open(&log_desc[i].chan, log_desc[i].name, NULL);
        if( ACH_OK != r ) {
            ACH_DIE( "Could not open channel %s: %s\n",
                     log_desc[i].name, ach_result_to_string(r) );
        }
        /* Open log file */
        if( opt_gzip ) {
            log_desc[i].fout = filter( "gzip -c", log_desc[i].name, ".gz" );
        } else {
            log_desc[i].fout = fopen(log_desc[i].name, "w");
        }
        if( NULL == log_desc[i].fout ) {
            ACH_DIE( "Could not open log file for %s: %s\n",
                     log_desc[i].name, strerror(errno) );
        }
    }

    /* get some data */
    if( clock_gettime(ACH_DEFAULT_CLOCK, &now_ach ) ||
        clock_gettime(CLOCK_REALTIME,    &now_real ) )
    {
        ACH_DIE( "Could not get time: %s\n", strerror(errno) );
    }
    if( gethostname( host, sizeof(host) ) ) {
        ACH_LOG(LOG_ERR, "Could not get host name: %s\n", strerror(errno));
    }
    host[sizeof(host)-1] = '\0';
    passwd = getpwuid(getuid());
    if( passwd ) {
        strtok(passwd->pw_gecos, ",");
    }
    now_real_str = ctime( &now_real.tv_sec );

    /* Create Workers */
    pthread_t thread[n_log];
    for( i = 0; i < n_log; i ++ ) {
        int r = pthread_create( thread+i, NULL, worker, (void*)(log_desc+i) );
        if( r ) ACH_DIE( "Couldn't start worker thread: %s\n", strerror(r) );
    }
    ach_notify(ACH_SIG_OK);

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
        if( opt_gzip ) {
            if( pclose(log_desc[i].fout) < 0 ) {
                ACH_LOG( LOG_ERR, "Could not pclose output for %s: %s\n",
                         log_desc[i].name, strerror(errno) );
            }
        } else {
            fclose(log_desc[i].fout);
        }
    }

    return 0;
}

static FILE *filter( const char *program, const char *channel, const char *suffix ) {
    size_t n = strlen(channel) + strlen(suffix) + 1;
    char buf[n];
    strcpy(buf, channel);
    strcat(buf, suffix);
    FILE *fout = fopen( buf, "w");
    if( NULL == fout ) {
        ACH_DIE( "Could not open log file %s: %s\n",
                 buf, strerror(errno) );
    }
    int oldout = dup(STDOUT_FILENO);
    if( oldout < 0 )  {
        ACH_DIE( "Could not dup stdout: %s\n", strerror(errno) );
    }
    if( dup2(fileno(fout), STDOUT_FILENO) < 0 ) {
        ACH_DIE( "Could not dup output: %s\n", strerror(errno) );
    }
    FILE *f = popen(program, "w");
    if( NULL == f ) {
        ACH_DIE( "Could not popen `%s': %s\n", program, strerror(errno) );
    }
    if( dup2(oldout, STDOUT_FILENO) < 0 ) {
        ACH_DIE( "Could not dup stdout back: %s\n", strerror(errno) );
    }
    if( close(oldout) ) {
        ACH_LOG( LOG_WARNING, "Could not close dup'ed stdout: %s\n", strerror(errno) );
    }
    if( fclose(fout) )  {
        ACH_LOG( LOG_WARNING, "Could not close dup'ed output file: %s\n", strerror(errno) );
    }

    return f;
}

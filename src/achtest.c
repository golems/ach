/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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

#define _XOPEN_SOURCE 500

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/wait.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include "ach.h"

#define CHAN  "ach-test"

/*#define MAX 102400 */

/* A C2D can roughly handle 16 publishers and 16 subscribers with
 * publishers firing every 1/4 millisecond */

int opt_n_sub = 8;
int opt_n_pub = 8;
int opt_pub_sleep_us = 250;
int opt_n_msgs = 1024;
const char *opt_channel_name = CHAN;

static void test(int r, const char *thing) {
    if( r != ACH_OK ) {
        fprintf(stderr, "%s: %s\n",
                thing, ach_result_to_string(r));
        exit(-1);
    }
}


int test_basic() {
    /* unlink */
    int r = ach_unlink(opt_channel_name);
    if( ! (ACH_OK==r || ACH_ENOENT == r) ) {
        fprintf(stderr, "ach_unlink failed\n: %s",
                ach_result_to_string(r));
        return -1;
    }

    /* create */
    r = ach_create(opt_channel_name, 32, 64, NULL );
    test(r, "ach_create");

    ach_channel_t chan;
    int s, p;
    size_t frame_size;

    /* open */
    r = ach_open(&chan, opt_channel_name, NULL);

    /* empty channel means stale */
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_LAST);
    if( ACH_STALE_FRAMES != r ) {
        printf("get stale failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 0);
    if( ACH_STALE_FRAMES != r ) {
        printf("get stale failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }


    /* put */
    p = 42;
    r = ach_put( &chan, &p, sizeof(p) );
    test(r, "ach_put");

    /* get */
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 0);
    test(r, "first ach_get");
    if(frame_size!= sizeof(s) || s != 42 ) exit(-1);

    /* put 2 */
    p = 43;
    r = ach_put( &chan, &p, sizeof(p) );
    test(r, "ach_put");
    p = 44;
    r = ach_put( &chan, &p, sizeof(p) );
    test(r, "ach_put");

    /* get last */
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_LAST);
    if( ACH_MISSED_FRAME != r ) {
        printf("get last failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }
    if(frame_size != sizeof(s) || s != 44 ) exit(-1);

    /* get last stale */
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_LAST);
    if( ACH_STALE_FRAMES != r ) {
        printf("get stale failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }

    /* copy last */
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_LAST | ACH_O_COPY);
    if( ACH_OK != r ) {
        printf("copy_last failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }
    if( 44 != s ) {
        printf("wrong copy last : %d\n", s);
        exit(-1);
    }

    /* get copy */
    /*ach_dump(chan.shm);*/
    /*printf("chan seq_num: %"PRIu64"\n", chan.seq_num);*/
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_COPY);
    if( ACH_OK != r ) {
        printf("copy_last failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }

    /* missed frames */
    size_t i;
    for( i = 0; i < 100; i ++ ) {
        r = ach_put( &chan, &p, sizeof(p) );
        test(r, "ach_put");
    }
    r = ach_get( &chan, &s, sizeof(s), &frame_size, NULL,
                 ACH_O_LAST);
    if( ACH_MISSED_FRAME != r ) {
        printf("get missed failed: %s\n", ach_result_to_string(r));
        exit(-1);
    }

    /* close */

    r = ach_close(&chan);
    test(r, "ach_close");

    /* unlink */
    r = ach_unlink(opt_channel_name);
    test(r, "ach_unlink");

    fprintf(stderr, "basic ok\n");
    return 0;
}


static int publisher( int32_t i ) {
    ach_channel_t chan;
    int r = ach_open( &chan, opt_channel_name, NULL );
    if( r != ACH_OK ) {
        fprintf(stderr, "publisher %d couldn't ach_open: %s\n",
                i, ach_result_to_string(r) );
        return -1;
    }

    int32_t data[2] = {i, 0};
    size_t j;
    for( j = 0; j < (unsigned int)opt_n_msgs; j++,data[1]++ ) {
        r = ach_put( &chan, data, sizeof(data) );
        if( r != ACH_OK ) {
            fprintf(stderr, "publisher %d couldn't ach_put: %s\n",
                    i, ach_result_to_string(r) );
            return -1;
        }
        if( opt_pub_sleep_us <= 0 ) {
            sched_yield();
        } else {
            usleep((unsigned int)opt_pub_sleep_us);
        }
    }
    r = ach_close(&chan);
    if( ACH_OK != r ) {
        fprintf(stderr, "publisher %d couldn't ach_close: %s\n",
                i, ach_result_to_string(r) );
        return -1;
    } else {
        fprintf(stderr, "publisher %d ok\n", i);
        return 0;
    }
}

static int subscriber( int i ) {
    ach_channel_t chan;
    int32_t ctr[opt_n_pub];
    memset(ctr,0,sizeof(ctr));
    int r = ach_open( &chan, opt_channel_name, NULL );
    if( r != ACH_OK ) {
        fprintf(stderr, "subscriber %d couldn't ach_open: %s",
                i, ach_result_to_string(r) );
        return -1;
    }

    int32_t data[2];
    int seen_last = 0;
    int j;
    for( j = 0; j < opt_n_pub*opt_n_msgs; j++ ) {
        struct timespec abstime;
        clock_gettime( CLOCK_REALTIME, &abstime );
        abstime.tv_sec += 1;
        size_t frame_size;
        r = ach_wait_next( &chan, data, sizeof(data), &frame_size,
                           &abstime );
        if( seen_last && ACH_TIMEOUT == r ) {
            break;
        } else if( ACH_OK != r && ACH_MISSED_FRAME != r) {
            fprintf(stderr, "subscriber %d couldn't ach_get: %s\n",
                    i, ach_result_to_string(r) );
            return -1;
        } else if( sizeof(data) != frame_size ) {
            fprintf(stderr, "subscriber %d bad frame size: %"PRIuPTR"\n",
                    i, frame_size);
            return -1;
        } else if( 0 > data[0] || opt_n_pub <= data[0] ) {
            fprintf(stderr, "subscriber %d bad pub id: %d\n",
                    i, data[0]);
            return -1;
        } else if( ctr[ data[0] ] > data[1] ) {
            fprintf(stderr, "subscriber %d, count %d, got [%d, %d]\n",
                    i, ctr[data[0]], data[1], data[0] );
            return -1;
        }  else {
            if( ctr[ data[0] ] != data[1] ) {
                fprintf(stderr, "subscriber %d missed %d frames from %d, (it's probably ok)\n", i, data[1] - ctr[data[0]], data[0]);
            }

            j += data[1] - ctr[ data[0] ] ;
            ctr[ data[0] ] = data[1]+1;
            if( data[1]+1 == opt_n_msgs ) seen_last = 1;
        }
    }
    r = ach_close(&chan);
    if( ACH_OK != r ) {
        fprintf(stderr, "subscriber %d couldn't ach_close: %s",
                i, ach_result_to_string(r) );
        return -1;
    }
    else {
        fprintf(stderr, "subscriber %d ok, last (%d)\n", i, seen_last);
        return 0;
    }
}

int test_multi() {

    int r = ach_unlink(opt_channel_name);
    if( ! (ACH_OK==r || ACH_ENOENT == r) ) {
        fprintf(stderr, "ach_unlink failed\n: %s",
                ach_result_to_string(r));
        return -1;
    }

    r = ach_create(opt_channel_name, 32, 64, NULL );


    pid_t sub_pid[opt_n_sub];
    pid_t pub_pid[opt_n_pub];
    int i;

    /* create subscribers */
    for( i = 0; i < opt_n_sub; i++ ) {
        pid_t p = fork();
        if( p < 0 ) exit(-1);
        else if( 0 == p ) return subscriber(i);
        else sub_pid[i] = p;

    }

    /* create publishers */
    for( i = 0; i < opt_n_pub; i++ ) {
        pid_t p = fork();
        if( p < 0 ) exit(-1);
        else if( 0 == p ) return publisher(i);
        else pub_pid[i] = p;
    }

    /* wait */
    for( i = 0; i < opt_n_sub+opt_n_pub; i++ ) {
        int s;
        pid_t pid = wait(&s);
        (void)pid;
        if( 0 != s ) return -1;
    }
    return 0;
}

int main( int argc, char **argv ){
    int c;
    while( (c = getopt( argc, argv, "p:s:u:n:c:")) != -1 ) {
        switch(c) {
        case 'p':
            opt_n_pub = atoi(optarg);
            break;
        case 's':
            opt_n_sub = atoi(optarg);
            break;
        case 'u':
            opt_pub_sleep_us = atoi(optarg);
            break;
        case 'n':
            opt_n_msgs = atoi(optarg);
            break;
        case 'c':
            opt_channel_name = strdup(optarg);
            break;
        case '?':
        case 'h':
        case 'H':
            puts( "Usage: achtest [OPTION...]\n"
                  "ach stress test\n"
                  "\n"
                  "  -c CHANNEL-NAME,           default `ach-test'\n"
                  "  -n MESAGE-COUNT,           messages to publish\n"
                  "  -p PUBLISHER-COUNT,        number of publishers\n"
                  "  -s SUBSCRIBER-COUNT,       number of subscribers\n"
                  "  -u MICROSECONDS            microseconds between publish\n"
                  "  -?, -H, -h                 Give this help list\n" );
        }
    }


    printf("p: %d\ts: %d\tn: %d\tu: %d\n",
           opt_n_pub, opt_n_sub, opt_n_msgs, opt_pub_sleep_us );

    int r;

    r = test_basic();
    if( 0 != r ) return r;

    r = test_multi();
    if( 0 != r ) return r;


    r = ach_unlink(opt_channel_name);
    if( ! (ACH_OK==r || ACH_ENOENT == r) ) {
        fprintf(stderr, "ach_unlink failed\n: %s",
                ach_result_to_string(r));
        return -1;
    }

    return 0;
}


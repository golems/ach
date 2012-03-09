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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>
#include <sys/wait.h>
#include <assert.h>
#include <stdio.h>
#include <ach.h>
#include <sys/resource.h>
#include <sched.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>

double FREQUENCY = (1000.0);
double SECS = 1;
size_t RECV_RT = 1;
size_t RECV_NRT = 0;
size_t SEND_RT = 1;


double overhead = 0;

#define BENCH_ACH
//#define BENCH_PIPE


typedef struct timespec ticks_t ;
static ticks_t get_ticks(void) {
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t );
    return t;
}
static double ticks_delta(ticks_t t0, ticks_t t1) {
    double d0 = (double)t0.tv_sec + (double)t0.tv_nsec / 1e9;
    double d1 = (double)t1.tv_sec + (double)t1.tv_nsec / 1e9;
    return (d1 - d0 - overhead);
}

void make_realtime(void) {
    if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
        fprintf(stderr, "Couldn't lock pages in memory: %s\n",
                strerror(errno) );
    }
    struct sched_param sp;
    sp.sched_priority = 99; /* max priority on linux */
    if( sched_setscheduler( 0, SCHED_RR, &sp) < 0 ) {
        fprintf(stderr, "Couldn't set scheduling priority: %s\n",
                strerror(errno) );

    }
}

#ifdef BENCH_ACH
ach_channel_t chan;
void sender(void) {
    fprintf(stderr,"sender\n");
    make_realtime();
    size_t i;
    for( i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        int r = ach_put(&chan, &ticks, sizeof(ticks));
        assert(ACH_OK == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver(int rt) {
    fprintf(stderr,"receiver\n");
    if (rt) make_realtime();
    // flush some initial delayed messages
    size_t i;
    for( i = 0; i < 5; i ++ ) {
        ticks_t ticks;
        size_t fs;
        ach_get(&chan, &ticks, sizeof(ticks), &fs, NULL,
                ACH_O_LAST | ACH_O_WAIT);
    }
    // now the good stuff
    while(1) {
        ticks_t ticks;
        size_t fs;
        int r = ach_get(&chan, &ticks, sizeof(ticks), &fs, NULL,
                        ACH_O_LAST | ACH_O_WAIT);
        ticks_t now = get_ticks();
        assert(ACH_OK == r || sizeof(ticks) == fs);
        // only print real-time latencies
        if (rt) {
            double result = ticks_delta(ticks,now);
            printf("%f\n", result*1e6);
        }
    }

}

void setup(void) {
    // create channel
    int r = ach_unlink("bench");               // delete first
    assert( ACH_OK == r || ACH_ENOENT == r);
    r = ach_create("bench", 10, 256, NULL );
    assert(ACH_OK == r);

    // open channel
    r = ach_open(&chan, "bench", NULL);
    assert(ACH_OK == r);
}
void destroy(void) {
    int r = ach_unlink("bench");
    assert(ACH_OK == r);
}
#endif /*BENCH_ACH */

#ifdef BENCH_PIPE
int fd[2];
void sender(void) {
    fprintf(stderr,"sender\n");
    make_realtime();
    size_t i;
    for(i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        ssize_t r = write(fd[1], &ticks, sizeof(ticks));
        assert(sizeof(ticks) == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver(int rt) {
    (void)rt;
    fprintf(stderr,"receiver\n");
    make_realtime();
    /* flush some initial delayed messages */
    size_t i;
    for( i = 0; i < 5; i ++ ) {
        ticks_t ticks;
        ssize_t r = read(fd[0], &ticks, sizeof(ticks));
        assert(sizeof(ticks) == r);
    }
    /* now the good stuff */
    while(1) {
        ticks_t ticks;
        ssize_t r = read(fd[0], &ticks, sizeof(ticks));
        ticks_t now = get_ticks();
        assert(sizeof(ticks) == r);
        double result = ticks_delta(ticks,now);
        printf("%f\n", result*1e6);
    }

}

void setup(void) {
    /* create channel */
    int r = pipe(fd);
    assert( !r );
}
void destroy(void) {
}
#endif /* BENCH_PIPE */



void calibrate(void) {
    make_realtime();
    double a = 0;
    ticks_t r0,r1;
    size_t i;
    for( i = 0; i<1000; i++ ) {
        r0 = get_ticks();
        r1 = get_ticks();
        a += ticks_delta(r0,r1);
    }
    overhead = (a) / 1000;
}

int main(int argc, char **argv) {

    /* parse args */
    int c;
    char *endptr = 0;
    while( (c = getopt( argc, argv, "f:s:p:r:l:")) != -1 ) {
        switch(c) {
        case 'f':
            FREQUENCY = strtod(optarg, &endptr);
            assert(endptr);
            break;
        case 's':
            SECS = strtod(optarg, &endptr);
            assert(endptr);
            break;
        case 'p':
            SEND_RT = (size_t)atoi(optarg);
            assert(SEND_RT);
            break;
        case 'r':
            RECV_RT = (size_t)atoi(optarg);
            assert(RECV_RT);
            break;
        case 'l':
            RECV_NRT = (size_t)atoi(optarg);
            assert(RECV_NRT);
            break;
        default:
            puts("Usage: ach-bench [OPTION....]\n"
                 "Benchmark Ach IPC\n"
                 "\n"
                 "  -f FREQUENCY,       Frequency in Hertz (1000)\n"
                 "  -s SECONDS,         Duration in seconds (1)\n"
                 "  -p COUNT,           Real-Time Publishers (1)\n"
                 "  -r COUNT,           Real-Time Receivers (1)\n"
                 "  -l COUNT,           Non-Real-Time Receivers (0)\n"
                );
            exit(EXIT_SUCCESS);
        }
    }

    fprintf(stderr, "freq: %f\n", FREQUENCY);
    fprintf(stderr, "secs: %f\n", SECS);
    fprintf(stderr, "real-time receivers: %"PRIuPTR"\n", RECV_RT);
    fprintf(stderr, "non-real-time receivers: %"PRIuPTR"\n", RECV_NRT);
    fprintf(stderr, "real-time senders: %"PRIuPTR"\n", SEND_RT);
    size_t i;

    /* warm up */
    for( i = 0; i < 10; i++) get_ticks();

    /* compute overhead */
    calibrate();
    fprintf(stderr,"overhead: %fus\n", overhead*1e6);

    /* setup comms */
    setup();

    /* fork */
    pid_t pid_recv_rt[RECV_RT+RECV_NRT];
    for( i = 0; i < RECV_RT+RECV_NRT; i ++ ) {
        pid_recv_rt[i] = fork();
        assert( pid_recv_rt[i] >= 0 );
        if(0 == pid_recv_rt[i]) {
            receiver(i < RECV_RT);
            exit(0);
        }
    }
    /* send */
    pid_t pid_send_rt[SEND_RT];
    for( i = 0; i < SEND_RT; i ++ ) {
        pid_send_rt[i] = fork();
        assert( pid_send_rt[i] >= 0 );
        if(0 == pid_send_rt[i]) {
            sender();
            exit(0);
        }
    }
    sleep((unsigned)(SECS * 1.5) + 5);
    /* stop */
    for( i = 0; i < RECV_RT; i ++ ) {
        kill(pid_recv_rt[i],SIGTERM);
    }
    for( i = 0; i < SEND_RT; i ++ ) {
        kill(pid_send_rt[i],SIGTERM);
    }
    destroy();
    exit(0);
}

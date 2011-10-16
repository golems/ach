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
#include <argp.h>
#include <errno.h>
#include <sys/mman.h>

double FREQUENCY = (1000.0);
double SECS = 1;


double overhead = 0;

/*#define BENCH_ACH */
#define BENCH_PIPE

static struct argp_option options[] = {
    {
        .name = "freq",
        .key = 'f',
        .arg = "frequency",
        .flags = 0,
        .doc = "frequency of publication"
    },
    {
        .name = "sec",
        .key = 's',
        .arg = "seconds",
        .flags = 0,
        .doc = "seconds to collect for"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }
};
/** argp parsing function */
static int parse_opt( int key, char *arg, struct argp_state *state);
/** argp program version */
const char *argp_program_version = "achbench-" ACH_VERSION_STRING;
/** argp program arguments documention */
static char args_doc[] = "";
/** argp program doc line */
static char doc[] = "ach benchark program";
/** argp object */
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


typedef struct timespec ticks_t ;
static ticks_t get_ticks(void) {
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t );
    return t;
}
static double ticks_delta(ticks_t t0, ticks_t t1) {
    double d0 = t0.tv_sec + t0.tv_nsec / 1e9;
    double d1 = t1.tv_sec + t1.tv_nsec / 1e9;
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
    for(size_t i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        int r = ach_put(&chan, &ticks, sizeof(ticks));
        assert(ACH_OK == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver(void) {
    fprintf(stderr,"receiver\n");
    make_realtime();
    // flush some initial delayed messages
    for( size_t i = 0; i < 5; i ++ ) {
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
        double result = ticks_delta(ticks,now);
        printf("%f\n", result*1e6);
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
#endif //BENCH_ACH

#ifdef BENCH_PIPE
int fd[2];
void sender(void) {
    fprintf(stderr,"sender\n");
    make_realtime();
    size_t i;
    for(i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        int r = write(fd[1], &ticks, sizeof(ticks));
        assert(sizeof(ticks) == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver(void) {
    fprintf(stderr,"receiver\n");
    make_realtime();
    /* flush some initial delayed messages */
    size_t i;
    for( i = 0; i < 5; i ++ ) {
        ticks_t ticks;
        int r = read(fd[0], &ticks, sizeof(ticks));
        assert(sizeof(ticks) == r);
    }
    /* now the good stuff */
    while(1) {
        ticks_t ticks;
        int r = read(fd[0], &ticks, sizeof(ticks));
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

    argp_parse (&argp, argc, argv, 0, NULL, NULL);
    fprintf(stderr, "freq: %f\n", FREQUENCY);
    fprintf(stderr, "secs: %f\n", SECS);
    size_t i;
    /* warm up */
    for( i = 0; i < 10; i++) get_ticks();

    /* compute overhead */
    calibrate();
    fprintf(stderr,"overhead: %fus\n", overhead*1e6);

    /* setup comms */
    setup();

    /* fork */
    pid_t pid[1];
    for( i = 0; i < sizeof(pid)/sizeof(pid[0]); i ++ ) {
        pid[i] = fork();
        assert( pid[i] >= 0 );
        if(0 == pid[i]) {
            receiver();
            exit(0);
        }
    }
    /* send */
    sender();
    sleep(1);
    /* stop */
    for( i = 0; i < sizeof(pid)/sizeof(pid[0]); i ++ ) {
        kill(pid[i],SIGTERM);
    }
    destroy();
    exit(0);
}

static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; /* ignore unused parameter */
    switch(key) {
        char *endptr = 0;
    case 'f':
        FREQUENCY = strtod(arg, &endptr);
        assert(endptr);
        break;
    case 's':
        SECS = strtod(arg, &endptr);
        assert(endptr);
        break;
    }
    return 0;
}

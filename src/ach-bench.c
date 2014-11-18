/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2012, Georgia Tech Research Corporation
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
#include <sys/resource.h>
#include <sched.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include "ach.h"
#include "achutil.h"

#define STACK_SIZE (8*1024)

double FREQUENCY = (1000.0);
double SECS = 1;
size_t RECV_RT = 1;
size_t RECV_NRT = 0;
size_t SEND_RT = 1;
int PASS_NO_RT = 0;
int KERNDEV = 0;

double overhead = 0;


/**********/
/* TIMING */
/**********/

ach_channel_t time_chan;



typedef struct timespec ticks_t ;
static ticks_t get_ticks(void) {
    struct timespec t;
    clock_gettime( ACH_DEFAULT_CLOCK, &t );
    return t;
}
static double ticks_delta(ticks_t t0, ticks_t t1) {
    double dsec =  ((double)t1.tv_sec -  (double)t0.tv_sec);
    double dnsec = ((double)t1.tv_nsec - (double)t0.tv_nsec) / 1e9;
    return dsec + dnsec - overhead;
}

static void send_time(float t) {
    int r = ach_put(&time_chan, &t, sizeof(t));
    assert(ACH_OK == r);
}

void make_realtime( int priority ) {
    char prefault[STACK_SIZE];
    memset( prefault,0,sizeof(prefault) );

    if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
        fprintf(stderr, "Couldn't lock pages in memory: %s\n",
                strerror(errno) );
        if(!PASS_NO_RT) exit(1);
    }
    struct sched_param sp;
    sp.sched_priority = priority; /* 99 is max priority on linux */
    if( sched_setscheduler( 0, SCHED_FIFO, &sp) < 0 ) {
        fprintf(stderr, "Couldn't set scheduling priority: %s\n",
                strerror(errno) );
        if(!PASS_NO_RT) exit(1);
    }
}

void calibrate(void) {
    make_realtime(30);
    double a = 0;
    ticks_t r0,r1;
    size_t i;
    overhead = 0;
    for( i = 0; i<1000; i++ ) {
        r0 = get_ticks();
        r1 = get_ticks();
        a += ticks_delta(r0,r1);
    }
    overhead = (a) / 1000;
}

void init_time_chan(void) {
    /* create channel */
    ach_create_attr_t cattr;
    int r = ach_unlink("time");               /* delete first */
    assert( ACH_OK == r || ACH_ENOENT == r);
    if (KERNDEV) {
	ach_create_attr_init(&cattr);
	cattr.map = ACH_MAP_KERNEL;
    }
    r = ach_create("time", (size_t)FREQUENCY*(size_t)SECS*RECV_RT,
                   sizeof(float), KERNDEV ? &cattr : NULL );
    assert(ACH_OK == r);

    /* open channel */
    r = ach_open(&time_chan, "time", NULL);
    assert(ACH_OK == r);
}

void print_times(void) {
    while(1) {
        float tm;
        size_t fs;
        struct timespec then;
	if (KERNDEV) {
	    then.tv_sec = 3;
	    then.tv_nsec = 0;
	} else {
	    then = get_ticks();
	    then.tv_sec += 3;
	}
        int r = ach_get(&time_chan, &tm, sizeof(tm), &fs, &then,
                        ACH_O_WAIT);
        if( ACH_TIMEOUT == r ) break;
        assert(ACH_OK == r);
        assert(fs == sizeof(tm));
        printf("%f\n", tm*1e6);
    }
    fflush(stdout);
}

/****************/
/* ACH BENCHING */
/****************/
ach_channel_t chan;
void sender_ach(void) {
    fprintf(stderr,"sender\n");
    make_realtime(98);
    size_t i;
    for( i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        int r = ach_put(&chan, &ticks, sizeof(ticks));
        assert(ACH_OK == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver_ach(int rt) {
    fprintf(stderr,"receiver\n");
    make_realtime(99);
    /* flush some initial delayed messages */
    size_t i;
    for( i = 0; i < 5; i ++ ) {
        ticks_t ticks;
        size_t fs;
        ach_get(&chan, &ticks, sizeof(ticks), &fs, NULL,
                ACH_O_LAST | ACH_O_WAIT);
    }
    /* now the good stuff */

    ticks_t ticks = get_ticks();
    while(1) {
        size_t fs;
        ticks_t then;
	if (KERNDEV) {
	    then.tv_sec = 1;
	    then.tv_nsec = 0;
	} else {
	    then = ticks;
	    then.tv_sec += 1;
	}
        int r = ach_get(&chan, &ticks, sizeof(ticks), &fs, &then,
                        ACH_O_LAST | ACH_O_WAIT);
        ticks_t now = get_ticks();
        if( ACH_TIMEOUT == r ) break;
        assert(ACH_OK == r || sizeof(ticks) == fs);
        /* only print real-time latencies */
        if (rt) {
            send_time((float)ticks_delta(ticks,now));
        }
    }

}

void setup_ach(void) {
    /* create channel */
    int r = ach_unlink("bench");               /* delete first */
    assert( ACH_OK == r || ACH_ENOENT == r);
    ach_create_attr_t cattr;
    if (KERNDEV) {
	ach_create_attr_init(&cattr);
	cattr.map = ACH_MAP_KERNEL;
    }
    r = ach_create("bench", 10, 256, KERNDEV ? &cattr : NULL );
    assert(ACH_OK == r);

    /* open channel */
    r = ach_open(&chan, "bench", NULL);
    assert(ACH_OK == r);
}

void destroy_ach(void) {
    int r = ach_unlink("bench");
    assert(ACH_OK == r);
}

/*****************/
/* PIPE BENCHING */
/*****************/
int fd[2];
void sender_pipe(void) {
    fprintf(stderr,"sender\n");
    make_realtime(98);
    size_t i;
    for(i = 0; i < SECS*FREQUENCY; i ++) {
        ticks_t ticks = get_ticks();
        ssize_t r = write(fd[1], &ticks, sizeof(ticks));
        assert(sizeof(ticks) == r);
        usleep((useconds_t)(1e6/FREQUENCY));
    }
}

void receiver_pipe(int rt) {
    (void)rt;
    fprintf(stderr,"receiver\n");
    make_realtime(99);
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
        send_time((float)ticks_delta(ticks,now));
    }
}

void setup_pipe(void) {
    /* create channel */
    int r = pipe(fd);
    assert( !r );
}
void destroy_pipe(void) {
}


/************/
/* a vtable */
/************/
struct vtab {
    void (*sender)(void);
    void (*receiver)(int rt);
    void (*setup)(void);
    void (*destroy)(void);
};

struct vtab vtab_ach = {
    sender_ach, receiver_ach, setup_ach, destroy_ach
};

struct vtab vtab_pipe = {
    sender_pipe, receiver_pipe, setup_pipe, destroy_pipe
};

/********/
/* MAIN */
/********/

int main(int argc, char **argv) {

    /* parse args */
    int c;
    char *endptr = 0;

    struct vtab *vt = &vtab_ach;

    while( (c = getopt( argc, argv, "f:s:p:r:l:gPhH?Vk")) != -1 ) {
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
	case 'k':
	    KERNDEV = 1;
	    break;
        case 'r':
            RECV_RT = (size_t)atoi(optarg);
            break;
        case 'l':
            RECV_NRT = (size_t)atoi(optarg);
            break;
        case 'g':
            PASS_NO_RT = 1;
            break;
        case 'P':
            vt = &vtab_pipe;
            break;
        case 'V':   /* version     */
            ach_print_version("achbench");
            exit(EXIT_SUCCESS);
        case 'h':
        case 'H':
        case '?':
        default:
            puts("Usage: achbench [OPTION....]\n"
                 "Benchmark Ach IPC\n"
                 "\n"
                 "Note: Disable CPU power-saving to minimize latency\n"
                 "\n"
                 "Options:\n"
                 "  -f FREQUENCY,       Frequency in Hertz (1000)\n"
                 "  -s SECONDS,         Duration in seconds (1)\n"
                 "  -p COUNT,           Real-Time Publishers (1)\n"
                 "  -r COUNT,           Real-Time Receivers (1)\n"
                 "  -l COUNT,           Non-Real-Time Receivers (0)\n"
                 "  -g,                 Proceed even if real-time setup fails\n"
                 "  -P,                 Benchmark pipes instead of ach\n"
		 "  -k,                 Use kernel channels\n"
                );
            exit(EXIT_SUCCESS);
        }
    }

    fprintf(stderr, "-f %.2f ", FREQUENCY);
    fprintf(stderr, "-s %.2f ", SECS);
    fprintf(stderr, "-r %"PRIuPTR" ", RECV_RT);
    fprintf(stderr, "-l %"PRIuPTR" ", RECV_NRT);
    fprintf(stderr, "-p %"PRIuPTR"\n", SEND_RT);
    size_t i;

    init_time_chan();


    /* setup comms */
    vt->setup();

    /* fork printer */
    pid_t pid_print = fork();
    assert( pid_print >= 0 );
    if(0 == pid_print ) {
        print_times();
        exit(0);
    }

    /* warm up */
    for( i = 0; i < 10; i++) get_ticks();

    /* compute overhead */
    calibrate();
    fprintf(stderr,"overhead 0: %fus\n", overhead*1e6);


    /* fork receivers */
    pid_t pid_recv_rt[RECV_RT+RECV_NRT];
    /* non-realtime receivers */
    for( i = 0; i < RECV_NRT; i ++ ) {
        pid_recv_rt[i] = fork();
        assert( pid_recv_rt[i] >= 0 );
        if(0 == pid_recv_rt[i]) {
            vt->receiver(0);
            exit(0);
        }
    }
    /* realtime receivers */
    for( i = RECV_NRT; i < RECV_NRT + RECV_RT; i ++ ) {
        pid_recv_rt[i] = fork();
        assert( pid_recv_rt[i] >= 0 );
        if(0 == pid_recv_rt[i]) {
            vt->receiver(1);
            exit(0);
        }
    }

    /* fork senders */
    pid_t pid_send_rt[SEND_RT];
    for( i = 0; i < SEND_RT; i ++ ) {
        pid_send_rt[i] = fork();
        assert( pid_send_rt[i] >= 0 );
        if(0 == pid_send_rt[i]) {
            vt->sender();
            exit(0);
        }
    }

    /* Wait for senders */
    for( i = 0; i < SEND_RT; i ++ ) {
        int status;
        waitpid( pid_send_rt[i], &status, 0 );
    }
    /* Wait for printer */
    {
        int status;
        waitpid( pid_print, &status, 0 );
    }
    /* Wait for receivers */
    for( i = 0; i < RECV_RT + RECV_NRT; i ++ ) {
        if( vt == &vtab_pipe ) {
            kill(pid_recv_rt[i],SIGTERM);
        }
        int status;
        waitpid( pid_recv_rt[i], &status, 0 );
    }
    vt->destroy();
    exit(0);
}

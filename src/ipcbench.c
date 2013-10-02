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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif //HAVE_CONFIG

#include <signal.h>
#include <sys/wait.h>
#include "ipcbench.h"
#include "util.h"

#define MQ "/ipcbench.latency"
sig_atomic_t sig_canceled = 0;
double opt_freq = 1000;
double opt_sec = 1;
size_t opt_subscribers = 1;

struct timespec ipcbench_period;

#include <mqueue.h>
struct mq_attr mq_lat_attr = {.mq_maxmsg = 512,
                              .mq_msgsize = sizeof(double),
                              .mq_flags = 0};


static void sighandler( int sig ) {
    (void) sig;
    sig_canceled = 1;
}


static void register_handler( ) {
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = &sighandler;
    if( sigaction(SIGUSR1, &act, NULL) ) {
        perror("couldn't register signal handler");
        abort();
    }
}


static void send(struct ipcbench_vtab *vtab) {
    //make_realtime(99);

    usleep(0.25e6);
    if( vtab->init_send ) vtab->init_send();

    while( !sig_canceled ) {
        struct timespec ts = get_ticks();
        clock_gettime( CLOCK_MONOTONIC, &ts );
        vtab->send(&ts);
        clock_nanosleep( CLOCK_MONOTONIC, 0, &ipcbench_period, NULL ); // TODO, handle eintr
    }

    if( vtab->destroy_send ) vtab->destroy_send();
}

static void kill_wait(pid_t pid) {
    if( kill(pid, SIGUSR1) ) {
        perror("Couldn't kill");
        abort();
    }

    int status;
    if( waitpid( pid, &status, 0 ) < 0 ) {
        perror("Couldn't wait");
        abort();

    }
}

static void recv(struct ipcbench_vtab *vtab) {
    //make_realtime(99);
    if( vtab->init_recv ) vtab->init_recv();

    mqd_t mq;
    if( (mq = mq_open(MQ, O_CREAT | O_WRONLY | O_NONBLOCK, 0600, &mq_lat_attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }

    /* warm up */
    for( size_t i = 0; i < 10; i ++ ) {
        struct timespec ts;
        vtab->recv(&ts );
    }

    while(! sig_canceled ) {
        struct timespec ts;
        vtab->recv(&ts );
        struct timespec now = get_ticks();
        double us = ticks_delta( ts, now ) * 1e6;
        if( !sig_canceled ) {
            ssize_t r = mq_send(mq, (char*)&us, sizeof(us), 0);
            if( sizeof(us) == r )  {
                perror("mq send failed \n");
            }
        }
    }

    if( vtab->destroy_recv ) vtab->destroy_recv();
    exit(EXIT_SUCCESS);
}

static void time_print( ) {
    mqd_t mq;
    if( (mq = mq_open(MQ, O_CREAT | O_RDONLY, 0600, &mq_lat_attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }

    while( ! sig_canceled ) {
        double us;
        ssize_t r = mq_receive( mq, (char*)&us, sizeof(us), NULL );
        if( sizeof(us) == r ) printf("%lf\n", us );
    }
}

int main( int argc, char **argv ) {
    (void)argc;
    (void)argv;



    /* Lookup vtab */
    struct {
        const char *name;
        struct ipcbench_vtab *vtab;
    } sym_vtabs[] = {
#ifdef HAVE_ACH_H
        {"ach", &ipc_bench_vtab_ach},
#endif
#ifdef HAVE_LCM_LCM_H
        {"lcm", &ipc_bench_vtab_lcm},
#endif
#ifdef HAVE_TAO_ORB_H
        {"corba", &ipc_bench_vtab_corba},
#endif
        {"pipe", &ipc_bench_vtab_pipe},
        {"mq", &ipc_bench_vtab_mq},
        {"tcp", &ipc_bench_vtab_tcp},
        {"local", &ipc_bench_vtab_local},
        {"udp", &ipc_bench_vtab_udp},
        {"localdgram", &ipc_bench_vtab_local_dgram},
        {NULL, NULL},
    };


    /* Parse args */
    const char *type = "ach";
    char *endptr = 0;
    int c;
    while( (c = getopt( argc, argv, "lt:f:s:v?V")) != -1 ) {
        switch(c) {
        case 'V':   /* version     */
            puts("ipcbench 0.0");
            exit(EXIT_SUCCESS);
        case 'f':
            opt_freq = strtod(optarg, &endptr);
            if( NULL == endptr )  {
                fprintf( stderr, "Invalid frequency: %s\n", optarg );
                exit(EXIT_FAILURE);
            }
            break;
        case 't':
            opt_sec = strtod(optarg, &endptr);
            if( NULL == endptr )  {
                fprintf( stderr, "Invalid duration: %s\n", optarg );
                exit(EXIT_FAILURE);
            }
            break;
        case 'l':
            for( size_t i = 0; sym_vtabs[i].name; i++ ) {
                puts( sym_vtabs[i].name );
            }
            exit(EXIT_SUCCESS);
        case 's':
            opt_subscribers = (size_t)atoi(optarg);
            break;
        case '?':   /* version     */
            puts("Usage: ipcbench [OPTION....] TYPE\n"
                 "Benchmark IPC\n"
                 "\n"
                 "Options:\n"
                 "  -f FREQUENCY,     Frequency in Hertz (1000)\n"
                 "  -t SECONDS,       Duration in seconds (1)\n"
                 "  -s COUNT,         Number of subscribers, supported methods only (1)\n"
                 "  -l                List supported IPC methods\n"
                 "  -V                Version\n"
                 "  -?                Help\n"
                );
            exit(EXIT_SUCCESS);
        default:
            type = optarg;
        }
    }
    while( optind < argc ) {
        type = argv[optind++];
    }


    fprintf(stderr, "type: %s\n", type );

    struct ipcbench_vtab *vtab;
    {
        size_t i = 0;
        while( sym_vtabs[i].name && 0 != strcasecmp( type, sym_vtabs[i].name ) ) i++;
        if( NULL == sym_vtabs[i].name ) {
            fprintf(stderr, "Invalid type: %s\n", type );
            exit(EXIT_FAILURE);
        }
        vtab = sym_vtabs[i].vtab;
    }

    /* Compute period */
    {
        double period = 1.0 / opt_freq;
        ipcbench_period.tv_sec = period;
        ipcbench_period.tv_nsec = (period - (time_t)period) * 1e9;
    }

    /* Setup message queue */
    mq_unlink(MQ);
    pid_t pid_time_print = fork();
    if( 0 == pid_time_print ) {
        register_handler();
        time_print();
        return 0;
    } else if (pid_time_print < 0 ) {
        perror("Couldn't fork\n");
        abort();
    }

    /* Make realtime */
    make_realtime(30);
    calibrate();
    fprintf(stderr, "overhead: %f us\n", overhead * 1e6);

    /* Execute */
    if( vtab->init ) vtab->init();

    pid_t pid_send = fork();
    if( 0 == pid_send ) {
        register_handler();
        send(vtab);
        return 0;
    } else if ( pid_send < 0 ) {
        perror("Couldn't fork\n");
        abort();
    }

    pid_t pid_listen[opt_subscribers];
    for( size_t i = 0; i < opt_subscribers; i ++ ) {
        pid_listen[i] = fork();
        if( 0 == pid_listen[i] ) {
            register_handler();
            recv(vtab);
            return 0;
        } else if ( pid_listen[i] < 0 ) {
            perror("Couldn't fork\n");
            abort();
        }
    }

    sleep(opt_sec);
    for( size_t i = 0; i < opt_subscribers; i ++ ) {
        kill_wait(pid_listen[i]);
    }
    kill_wait(pid_send);

    usleep(100e3);
    kill_wait(pid_time_print);

    if( vtab->destroy ) vtab->destroy();

    return 0;

}

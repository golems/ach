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

sig_atomic_t sig_canceled = 0;
double opt_freq = 1000;
double opt_sec = 1;
size_t opt_subscribers = 1;
size_t opt_nonrt_subscribers = 0;
size_t opt_discard = 10;

struct timespec ipcbench_period;
double overhead = 0;

#include <mqueue.h>
struct mq_attr mq_lat_attr = {.mq_maxmsg = 10,
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
    register_handler();

    usleep(0.25e6);
    if( vtab->init_send ) vtab->init_send(ipcbench_cnt);

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

static void recv(struct ipcbench_vtab *vtab, int emit) {
    register_handler();
    //make_realtime(99);
    if( vtab->init_recv ) vtab->init_recv(ipcbench_cnt);

    mqd_t mq;
    if( (mq = mq_open(MQ, O_CREAT | O_WRONLY | O_NONBLOCK, 0600, &mq_lat_attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }

    /* warm up */
    for( size_t i = 0; i < opt_discard; i ++ ) {
        struct timespec ts;
        vtab->recv(&ts );
    }

    while(! sig_canceled ) {
        struct timespec ts;
        vtab->recv(&ts );
        struct timespec now = get_ticks();
        if( emit && !sig_canceled ) {
            double us = ticks_delta( ts, now ) * 1e6;
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
    register_handler();
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
        int multi_receiver;
    } sym_vtabs[] = {
#ifdef HAVE_ACH_H
        {"ach_user", &ipc_bench_vtab_ach_user, 1},
        {"ach_kernel", &ipc_bench_vtab_ach_kernel, 1},
#endif
#ifdef HAVE_LCM_LCM_H
        {"lcm", &ipc_bench_vtab_lcm, 1},
#endif
#ifdef HAVE_TAO_ORB_H
        {"corba", &ipc_bench_vtab_corba, 0},
        {"cos", &ipc_bench_vtab_cos, 1},
#endif
        {"pipe", &ipc_bench_vtab_pipe, 0},
        {"mq", &ipc_bench_vtab_mq, 0},
        {"tcp", &ipc_bench_vtab_tcp, 0},
        {"local", &ipc_bench_vtab_local, 0},
        {"udp", &ipc_bench_vtab_udp, 0},
        {"udp_multicast", &ipc_bench_vtab_udp_multicast, 1},
        {"localdgram", &ipc_bench_vtab_local_dgram, 0},
        {NULL, NULL, 0},
    };


    /* Parse args */
    const char *type = "ach";
    char *endptr = 0;
    int c;
    while( (c = getopt( argc, argv, "lt:f:s:S:c:d:v?V")) != -1 ) {
        switch(c) {
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
            for( size_t i = 0; NULL != sym_vtabs[i].name; i++ ) {
                puts( sym_vtabs[i].name );
            }
            exit(EXIT_SUCCESS);
        case 's':
            opt_subscribers = (size_t)atoi(optarg);
            break;
        case 'S':
            opt_nonrt_subscribers = (size_t)atoi(optarg);
            break;
        case 'd':
            opt_discard = (size_t)atoi(optarg);
            break;
        case 'c':
            ipcbench_cnt = (size_t)atoi(optarg);
            break;
        case 'V':   /* version     */
            puts("ipcbench " PACKAGE_VERSION);
            exit(EXIT_SUCCESS);
        case '?':
            puts("Usage: ipcbench [OPTION....] TYPE\n"
                 "Benchmark IPC\n"
                 "\n"
                 "Options:\n"
                 "  -f FREQUENCY,     Frequency in Hertz (1000)\n"
                 "  -t SECONDS,       Duration in seconds (1)\n"
                 "  -s COUNT,         Number of subscribers (1)\n"
                 "  -S COUNT,         Number of non-real-time subscribers (0)\n"
                 "  -d COUNT,         Initial messages to discard (10)\n"
                 "  -c COUNT,         Number of channels (1)\n"
                 "  -l                List supported IPC methods\n"
                 "  -V                Version\n"
                 "  -?                Help\n"
                 "\n"
                 "Examples:\n"
                 "  ipcbench -f 1000 -t 60 -s 2 ach     Benchmark Ach at 1000 hertz\n"
                 "                                      for 60 seconds with 2 subscribers\n"
                 "  ipcbench -f 1000 -t 120 mq          Benchmark Message Queueus at 1000 hertz\n"
                 "                                      for 120 seconds\n"
                 "  ipcbench -l                         List supported IPC methods\n"
                 "\n"
                 "Report bugs to <" PACKAGE_BUGREPORT ">"
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

        if( opt_subscribers + opt_nonrt_subscribers > 1 && !sym_vtabs[i].multi_receiver ) {
            fprintf(stderr, "%s does not support multiple subscribers\n", sym_vtabs[i].name );
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
    ipcbench_pfd = (struct pollfd*)calloc(ipcbench_cnt, sizeof(struct pollfd));
    for( size_t i = 0; i < ipcbench_cnt; i ++ ) {
        ipcbench_pfd[i].events = POLLIN;
    }
    if( vtab->init ) vtab->init();

    /* Fork subscribers */
    pid_t pid_listen[opt_subscribers + opt_nonrt_subscribers];
    for( size_t i = 0; i < opt_subscribers + opt_nonrt_subscribers; i ++ ) {
        pid_listen[i] = fork();
        if( 0 == pid_listen[i] ) {
            int emit = 1;
            if( i >= opt_subscribers ) {
                make_realtime(0);
                emit=0;
            }
            if( vtab->recv_loop ) vtab->recv_loop(emit);
            else recv(vtab, emit);
            return 0;
        } else if ( pid_listen[i] < 0 ) {
            perror("Couldn't fork\n");
            abort();
        }
    }

    /* Fork Publishers */
    pid_t pid_send = fork();
    if( 0 == pid_send ) {
        if( vtab->send_loop ) vtab->send_loop();
        else send(vtab);
        return 0;
    } else if ( pid_send < 0 ) {
        perror("Couldn't fork\n");
        abort();
    }

    /* Wait */
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

size_t ipcbench_cnt = 1;
struct pollfd *ipcbench_pfd = NULL;

size_t pollin()
{
    size_t i_fd = 0;
    if( ipcbench_cnt > 1 ) {
        int k = poll( ipcbench_pfd, ipcbench_cnt, -1);
        if( k < 0 ) {
            perror("poll");
            abort();
        }
        for( i_fd=0;  i_fd < ipcbench_cnt; i_fd++ ) {
            if(ipcbench_pfd[i_fd].revents & POLLIN) {
                return i_fd;
            }

        }
        fprintf(stderr, "Poll succeeded, but nothing is ready\n");
        abort();
    }
    return i_fd;
}

size_t pubnext() {
    static size_t i = 0;
    size_t j = i;
    i = (i+1) % ipcbench_cnt;
    return j;

}

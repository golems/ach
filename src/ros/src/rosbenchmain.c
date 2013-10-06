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

#include  "rosbench.h"

struct mq_attr mq_lat_attr = {.mq_maxmsg = 512,
                              .mq_msgsize = sizeof(double),
                              .mq_flags = 0};


static void time_print( mqd_t mq ) {
    while( 1 ) {
        double us;
        ssize_t r = mq_receive( mq, (char*)&us, sizeof(us), NULL );
        if( sizeof(us) == r ) printf("%lf\n", us );
    }
}


int main( int argc, char **argv) {

    int opt_freq = 1000;
    int opt_sec = 1;
    int opt_subscribers = 1;
    int opt_nonrt_subscribers = 0;
    int opt_discard = 120; // ROS UDP is slow for ~100 messages
    int c;
    const char *opt_transport = "tcp";
    char *endptr;
    while( (c = getopt( argc, argv, "t:f:s:S:v?V")) != -1 ) {
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
        case 's':
            opt_subscribers = (size_t)atoi(optarg);
            break;
        case 'S':
            opt_nonrt_subscribers = (size_t)atoi(optarg);
            break;
        case '?':   /* version     */
            puts("Usage: rosbench [OPTION....] \n"
                 "Benchmark IPC\n"
                 "\n"
                 "Options:\n"
                 "  -f FREQUENCY,     Frequency in Hertz (1000)\n"
                 "  -t SECONDS,       Duration in seconds (1)\n"
                 "  -s COUNT,         Number of subscribers (1)\n"
                 "  -S COUNT,         Number of non-real-time subscribers (0)\n"
                 "  -V                Version\n"
                 "  -?                Help\n"
                );
            exit( EXIT_SUCCESS );
        default:
            opt_transport = optarg;
        }
    }
    while( optind < argc ) {
        opt_transport = argv[optind++];
    }
    fprintf(stderr, "transport: %s\n", opt_transport);

    enum rosbench_transport transport;
    if( 0 == strcasecmp(opt_transport, "tcp") ) {
        transport = ROS_TCP;
    } else if( 0 == strcasecmp(opt_transport, "udp") ) {
        transport = ROS_UDP;
    } else {
        fprintf(stderr, "Unknown transport: %s\n", opt_transport );
        exit(EXIT_FAILURE);
    }


    mqd_t mq;
    if( (mq = mq_open("/rosmq", O_CREAT | O_RDWR, 0600, &mq_lat_attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }


    pid_t pid_mq = fork();
    if( 0 == pid_mq ) {
        time_print(mq);
    } else if( 0 > pid_mq ) {
        perror("printer fork");
        abort();
    }

    make_realtime(30);

    /* Fork subscribers */
    pid_t pid_listen[opt_subscribers + opt_nonrt_subscribers];
    {
        size_t i;
        for( i = 0; i < opt_subscribers + opt_nonrt_subscribers; i ++ ) {
            pid_listen[i] = fork();
            if( 0 == pid_listen[i] ) {
                int emit = 1;
                if( i >= opt_subscribers ) {
                    make_realtime(0);
                    emit=0;
                }
                subscribe( argc, argv, i, i < opt_subscribers, transport, opt_discard, mq );
                return 0;
            } else if ( pid_listen[i] < 0 ) {
                perror("Couldn't fork\n");
                abort();
            }
        }
    }


    pid_t pub = fork();
    if( 0 == pub ) {
        publish(argc, argv, opt_freq);
    } else if( 0 > pub ) {
        perror("pub fork");
        abort();
    }

    sleep(opt_sec);

    {
        size_t i;
        for( i = 0; i < opt_subscribers; i ++ ) {
            kill(pid_listen[i], SIGTERM);
        }
    }


    kill(pub, SIGTERM);
    usleep(100e3);
    kill(pid_mq, SIGTERM);


}

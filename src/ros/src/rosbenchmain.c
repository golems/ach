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

#include  <unistd.h>
#include  <stdlib.h>
#include  <signal.h>
#include  "rosbench.h"

int main( int argc, char **argv) {

    int opt_freq = 1000;
    int opt_sec = 1;
    int opt_subscribers = 1;
    int opt_nonrt_subscribers = 0;
    int c;
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
        default:
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
            exit( '?' == c ? EXIT_SUCCESS : EXIT_FAILURE);
        }
    }

    make_realtime(30);


    pid_t pub = fork();
    if( 0 == pub ) {
        publish(argc, argv, opt_freq);
    } else if( 0 > pub ) {
        perror("pub fork");
        abort();
    }

    pid_t sub = fork();
    if( 0 == sub ) {
        subscribe(argc, argv);
    } else if( 0 > sub ) {
        perror("sub fork");
        abort();
    }

    sleep(opt_sec);
    kill(pub, SIGTERM);
    kill(sub, SIGTERM);


}

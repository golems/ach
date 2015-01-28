/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/wait.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <errno.h>
#include "ach.h"
#include "achtest.h"

#define OPT_CHAN  "ach-test"

size_t opt_msg_cnt = 10;
size_t opt_msg_size = 16;

volatile sig_atomic_t count_sigusr1 = 0;

static void sighandler_count(int sig, siginfo_t *siginfo, void *context) {
    (void)sig; (void)siginfo; (void)context;
    if( SIGUSR1 == sig ) count_sigusr1++;
}

static void sighandler_install() {
    struct sigaction act;
    memset(&act, 0, sizeof(act));

    act.sa_sigaction = &sighandler_count;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(SIGUSR1, &act, NULL) < 0) {
        fprintf( stderr, "Couldn't install signal handler: %s", strerror(errno) );
        exit(EXIT_FAILURE);
    }
}

static int testsig_gp(pid_t pid_p)
{
    int status;
    pid_t wp = wait(&status);
    if( wp != pid_p ) fail_errno("Wait 0");
    if( WIFEXITED(status) && (0 == WEXITSTATUS(status)) ) {
        /* This is the end */
        /* Unlink kernel channel */
        check_ach( "unlink end", ach_unlink(OPT_CHAN) );
        return 0;
    } else {
        fprintf(stderr, "Child 0 failed\n");
        exit(EXIT_FAILURE);
    }
}

static int testsig(void)
{
    enum ach_status r;
    /* Fork 0 */
    pid_t pid_p = fork();
    check_errno( "Fork 0", pid_p );

    /* GP: wait */
    if( pid_p ) {
        return testsig_gp(pid_p);
    }

    /* Parent */
    /* Create Kernel Channel */
    {
        ach_create_attr_t attr;
        ach_create_attr_init(&attr);
        attr.map = ACH_MAP_KERNEL;

        r = ach_unlink(OPT_CHAN);
        if( ! ach_status_match(r, ACH_MASK_OK | ACH_MASK_ENOENT) ) {
            fail_ach( "unlink before create", r );
        }
        r = ach_unlink(OPT_CHAN);
        if( ACH_ENOENT != r ) fail_ach( "unlink noent", r );

        check_ach( "ach_create",
                   ach_create( OPT_CHAN, opt_msg_cnt, opt_msg_size, &attr ) );
    }
    /* Open Kernel Channel */
    struct ach_channel chan;
    {
        for(;;) {
            usleep(1000); /* Race against udev */
            r = ach_open( &chan, OPT_CHAN, NULL );
            if( ACH_EACCES == r ) continue;
            else if (ACH_OK == r) break;
            else fail_ach("ach_open", r);

        }
    }

    /* Install Parent sighandler */
    sighandler_install();

    /* fork 1 */
    pid_t pid_c = fork();
    check_errno( "fork 1", pid_c );

    if( pid_c ) {
        /* Parent: */
        for(;;) {
            usleep(10000); /* Racy... */
            check_errno( "kill child", kill( pid_c, SIGUSR1) );
            usleep(10000);
            int i = 42;
            check_ach( "put to child",  ach_put( &chan, &i, sizeof(i) ) );
            int status;
            pid_t wp = waitpid( pid_c, &status, WNOHANG );
            if( wp ) {
                if( wp != pid_c ) {
                    fail_errno("Wait 1");
                } else if( WIFEXITED(status) && (0 == WEXITSTATUS(status)) ) {
                    exit(EXIT_SUCCESS);
                } else {
                    fprintf(stderr, "Child 1 failed\n");
                    exit(EXIT_FAILURE);
                }
            }
        }
    } else {
        /* child */
        sig_atomic_t s0, s1;
        int i;
        do {
            size_t frame_size;
            s0 = count_sigusr1;
            r = ach_get(&chan, &i, sizeof(i), &frame_size, NULL, ACH_O_WAIT  );
            s1 = count_sigusr1;
            check_ach("child sig get", r);
        } while( s1 == s0 || s1 < 10 ); /* This is racy... */
        printf("done: %s, %d,%d,%d\n", ach_result_to_string(r), s0, s1, i);
        exit(EXIT_SUCCESS);
    }

    return 0;
}

static int testfault(void)
{
    if( ACH_OK == ach_create( NULL, opt_msg_cnt, opt_msg_size, NULL ) ) {
        fprintf(stderr, "Error on ach_create with null channel\n");
        exit(EXIT_FAILURE);
    }
    return 0;
}

int main( int argc, char **argv ){
    (void) argc; (void) argv;

    testsig();

    testfault();
    return 0;
}

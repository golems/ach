/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#define OPT_CHAN  "ach-test-cancel"

static void test(ach_status_t r, const char *thing) {
    if( r != ACH_OK ) {
        fprintf(stderr, "%s: %s\n",
                thing, ach_result_to_string(r));
        exit(-1);
    }
}

ach_channel_t channel;

static void sighandler (int sig ) {
    if( SIGALRM != sig ) exit(EXIT_FAILURE);
    enum ach_status r = ach_cancel(&channel, NULL);
    if( ACH_OK != r ) exit(EXIT_FAILURE);
}

static void check() {
    enum ach_status r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_WAIT );
    if( r != ACH_CANCELED ) {
        exit(EXIT_FAILURE);
    }
}

int main( int argc, char **argv ){
    (void)argc; (void)argv;

    /* unlink */
    ach_status_t r = ach_unlink(OPT_CHAN);
    if( ! (ACH_OK==r || ACH_ENOENT == r) ) {
        fprintf(stderr, "ach_unlink failed\n: %s",
                ach_result_to_string(r));
        return -1;
    }

    /* create */
    r = ach_create(OPT_CHAN, 32ul, 64ul, NULL );
    test(r, "ach_create");

    /* open */
    r = ach_open(&channel, OPT_CHAN, NULL);
    test(r, "ach_open");

    /* unlink */
    r = ach_unlink(OPT_CHAN);
    test(r, "ach_unlink");

    /* Install signal handler */
    {
        struct sigaction act;
        memset(&act, 0, sizeof(act));
        act.sa_handler = &sighandler;
        if( sigaction(SIGALRM, &act, NULL) ) exit(EXIT_FAILURE);
    }

    {
        ach_cancel_attr_t attr;
        ach_cancel_attr_init(&attr);
        attr.async_unsafe = 1;
        r = ach_cancel(&channel, &attr);
        test(r, "cancel");
    }
    check();
    channel.cancel = 0;

    /* Set alarm */
    alarm(1);

    /* No race here since if cancel is set before the call to ach_get,
     * we still notice it */

    /* ach_get */
    int i;
    for( i = 0; i < 2; i ++ ) {
        check();
    }

    return 0;

}

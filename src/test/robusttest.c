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
#include <assert.h>
#include "ach.h"
#include "ach_private_posix.h"

#define OPT_CHAN  "ach-test-robust"

static void test(int t, enum ach_status r, const char *thing) {
    if( !t ) {
        fprintf(stderr, "%s failed: %s\n",
                thing, ach_result_to_string(r));
        exit(-1);
    }
}

static void make_locked( ) {
    pid_t pid = fork();
    if( 0 == pid ) { /* child */
        ach_channel_t channel;
        /* open */
        enum ach_status r = ach_open(&channel, OPT_CHAN, NULL);
        test(ACH_OK == r, r, "ach_open");

        pthread_mutex_lock(&channel.shm->sync.mutex);
        exit(EXIT_SUCCESS);

    } else if (0 < pid) {
        wait( NULL );
    } else {
        exit(EXIT_FAILURE);
    }
}

int main( int argc, char **argv ){
    (void)argc; (void) argv;

    ach_channel_t channel;

    ach_status_t r;

    /* unlink */
    r = ach_unlink(OPT_CHAN);
    test( (ACH_OK==r || ACH_ENOENT == r),
          r, "ach_unlink");

    /* create */
    r = ach_create(OPT_CHAN, 32ul, 64ul, NULL );
    test(ACH_OK == r, r, "ach_create");


    /* open */
    r = ach_open(&channel, OPT_CHAN, NULL);
    test(ACH_OK == r, r, "ach_open");

    /* first test */
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    test( ACH_STALE_FRAMES == r, r, "get stale");

    /* read test */
    make_locked();
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    test( ACH_STALE_FRAMES == r, r, "get stale");

    /* corrupt test */
    make_locked();
    channel.shm->sync.dirty = 1;
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    test( ACH_CORRUPT == r, r, "get corrupt");
    /* and again */
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    test( ACH_CORRUPT == r, r, "get corrupt");

    /* another read test */
    channel.shm->sync.dirty = 0;
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    r = ach_get( &channel, NULL, 0, NULL, NULL, ACH_O_LAST );
    test( ACH_STALE_FRAMES == r, r, "get stale");

    return 0;

}

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

int main( int argc, char **argv ){
    (void) argc; (void) argv;
    size_t i = 0;
    enum ach_status r;

    ach_create_attr_t attr;
    ach_create_attr_init(&attr);
    attr.map = ACH_MAP_KERNEL;

    r = ach_unlink(OPT_CHAN);
    if( ! ach_status_match(r, ACH_MASK_OK | ACH_MASK_ENOENT) ) {
        fail_ach( "unlink before create", r );
    }

    /* Create */
    check_ach( "ach_create",
               ach_create( OPT_CHAN, opt_msg_cnt, opt_msg_size, &attr ) );

    while(i++ < (size_t)1e6) {
        if( 0 == i % (size_t)1e3 ) {
            printf("iteration %lu\n", i );
        }

        /* Open */
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


        /* Close */
        check_ach("close", ach_close(&chan) );

        /* Unlink */
        check_ach("unlink", ach_unlink(OPT_CHAN) );

        /* Create */
        check_ach( "ach_create",
                   ach_create( OPT_CHAN, opt_msg_cnt, opt_msg_size, &attr ) );

    }

    /* Unlink */
    check_ach("unlink", ach_unlink(OPT_CHAN) );

    return 0;
}

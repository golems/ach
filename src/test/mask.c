/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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

int main( int argc, char **argv ){
    (void) argc; (void) argv;

    int i = 0;
    for(;;) {
        enum ach_status r = (enum ach_status)i;
        switch(r) {
        case ACH_OVERFLOW:
        case ACH_BAD_SHM_FILE:
        case ACH_INVALID_NAME:
        case ACH_FAILED_SYSCALL:
        case ACH_MISSED_FRAME:
        case ACH_TIMEOUT:
        case ACH_STALE_FRAMES:
        case ACH_ENOENT:
        case ACH_CLOSED:
        case ACH_BUG:
        case ACH_EINVAL:
        case ACH_CORRUPT:
        case ACH_BAD_HEADER:
        case ACH_EACCES:
        case ACH_CANCELED:
        case ACH_EFAULT:
        case ACH_EINTR:
        case ACH_ENOTSUP:
        case ACH_EEXIST:
            CHECK_TRUE("Mask values\n",
                       ACH_STATUS_MASK(i) ==
                       2*ACH_STATUS_MASK(i-1) );
        case ACH_OK:
            printf("%-18s (%02d): 0x%05x\n",
                   ach_result_to_string(r),
                   r, ach_status_mask(r));
            i++;
            continue;
        }
        return 0;
    }
    return -1;
}

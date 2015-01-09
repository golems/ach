/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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
 *   * Neither the name of Rice University nor the names of its
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

#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/stat.h>

#include <string.h>
#include <inttypes.h>

#include "ach.h"
#include "libach_private.h"

/** \file libach_private.h
 *
 *  \brief This file defines helper functions and variables to be used
 *         by language bindings; it is used by the Python ctypes
 *         interface.
 *
 *  \author Neil T. Dantam
 *
 */

ach_channel_t *ach_channel_alloc(void);
void  ach_channel_free( ach_channel_t *);


ach_channel_t *ach_channel_alloc(void) {
    return (ach_channel_t*)malloc(sizeof(ach_channel_t));
}

void ach_channel_free( ach_channel_t *chan) {
    free(chan);
}

const int ach_ok             = ACH_OK;
const int ach_overflow       = ACH_OVERFLOW;
const int ach_invalid_name   = ACH_INVALID_NAME;
const int ach_bad_shm_file   = ACH_BAD_SHM_FILE;
const int ach_failed_syscall = ACH_FAILED_SYSCALL;
const int ach_stale_frames   = ACH_STALE_FRAMES;
const int ach_missed_frame   = ACH_MISSED_FRAME;
const int ach_timeout        = ACH_TIMEOUT;
const int ach_eexist         = ACH_EEXIST;
const int ach_enoent         = ACH_ENOENT;
const int ach_closed         = ACH_CLOSED;
const int ach_bug            = ACH_BUG;
const int ach_einval         = ACH_EINVAL;
const int ach_corrupt        = ACH_CORRUPT;
const int ach_bad_header     = ACH_BAD_HEADER;
const int ach_eacces         = ACH_EACCES;

const int ach_o_wait         = ACH_O_WAIT;
const int ach_o_last         = ACH_O_LAST;

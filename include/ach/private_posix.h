/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2014, Georgia Tech Research Corporation
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

/** \file ach_private_posix.h
 *  \author Neil T. Dantam
 */

#ifndef ACH_PRIVATE_POSIX_H
#define ACH_PRIVATE_POSIX_H

#ifdef __cplusplus
extern "C" {
#endif

#define ACH_POSIX


/** prefix to apply to channel names to get the shared memory file name */
#define ACH_SHM_CHAN_NAME_PREFIX_PATH "/dev/shm"
#define ACH_SHM_CHAN_NAME_PREFIX_NAME "/achshm-"

/** Number of times to retry a syscall on EINTR before giving up */
#define ACH_INTR_RETRY 8


#include "private_generic.h"

    /** Format for ach frames sent over pipes or stored on disk */
    typedef struct {
        char magic[8];         /**< magic number: "achpipe", null terminated */
        uint8_t size_bytes[8]; /**< size, stored little endian for disk and network transmission */
        uint8_t data[1];       /**< flexible array */
    } ach_pipe_frame_t;

    /** Malloc an ach_pipe_frame_t with room for `size' data bytes.
     *
     * \return a newly allocated ach_pipe_frame with its magic and
     * size fields properly filled.
     */
    ach_pipe_frame_t *ach_pipe_alloc(size_t size);

    /** Set size field in ach frame, always stored little endian.
     * \param frame The frame struct
     * \param size The size in native byte order
     */
    void ach_pipe_set_size(ach_pipe_frame_t *frame, uint64_t size);

    /** Set size field in ach frame, always stored little endian.
     * \param frame The frame struct
     * \returns The size in native byte order
     */
    uint64_t ach_pipe_get_size(const ach_pipe_frame_t *frame );

    void ach_set_errstr( const char *str );

#define SYSCALL_RETRY( expr, test )                     \
    {                                                   \
        int ach_intr_cnt = 0;                           \
        do { (expr); }                                  \
        while( (test) &&                                \
               EINTR == errno &&                        \
               ach_intr_cnt++  < ACH_INTR_RETRY         \
            );                                          \
    }


static enum ach_status
check_errno(void) {
    switch(errno) {
    case 0:                   return ACH_OK;
    case EMSGSIZE:            return ACH_OVERFLOW;
    case ENAMETOOLONG:        return ACH_INVALID_NAME;
    case EBADSLT:             return ACH_BAD_SHM_FILE;
    case EIO:                 return ACH_FAILED_SYSCALL;
    case EAGAIN:              return ACH_STALE_FRAMES;
    case EREMOTEIO:           return ACH_MISSED_FRAME;
    case ETIME:               return ACH_TIMEOUT;
    case EEXIST:              return ACH_EEXIST;
    case ENOENT:              return ACH_ENOENT;
    case ESHUTDOWN:           return ACH_CLOSED;
    case EPERM:               return ACH_BUG;
    case EINVAL:              return ACH_EINVAL;
    case EUCLEAN:             return ACH_CORRUPT;
    case EPROTO:              return ACH_BAD_HEADER;
    case EACCES:              return ACH_EACCES;
    case ECANCELED:           return ACH_CANCELED;
    case EFAULT:              return ACH_EFAULT;
    case EINTR:               return ACH_EINTR;
    default:
        return ACH_FAILED_SYSCALL;
    }
}
static inline enum ach_status
check_ret_errno(int ret) {
    if( ret < 0 ) return check_errno();
    else return ACH_OK;
}



#ifdef __cplusplus
}
#endif

#endif /* ACH_PRIVATE_POSIX_H */

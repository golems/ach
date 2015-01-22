/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
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

/** \file generic.h
 *
 *  \brief This file contains declarations needed by both the
 *         userspace public interface and the Linux kernel
 *         implementation.
 *
 *  \author Neil T. Dantam
 */


#ifndef ACH_GENERIC_H
#define ACH_GENERIC_H


#ifdef __cplusplus
extern "C" {
#endif


/** return status codes for ach functions */
typedef enum ach_status {
    ACH_OK = 0,             /**< Call successful */
    ACH_OVERFLOW = 1,       /**< destination too small to hold frame */
    ACH_INVALID_NAME = 2,   /**< invalid channel name */
    ACH_BAD_SHM_FILE = 3,   /**< channel file didn't look right */
    ACH_FAILED_SYSCALL = 4, /**< a system call failed */
    ACH_STALE_FRAMES = 5,   /**< no new data in the channel */
    ACH_MISSED_FRAME = 6,   /**< we missed the next frame */
    ACH_TIMEOUT = 7,        /**< timeout before frame received */
    ACH_EEXIST = 8,         /**< channel file already exists */
    ACH_ENOENT = 9,         /**< channel file doesn't exist */
    ACH_CLOSED = 10,        /**< unused */
    ACH_BUG = 11,           /**< internal ach error */
    ACH_EINVAL = 12,        /**< invalid parameter */
    ACH_CORRUPT = 13,       /**< channel memory has been corrupted */
    ACH_BAD_HEADER = 14,    /**< an invalid header was given */
    ACH_EACCES = 15,        /**< permission denied */
    ACH_CANCELED = 16,      /**< operation canceled */
    ACH_EFAULT = 17         /**< bad address for data copy */
} ach_status_t;



/** Option flags for ach_get().
 *
 * Default behavior is to retrieve the oldest unseen frame without
 * waiting.*/
typedef enum {
    /** Blocks until an unseen message arrives
     *  or timeout.  If the channel already has data that this subscriber
     *  has not seen, ach_get() immediately copies the new data.
     *  Otherwise, it waits for some other process or thread to put data
     *  into the channel.
     */
    ACH_O_WAIT = 0x01,
    /** Reads the newest message out of the channel.  If the channel
     * contains multiple messages that this subscriber has not seen,
     * ach_get() will return the newest of these messages.  The subscriber
     * will skip past all older messages.
     */
    ACH_O_LAST = 0x02,
    /** Copy the message out of the channel, even if already seen.
     *  Return code of ach_get() for successful copy will be ACH_OK.
     */
    ACH_O_COPY = 0x04,

    /** Timeout is a relative time.
     */
    ACH_O_RELTIME = 0x08
} ach_get_opts_t;

/**  maximum size of a channel name */
#define ACH_CHAN_NAME_MAX 64ul


/** Function type to transfer data out of the channel.
 *
 * This function could, for example, perform tasks such as
 * de-serialization and memory allocation.
 *
 * \returns 0 on success, nonzero on failure
 */
typedef enum ach_status
ach_get_fun(void *cx, void **obj_dst, const void *chan_src, size_t frame_size );


/** Function type to transfer data into the channel.
 *
 * This function could, for example, perform tasks such as serialization.
 *
 * \returns 0 on success, nonzero on failure
 */
typedef enum ach_status
ach_put_fun(void *cx, void *chan_dst, const void *obj_src);

#ifdef __cplusplus
}
#endif

#endif

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

package org.golems.ach;

/** Low-level JNI interface to libach
 */
public class Lib
{

    static{
        System.loadLibrary("achj");
    }

    /* Native Function Wrappers */

    /** Wrapper for ach_open.
     *
     * @param channel_name Name of the channel to open
     * @param chan_ptr Array in which the channel pointer is returned
     *
     * @return An integer ach_status code and writes the channel pointer to element 0 of chan_ptr array
     *
     */
    public static native int open(String channel_name, long[] chan_ptr);

    /** Wrapper for ach_close.
     *
     * @param chan_ptr The channel pointer returned by ach_open
     *
     * @return An integer ach_status code
     */
    public static native int close(long chan_ptr);

    /** Wrapper for ach_flush.
     *
     * @param chan_ptr The channel pointer returned by ach_open
     *
     * @return An integer ach_status code
     */
    public static native int flush(long chan_ptr);

    /** Wrapper for ach_put.
     *
     * @param chan_ptr The channel pointer returned by ach_open
     * @param buf The byte array to put to the channel
     *
     * @return An integer ach_status code
     */
    public static native int put(long chan_ptr, byte[] buf);

    /** Wrapper for ach_get without timeout.
     *
     * @param chan_ptr The channel pointer returned by ach_open
     * @param buf The byte array hold message data
     * @param frame_size On return, element 0 holds the number of bytes read from the channel
     * @param options OR'ed ach_get_opts values
     *
     * @return An integer ach_status code
     */
    public static native int get(long chan_ptr, byte[] buf, long[] frame_size, int options);

    /** Wrapper for ach_get with timeout.
     *
     * @param chan_ptr The channel pointer returned by ach_open
     * @param buf The byte array hold message data
     * @param frame_size On return, element 0 holds the number of bytes read from the channel
     * @param options OR'ed ach_get_opts values
     * @param sec Absolute timeout seconds
     * @param nsec Absolute timeout nanoseconds
     *
     * @return An integer ach_status code
     */
    public static native int get(long chan_ptr, byte[] buf, long[] frame_size, long sec, int nsec, int options);

    /** Get the current time using the default ach clock.
     *
     * @param abstime An array of length 2, in which the seconds and
     * nanoseconds are returned.
     */
    public static native int time( long[] abstime );

    /* Constant accessor functions */

    /** Return ach_status constant of same name */
    public static native int ACH_OK();
    /** Return ach_status constant of same name */
    public static native int ACH_OVERFLOW();
    /** Return ach_status constant of same name */
    public static native int ACH_INVALID_NAME();
    /** Return ach_status constant of same name */
    public static native int ACH_BAD_SHM_FILE();
    /** Return ach_status constant of same name */
    public static native int ACH_FAILED_SYSCALL();
    /** Return ach_status constant of same name */
    public static native int ACH_STALE_FRAMES();
    /** Return ach_status constant of same name */
    public static native int ACH_MISSED_FRAME();
    /** Return ach_status constant of same name */
    public static native int ACH_TIMEOUT();
    /** Return ach_status constant of same name */
    public static native int ACH_EEXIST();
    /** Return ach_status constant of same name */
    public static native int ACH_ENOENT();
    /** Return ach_status constant of same name */
    public static native int ACH_CLOSED();
    /** Return ach_status constant of same name */
    public static native int ACH_BUG();
    /** Return ach_status constant of same name */
    public static native int ACH_EINVAL();
    /** Return ach_status constant of same name */
    public static native int ACH_CORRUPT();
    /** Return ach_status constant of same name */
    public static native int ACH_BAD_HEADER();
    /** Return ach_status constant of same name */
    public static native int ACH_EACCES();
    /** Return ach_status constant of same name */
    public static native int ACH_CANCELED();

    /** Return ach_get_opts constant of same name */
    public static native int ACH_O_WAIT();
    /** Return ach_get_opts constant of same name */
    public static native int ACH_O_LAST();
    /** Return ach_get_opts constant of same name */
    public static native int ACH_O_COPY();

    /* Static variables for constants */

    /** Constant field for ach_status constant of same name */
    public static final int OK = ACH_OK();
    /** Constant field for ach_status constant of same name */
    public static final int OVERFLOW = ACH_OVERFLOW();
    /** Constant field for ach_status constant of same name */
    public static final int INVALID_NAME = ACH_INVALID_NAME();
    /** Constant field for ach_status constant of same name */
    public static final int BAD_SHM_FILE = ACH_BAD_SHM_FILE();
    /** Constant field for ach_status constant of same name */
    public static final int FAILED_SYSCALL = ACH_FAILED_SYSCALL();
    /** Constant field for ach_status constant of same name */
    public static final int STALE_FRAMES = ACH_STALE_FRAMES();
    /** Constant field for ach_status constant of same name */
    public static final int MISSED_FRAME = ACH_MISSED_FRAME();
    /** Constant field for ach_status constant of same name */
    public static final int TIMEOUT = ACH_TIMEOUT();
    /** Constant field for ach_status constant of same name */
    public static final int EEXIST = ACH_EEXIST();
    /** Constant field for ach_status constant of same name */
    public static final int ENOENT = ACH_ENOENT();
    /** Constant field for ach_status constant of same name */
    public static final int CLOSED = ACH_CLOSED();
    /** Constant field for ach_status constant of same name */
    public static final int BUG = ACH_BUG();
    /** Constant field for ach_status constant of same name */
    public static final int EINVAL = ACH_EINVAL();
    /** Constant field for ach_status constant of same name */
    public static final int CORRUPT = ACH_CORRUPT();
    /** Constant field for ach_status constant of same name */
    public static final int BAD_HEADER = ACH_BAD_HEADER();
    /** Constant field for ach_status constant of same name */
    public static final int EACCES = ACH_EACCES();
    /** Constant field for ach_status constant of same name */
    public static final int CANCELED = ACH_CANCELED();


    /** Bid mask for ach_status constant of same name */
    public static final int MASK_OK = 1<<OK;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_OVERFLOW = 1<<OVERFLOW;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_INVALID_NAME = 1<<INVALID_NAME;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_BAD_SHM_FILE = 1<<BAD_SHM_FILE;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_FAILED_SYSCALL = 1<<FAILED_SYSCALL;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_STALE_FRAMES = 1<<STALE_FRAMES;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_MISSED_FRAME = 1<<MISSED_FRAME;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_TIMEOUT = 1<<TIMEOUT;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_EEXIST = 1<<EEXIST;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_ENOENT = 1<<ENOENT;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_CLOSED = 1<<CLOSED;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_BUG = 1<<BUG;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_EINVAL = 1<<EINVAL;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_CORRUPT = 1<<CORRUPT;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_BAD_HEADER = 1<<BAD_HEADER;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_EACCES = 1<<EACCES;
    /** Bid mask for ach_status constant of same name */
    public static final int MASK_CANCELED = 1<<CANCELED;

    /** Constant field for ach_get_opts constant of same name */
    public static final int O_WAIT = ACH_O_WAIT();
    /** Constant field for ach_get_opts constant of same name */
    public static final int O_LAST = ACH_O_LAST();
    /** Constant field for ach_get_opts constant of same name */
    public static final int O_COPY = ACH_O_COPY();
}






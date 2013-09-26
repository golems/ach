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
class Lib
{

    /* Native Function Wrappers */

    public static native int open(String channel_name, long[] chan_ptr);
    public static native int close(long chan_ptr);
    public static native int flush(long chan_ptr);
    public static native int put(long chan_ptr, byte[] buf);
    public static native int get(long chan_ptr, byte[] buf, long[] frame_size, int options);
    public static native int get(long chan_ptr, byte[] buf, long[] frame_size, long sec, int nsec, int options);
    public static native int time( long[] abstime );

    /* Constant accessor functions */

    public static native int ACH_OK();
    public static native int ACH_OVERFLOW();
    public static native int ACH_INVALID_NAME();
    public static native int ACH_BAD_SHM_FILE();
    public static native int ACH_FAILED_SYSCALL();
    public static native int ACH_STALE_FRAMES();
    public static native int ACH_MISSED_FRAME();
    public static native int ACH_TIMEOUT();
    public static native int ACH_EEXIST();
    public static native int ACH_ENOENT();
    public static native int ACH_CLOSED();
    public static native int ACH_BUG();
    public static native int ACH_EINVAL();
    public static native int ACH_CORRUPT();
    public static native int ACH_BAD_HEADER();
    public static native int ACH_EACCES();
    public static native int ACH_CANCELED();

    public static native int ACH_O_WAIT();
    public static native int ACH_O_LAST();
    public static native int ACH_O_COPY();

    /* Static variables for constants */

    public static final int OK = ACH_OK();
    public static final int OVERFLOW = ACH_OVERFLOW();
    public static final int INVALID_NAME = ACH_INVALID_NAME();
    public static final int BAD_SHM_FILE = ACH_BAD_SHM_FILE();
    public static final int FAILED_SYSCALL = ACH_FAILED_SYSCALL();
    public static final int STALE_FRAMES = ACH_STALE_FRAMES();
    public static final int MISSED_FRAME = ACH_MISSED_FRAME();
    public static final int TIMEOUT = ACH_TIMEOUT();
    public static final int EEXIST = ACH_EEXIST();
    public static final int ENOENT = ACH_ENOENT();
    public static final int CLOSED = ACH_CLOSED();
    public static final int BUG = ACH_BUG();
    public static final int EINVAL = ACH_EINVAL();
    public static final int CORRUPT = ACH_CORRUPT();
    public static final int BAD_HEADER = ACH_BAD_HEADER();
    public static final int EACCES = ACH_EACCES();
    public static final int CANCELED = ACH_CANCELED();

    public static final int O_WAIT = ACH_O_WAIT();
    public static final int O_LAST = ACH_O_LAST();
    public static final int O_COPY = ACH_O_COPY();

    static{
        System.loadLibrary("libachj");
        System.loadLibrary("libachj");
    }
}






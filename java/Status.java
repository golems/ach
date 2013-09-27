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


class Status
{

    /** Exception base class */
    public static class AchException extends Exception {
        public int status() {return -1;}
    };

    public static class OK extends AchException {
        public int status() {return Lib.OK;}
    };
    public static class OVERFLOW extends AchException {
        public int status() {return Lib.OVERFLOW;}
    };
    public static class INVALID_NAME extends AchException {
        public int status() {return Lib.INVALID_NAME;}
    };
    public static class BAD_SHM_FILE extends AchException {
        public int status() {return Lib.BAD_SHM_FILE;}
    };
    public static class FAILED_SYSCALL extends AchException {
        public int status() {return Lib.FAILED_SYSCALL;}
    };
    public static class STALE_FRAMES extends AchException {
        public int status() {return Lib.STALE_FRAMES;}
    };
    public static class MISSED_FRAME extends AchException {
        public int status() {return Lib.MISSED_FRAME;}
    };
    public static class TIMEOUT extends AchException {
        public int status() {return Lib.TIMEOUT;}
    };
    public static class EEXIST extends AchException {
        public int status() {return Lib.EEXIST;}
    };
    public static class ENOENT extends AchException {
        public int status() {return Lib.ENOENT;}
    };
    public static class CLOSED extends AchException {
        public int status() {return Lib.CLOSED;}
    };
    public static class BUG extends AchException {
        public int status() {return Lib.BUG;}
    };
    public static class EINVAL extends AchException {
        public int status() {return Lib.EINVAL;}
    };
    public static class CORRUPT extends AchException {
        public int status() {return Lib.CORRUPT;}
    };
    public static class BAD_HEADER extends AchException {
        public int status() {return Lib.BAD_HEADER;}
    };
    public static class EACCES extends AchException {
        public int status() {return Lib.EACCES;}
    };
    public static class CANCELED extends AchException {
        public int status() {return Lib.CANCELED;}
    };

    /** Create an exception object for the given status code */
    public static AchException create_exception(int status) {
        if( Lib.OK == status ) return new OK();
        if( Lib.OVERFLOW == status ) return new OVERFLOW();
        if( Lib.INVALID_NAME == status ) return new INVALID_NAME();
        if( Lib.BAD_SHM_FILE == status ) return new BAD_SHM_FILE();
        if( Lib.FAILED_SYSCALL == status ) return new FAILED_SYSCALL();
        if( Lib.STALE_FRAMES == status ) return new STALE_FRAMES();
        if( Lib.MISSED_FRAME == status ) return new MISSED_FRAME();
        if( Lib.TIMEOUT == status ) return new TIMEOUT();
        if( Lib.EEXIST == status ) return new EEXIST();
        if( Lib.ENOENT == status ) return new ENOENT();
        if( Lib.CLOSED == status ) return new CLOSED();
        if( Lib.BUG == status ) return new BUG();
        if( Lib.EINVAL == status ) return new EINVAL();
        if( Lib.CORRUPT == status ) return new CORRUPT();
        if( Lib.BAD_HEADER == status ) return new BAD_HEADER();
        if( Lib.EACCES == status ) return new EACCES();
        if( Lib.CANCELED == status ) return new CANCELED();
        return null;
    }

    /** Throw an exception for the given status code */
    public static void throw_exception(int status) throws AchException {
        throw create_exception(status);
    }

    /** Throw an exception if status code does not match any true mask bits */
    public static void maybe_throw(int status, int mask) throws AchException {
        if( Lib.OK != status &&
            0 != ( (1 << status) & ~mask)  )
        {
            throw create_exception(status);
        }
    }
}







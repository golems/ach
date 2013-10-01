/* Copyright (c) 2013, Matt Zucker
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Matt Zucker <mzucker1@swarthmore.edu>
 *            Neil T. Dantam <ntd@gatech.edu>
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

#ifndef _ACH_HPP_
#define _ACH_HPP_

#include <ach.h>
#include <vector>

#define ACH_MASK_FROM_STATUS(r) (1<<(r))


enum ach_mask_t {

    ACH_MASK_OK             = ACH_MASK_FROM_STATUS(ACH_OK),
    ACH_MASK_OVERFLOW       = ACH_MASK_FROM_STATUS(ACH_OVERFLOW),
    ACH_MASK_INVALID_NAME   = ACH_MASK_FROM_STATUS(ACH_INVALID_NAME),
    ACH_MASK_BAD_SHM_FILE   = ACH_MASK_FROM_STATUS(ACH_BAD_SHM_FILE),
    ACH_MASK_FAILED_SYSCALL = ACH_MASK_FROM_STATUS(ACH_FAILED_SYSCALL),
    ACH_MASK_STALE_FRAMES   = ACH_MASK_FROM_STATUS(ACH_STALE_FRAMES),
    ACH_MASK_MISSED_FRAME   = ACH_MASK_FROM_STATUS(ACH_MISSED_FRAME),
    ACH_MASK_TIMEOUT        = ACH_MASK_FROM_STATUS(ACH_TIMEOUT),
    ACH_MASK_EEXIST         = ACH_MASK_FROM_STATUS(ACH_EEXIST),
    ACH_MASK_ENOENT         = ACH_MASK_FROM_STATUS(ACH_ENOENT),
    ACH_MASK_CLOSED         = ACH_MASK_FROM_STATUS(ACH_CLOSED),
    ACH_MASK_BUG            = ACH_MASK_FROM_STATUS(ACH_BUG),
    ACH_MASK_EINVAL         = ACH_MASK_FROM_STATUS(ACH_EINVAL),
    ACH_MASK_CORRUPT        = ACH_MASK_FROM_STATUS(ACH_CORRUPT),
    ACH_MASK_BAD_HEADER     = ACH_MASK_FROM_STATUS(ACH_BAD_HEADER),
    ACH_MASK_EACCES         = ACH_MASK_FROM_STATUS(ACH_EACCES),

    ACH_MASK_NONE           = 0,
    ACH_MASK_ALL            = 0xffffffff

};

namespace ach {


/** Base class for Ach channels.
 *
 * Errors are handled through the warn_* and error_* virtual
 * functions.  When the mask value for an ach_status code does not
 * match the allow_mask, then one of the error_* or warn_* handlers is
 * called.  The warn_* handler is called when the ach_status mask
 * matches a bit in the warn_mask.  Otherwise, the error_* handler is
 * called.
 *
 * The error handlers for this base class are no-ops.
 *
 * \see LogChannel, CerrChannel, SyslogChannel
 */
class Channel {
public:
    ach_channel_t channel;

    /** Do-nothing constructor. */
    Channel();

    /** Destructor closes channel if still open. */
    virtual ~Channel();


    /** Open the channel. */
    ach_status_t open(const char* channel_name,
                      ach_attr_t* attr = NULL,
                      int allow_mask=ACH_MASK_OK,
                      int warn_mask=ACH_MASK_NONE);

    /** Close the channel. */
    ach_status_t close(int allow_mask=ACH_MASK_OK,
                       int warn_mask=ACH_MASK_NONE);

    /** Put frame to channel. */
    ach_status_t put(const void* buf,
                     size_t len,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE);

    /** Get frame from channel into an STL vector.
     *
     * This function will resize vec if it is too small.  Note that
     * this will malloc, so it is best ensure vec is sufficiently
     * large before beginning a real-time loop.
     */
    ach_status_t get(::std::vector<uint8_t> *vec, size_t offset,
                     size_t* frame_size,
                     const struct timespec *ACH_RESTRICT abstime = NULL,
                     int options = 0,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE);


    /** Get frame from channel. */
    ach_status_t get(void* buf, size_t size,
                     size_t* frame_size,
                     const struct timespec *ACH_RESTRICT abstime = NULL,
                     int options = 0,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE);


    /** Ignore old messages in channel. */
    ach_status_t flush(int allow_mask=ACH_MASK_OK,
                       int warn_mask=ACH_MASK_NONE);

protected:

    virtual void warn_open( ach_status_t status );
    virtual void error_open( ach_status_t status );

    virtual void warn_close( ach_status_t status );
    virtual void error_close( ach_status_t status );

    virtual void warn_put( ach_status_t status );
    virtual void error_put( ach_status_t status );

    virtual void warn_get( ach_status_t status );
    virtual void error_get( ach_status_t status );

    virtual void warn_flush( ach_status_t status );
    virtual void error_flush( ach_status_t status );
};

/** Abstract base class for Ach channels that log errors */
class LogChannel : public Channel{
protected:
    virtual void warn( const char *method, ach_status_t status ) = 0;
    virtual void error( const char *method, ach_status_t status ) = 0;

    virtual void warn_open( ach_status_t status );
    virtual void error_open( ach_status_t status );

    virtual void warn_close( ach_status_t status );
    virtual void error_close( ach_status_t status );

    virtual void warn_put( ach_status_t status );
    virtual void error_put( ach_status_t status );

    virtual void warn_get( ach_status_t status );
    virtual void error_get( ach_status_t status );

    virtual void warn_flush( ach_status_t status );
    virtual void error_flush( ach_status_t status );
};

/** Ach channel that logs errors to stderr.
 *
 * Errors will terminate the process.  Warnings will not.
 */
class CerrChannel : public LogChannel {
    virtual void warn( const char *method, ach_status_t status );
    virtual void error( const char *method, ach_status_t status );

};

/** Ach channel that logs errors to syslog
 *
 * Errors will terminate the process.  Warnings will not.
 */
class SyslogChannel : public LogChannel {
    virtual void warn( const char *method, ach_status_t status );
    virtual void error( const char *method, ach_status_t status );

};

}

#endif

/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* indent-tabs-mode:  nil                    */
/* c-basic-offset: 4                         */
/* c-file-offsets: ((innamespace . 0))       */
/* End:                                      */

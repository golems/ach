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

/** \file Ach.hpp
 *
 * \brief This file contains a C++ convenience interface for ach.
 *
 * \author Matt Zucker, Neil T. Dantam
 */

#include <ach.h>
#include <vector>
#include <iostream>
#include <cstdlib>


namespace ach {


enum status_check_result {
    STATUS_OK,
    STATUS_WARN,
    STATUS_ERR
};


static int check_status( ach_status_t result,
                         int allow_mask,
                         int warn_mask) {

    int r = ACH_STATUS_MASK(result);
    int x;
    if (!(allow_mask & r)) {
        if (warn_mask & r) return STATUS_WARN;
        else return STATUS_ERR;
    } else {
        return STATUS_OK;
    }
}

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
    Channel() {}

    /** Destructor closes channel if still open. */
    virtual ~Channel() {
        if (this->channel.shm) { close(); }
    }

    enum ach_status dispatch( enum ach_status r,
                              int allow_mask,
                              int warn_mask,
                              void (Channel::*warn)(enum ach_status),
                              void (Channel::*error)(enum ach_status) ) {
        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: (*this.*warn)(r);  break;
        case STATUS_ERR:  (*this.*error)(r); break;
        case STATUS_OK: break;
        }
        return r;
    }

    /** Open the channel. */
    ach_status_t open(const char* channel_name,
                      ach_attr_t* attr = NULL,
                      int allow_mask=ACH_MASK_OK,
                      int warn_mask=ACH_MASK_NONE) {
        ach_status_t r = ach_open(&channel, channel_name, attr);

        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: warn_open(r);  break;
        case STATUS_ERR:  error_open(r); break;
        case STATUS_OK: break;
        }

        return r;
    }

    /** Close the channel. */
    ach_status_t close(int allow_mask=ACH_MASK_OK,
                       int warn_mask=ACH_MASK_NONE) {
        ach_status_t r = ach_close(&channel);

        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: warn_close(r);  break;
        case STATUS_ERR:  error_close(r); break;
        case STATUS_OK: break;
        }

        return r;
    }

    /** Put frame to channel. */
    ach_status_t put(const void* buf,
                     size_t len,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE) {
        ach_status_t r = ach_put(&channel, buf, len);

        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: warn_put(r);  break;
        case STATUS_ERR:  error_put(r); break;
        case STATUS_OK: break;
        }

        return r;
    }

    /** Get frame from channel into an STL vector.
     *
     * This function will resize vec if it is too small.  Note that
     * this will malloc, so it is best ensure vec is sufficiently
     * large before beginning a real-time loop.
     */
    ach_status_t get(::std::vector<uint8_t> *buf, size_t offset,
                     size_t* frame_size,
                     const struct timespec *ACH_RESTRICT abstime = NULL,
                     int options = 0,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE) {
        size_t fs = 0;
        while(1) {
            buf->reserve( offset + fs );
            uint8_t *ptr = &((*buf)[0]);
            size_t size = buf->capacity() - offset;
            if( size > buf->capacity() ) size = 0; // check overflow
            ach_status_t r = this->get( ptr, size, &fs,
                                        abstime, options,
                                        allow_mask | ACH_MASK_OVERFLOW, warn_mask );
            if( ACH_OVERFLOW != r ) {
                if( frame_size ) *frame_size = fs;
                return r;
            }
        }
    }


    /** Get frame from channel. */
    ach_status_t get(void* buf, size_t size,
                     size_t* frame_size,
                     const struct timespec *ACH_RESTRICT abstime = NULL,
                     int options = 0,
                     int allow_mask=ACH_MASK_OK,
                     int warn_mask=ACH_MASK_NONE) {
        ach_status_t r = ach_get(&channel, buf, size, frame_size, abstime, options);

        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: warn_get(r);  break;
        case STATUS_ERR:  error_get(r); break;
        case STATUS_OK: break;
        }

        return r;
    }


    /** Ignore old messages in channel. */
    ach_status_t flush(int allow_mask=ACH_MASK_OK,
                       int warn_mask=ACH_MASK_NONE) {
        ach_status_t r = ach_flush(&channel);

        switch(check_status(r, allow_mask, warn_mask)) {
        case STATUS_WARN: warn_flush(r);  break;
        case STATUS_ERR:  error_flush(r); break;
        case STATUS_OK: break;
        }

        return r;
    }


protected:

    virtual void warn_open( ach_status_t status ) {}
    virtual void error_open( ach_status_t status ) {}

    virtual void warn_close( ach_status_t status ) {}
    virtual void error_close( ach_status_t status ) {}

    virtual void warn_put( ach_status_t status ) {}
    virtual void error_put( ach_status_t status ) {}

    virtual void warn_get( ach_status_t status ) {}
    virtual void error_get( ach_status_t status ) {}

    virtual void warn_flush( ach_status_t status ) {}
    virtual void error_flush( ach_status_t status ) {}
};

/** Abstract base class for Ach channels that log errors */
class LogChannel : public Channel{
protected:
    virtual void warn( const char *method, ach_status_t status ) = 0;
    virtual void error( const char *method, ach_status_t status ) = 0;

    virtual void warn_open( ach_status_t status )  { warn("ach_open",status); }
    virtual void error_open( ach_status_t status ) { error("ach_open",status); }

    virtual void warn_close( ach_status_t status ) { warn("ach_close",status); }
    virtual void error_close( ach_status_t status) { error("ach_close",status); }

    virtual void warn_put( ach_status_t status ) { warn("ach_put",status); }
    virtual void error_put( ach_status_t status) { error("ach_put",status); }

    virtual void warn_get( ach_status_t status )  { warn("ach_get",status); }
    virtual void error_get( ach_status_t status ) { error("ach_get",status); }

    virtual void warn_flush( ach_status_t status )  { warn("ach_flush",status); }
    virtual void error_flush( ach_status_t status ) { error("ach_flush",status); }
};

/** Ach channel that logs errors to stderr.
 *
 * Errors will terminate the process.  Warnings will not.
 */
class CerrChannel : public LogChannel {
    virtual void warn( const char *method, ach_status_t status ) {
        std::cerr << "warning: " << method << " "
                  << ach_result_to_string(status)
                  << " (" << status << ")" << std::endl;
    }
    virtual void error( const char *method, ach_status_t status ) {
        std::cerr << "error: " << method << " "
                  << ach_result_to_string(status)
                  << " (" << status << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
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

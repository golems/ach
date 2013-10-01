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

#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include "Ach.hpp"
#include <syslog.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>

using namespace ach;

enum status_check_result {
    STATUS_OK,
    STATUS_WARN,
    STATUS_ERR
};

static int check_status( ach_status_t result,
                         int allow_mask,
                         int warn_mask) {

    int r = ACH_MASK_FROM_STATUS(result);
    int x;
    if (!(allow_mask & r)) {
        if (warn_mask & r) return STATUS_WARN;
        else return STATUS_ERR;
    } else {
        return STATUS_OK;
    }
}


Channel::Channel() {}

Channel::~Channel() {
    if (this->channel.shm) { close(); }
}

ach_status_t Channel::open(const char* channel_name,
                           ach_attr_t* attr,
                           int allow_mask,
                           int warn_mask) {

    ach_status_t r = ach_open(&channel, channel_name, attr);

    switch(check_status(r, allow_mask, warn_mask)) {
    case STATUS_WARN: warn_open(r);  break;
    case STATUS_ERR:  error_open(r); break;
    case STATUS_OK: break;
    }

    return r;

}

ach_status_t Channel::close(int allow_mask,
                            int warn_mask) {

    ach_status_t r = ach_close(&channel);

    switch(check_status(r, allow_mask, warn_mask)) {
    case STATUS_WARN: warn_close(r);  break;
    case STATUS_ERR:  error_close(r); break;
    case STATUS_OK: break;
    }

    return r;

}

ach_status_t Channel::flush(int allow_mask,
                            int warn_mask) {

    ach_status_t r = ach_flush(&channel);

    switch(check_status(r, allow_mask, warn_mask)) {
    case STATUS_WARN: warn_flush(r);  break;
    case STATUS_ERR:  error_flush(r); break;
    case STATUS_OK: break;
    }

    return r;

}

ach_status_t Channel::get(void* buf,
                          size_t size,
                          size_t* frame_size,
                          const struct timespec *ACH_RESTRICT abstime,
                          int options,
                          int allow_mask,
                          int warn_mask) {

    ach_status_t r = ach_get(&channel, buf, size, frame_size, abstime, options);

    switch(check_status(r, allow_mask, warn_mask)) {
    case STATUS_WARN: warn_get(r);  break;
    case STATUS_ERR:  error_get(r); break;
    case STATUS_OK: break;
    }

    return r;

}


ach_status_t Channel::get(::std::vector<uint8_t> *buf, size_t offset,
                          size_t* frame_size,
                          const struct timespec *ACH_RESTRICT abstime,
                          int options,
                          int allow_mask,
                          int warn_mask) {
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

ach_status_t Channel::put(void* buf,
                          size_t len,
                          int allow_mask,
                          int warn_mask) {

    ach_status_t r = ach_put(&channel, buf, len);

    switch(check_status(r, allow_mask, warn_mask)) {
    case STATUS_WARN: warn_put(r);  break;
    case STATUS_ERR:  error_put(r); break;
    case STATUS_OK: break;
    }

    return r;
}

void Channel::warn_open( ach_status_t r ) {}
void Channel::error_open( ach_status_t r ) {}

void Channel::warn_close( ach_status_t r ) {}
void Channel::error_close( ach_status_t r ) {}

void Channel::warn_get( ach_status_t r ) {}
void Channel::error_get( ach_status_t r ) {}

void Channel::warn_put( ach_status_t r ) {}
void Channel::error_put( ach_status_t r ) {}

void Channel::warn_flush( ach_status_t r ) {}
void Channel::error_flush( ach_status_t r ) {}


void LogChannel::warn_open( ach_status_t r ) { warn("ach_open",r); }
void LogChannel::error_open( ach_status_t r ) { error("ach_open",r); }

void LogChannel::warn_close( ach_status_t r ) { warn("ach_close",r); }
void LogChannel::error_close( ach_status_t r ) { error("ach_close",r); }

void LogChannel::warn_get( ach_status_t r ) { warn("ach_get",r); }
void LogChannel::error_get( ach_status_t r ) { error("ach_get",r); }

void LogChannel::warn_put( ach_status_t r ) { warn("ach_put",r); }
void LogChannel::error_put( ach_status_t r ) { error("ach_put",r); }

void LogChannel::warn_flush( ach_status_t r ) { warn("ach_flush",r); }
void LogChannel::error_flush( ach_status_t r ) { error("ach_flush",r); }


void CerrChannel::warn( const char *method, ach_status_t r ) {
    std::cerr << "warning: " << method << " "
              << ach_result_to_string(r)
              << " (" << r << ")" << std::endl;
}
void CerrChannel::error( const char *method, ach_status_t r ) {
    std::cerr << "error: " << method << " "
              << ach_result_to_string(r)
              << " (" << r << ")" << std::endl;
    exit(EXIT_FAILURE);
}


void SyslogChannel::warn( const char *method, ach_status_t r ) {
    syslog(LOG_WARNING, "warning: %s, %s (%d)\n", method, ach_result_to_string(r), r );
}
void SyslogChannel::error( const char *method, ach_status_t r ) {
    syslog(LOG_ERR, "error: %s, %s (%d)\n", method, ach_result_to_string(r), r );
    exit(EXIT_FAILURE);
}

/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* indent-tabs-mode:  nil                    */
/* c-basic-offset: 4                         */
/* c-file-offsets: ((innamespace . 0))       */
/* End:                                      */

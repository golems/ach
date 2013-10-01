/* Copyright (c) 2013, Matt Zucker
 * All rights reserved.
 *
 * Author(s): Matt Zucker <mzucker1@swarthmore.edu>
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
#include <string>

#define ACH_MASK_FROM_STATUS(r) (1<<(r))

enum ach_allowed_t {

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

typedef void (*ach_handler_t)(const std::string&, ach_status_t);

class AchChannel {
public:

    std::string display_name;

    ach_channel_t channel;
    bool is_open;

    ach_status_t result;

    ach_handler_t warning_handler;
    ach_handler_t error_handler;

    AchChannel(const std::string& display_name="");

    ~AchChannel();

    void check_status(const std::string& context,
                      ach_status_t result,
                      uint32_t allow_mask,
                      uint32_t warn_mask);

    ach_status_t open(const char* channel_name,
                      ach_attr_t* attr,
                      uint32_t allow_mask=ACH_MASK_OK,
                      uint32_t warn_mask=ACH_MASK_NONE);

    ach_status_t get(void* buf, size_t size,
                     size_t* frame_size,
                     const struct timespec *ACH_RESTRICT abstime,
                     int options,
                     uint32_t allow_mask=ACH_MASK_OK,
                     uint32_t warn_mask=ACH_MASK_NONE);

    ach_status_t put(void* buf,
                     size_t len,
                     uint32_t allow_mask=ACH_MASK_OK,
                     uint32_t warn_mask=ACH_MASK_NONE);

    ach_status_t close(uint32_t allow_mask=ACH_MASK_OK,
                       uint32_t warn_mask=ACH_MASK_NONE);

    ach_status_t flush(uint32_t allow_mask=ACH_MASK_OK,
                       uint32_t warn_mask=ACH_MASK_NONE);

};

#endif

/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */

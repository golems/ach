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

#include "Ach.hpp"
#include <iostream>
#include <stdlib.h>

static void warning(const std::string& context,
                    ach_status_t result) {

    std::cerr << context << ": warning: "
              << "result was " << ach_result_to_string(result) << "\n";

}

static void error(const std::string& context,
                  ach_status_t result) {

    std::cerr << context << ": error: "
              << "result was " << ach_result_to_string(result) << "\n";

    exit(1);

}

AchChannel::AchChannel(const std::string& dn):
    display_name(dn),
    is_open(false),
    result(ACH_OK),
    warning_handler(0),
    error_handler(0) {}

AchChannel::~AchChannel() {
    if (is_open) { close(); }
}

ach_status_t AchChannel::close(uint32_t allow_mask,
                               uint32_t warn_mask) {

    result = ach_close(&channel);

    check_status("calling ach_close on " + display_name,
                 result, allow_mask, warn_mask);

    if (result == ACH_OK) {
        is_open = false;
    }

    return result;

}

ach_status_t AchChannel::flush(uint32_t allow_mask,
                               uint32_t warn_mask) {

    result = ach_flush(&channel);

    check_status("calling ach_flush on " + display_name,
                 result, allow_mask, warn_mask);

    return result;

}

ach_status_t AchChannel::open(const char* channel_name,
                              ach_attr_t* attr,
                              uint32_t allow_mask,
                              uint32_t warn_mask) {

    result = ach_open(&channel, channel_name, attr);

    check_status("calling ach_open on " + display_name,
                 result, allow_mask, warn_mask);

    if (result == ACH_OK) {
        is_open = true;
        if (display_name.empty()) {
            display_name = channel_name;
        }
    }

    return result;

}

ach_status_t AchChannel::get(void* buf,
                             size_t size,
                             size_t* frame_size,
                             const struct timespec *ACH_RESTRICT abstime,
                             int options,
                             uint32_t allow_mask,
                             uint32_t warn_mask) {

    result = ach_get(&channel, buf, size, frame_size, abstime, options);

    check_status("calling ach_get on " + display_name,
                 result, allow_mask, warn_mask);

    return result;

}

ach_status_t AchChannel::put(void* buf,
                             size_t len,
                             uint32_t allow_mask,
                             uint32_t warn_mask) {

    result = ach_put(&channel, buf, len);

    check_status("calling ach_put on " + display_name,
                 result, allow_mask, warn_mask);

    return result;

}

void AchChannel::check_status(const std::string& context,
                              ach_status_t result,
                              uint32_t allow_mask,
                              uint32_t warn_mask) {

    uint32_t r = ACH_MASK_FROM_STATUS(result);

    if (!(allow_mask & r)) {
        if (warn_mask & r) {
            ach_handler_t w = warning_handler ? warning_handler : warning;
            w(context, result);
        } else {
            ach_handler_t e = error_handler ? error_handler : error;
            e(context, result);
        }
    }

}

/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */

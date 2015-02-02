/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * Copyright (C) 2015, Rice University
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

/** \file experimental.h
 *
 *  \brief This header file defines experimental public interfaces for Ach.
 *
 *  \author Neil T. Dantam
 *
 */


#ifndef ACH_EXPERIMENTAL_H
#define ACH_EXPERIMENTAL_H



#ifdef __cplusplus
extern "C" {
#endif

/** Control structure for event handling loop
 */
struct ach_evhandler {
    /** Channel to get messages from
     */
    struct ach_channel *channel;

    /** Context argument for handler
     */
    void *context;

    /** Handler function.
     *
     *  Called whenever there is new data in the channel.
     *
     *  Handler should return ACH_OK when new frames are read, or
     *  ACH_STALE_FRAMES if no new frames are read.  Any other
     *  return value will terminate the event loop.
     */
    enum ach_status (*handler)
    ( void *context, struct ach_channel *channel );
};

/* Options for event handler */

/** Execute the periodic function after the period expires.
 */
#define ACH_EV_O_PERIODIC_TIMEOUT 0x01

/** Execute the periodic function every time new messages are received.
 */
#define ACH_EV_O_PERIODIC_INPUT 0x02


/** Event loop for handling multiple channels.
 *
 *  \param[in,out]              handlers array of handler descriptors
 *
 *  \param[in] n                size of handlers array
 *
 *  \param[in] period           timeout to wait between execution of periodic
 *                              function when requested in flags.
 *
 *  \param[in] periodic_handler function to executre periodicly,
 *                              i.e., when timeout occurs or when
 *                              new messages are received.
 *
 *  \param[in] periodic_context context argument to the periodic_handler
 *
 * \param[in] options           bit flags, may include
 *                              ACH_EV_O_PERIODIC_INPUT and
 *                              ACH_EV_O_PERIODIC_TIMEOUT
 */
enum ach_status ACH_WARN_UNUSED
ach_evhandle( struct ach_evhandler *handlers,
              size_t n,
              const struct timespec *period,
              enum ach_status (*periodic_handler)(void *context),
              void *periodic_context,
              int options );

#ifdef __cplusplus
}
#endif

#endif /* ACH_H */

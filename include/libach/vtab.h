/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
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

/** \file vtab.h.h
 *  \author Neil T. Dantam
 */

#ifndef ACHLIB_VTAB_H
#define ACHLIB_VTAB_H

#ifdef __cplusplus
extern "C" {
#endif

/** Vtable for different channel mapping implementations */
struct ach_channel_vtab {

    /* Implementation of ach_create() */
    enum ach_status
    (*create)( const char *channel_name,
               size_t frame_cnt, size_t frame_size,
               ach_create_attr_t *attr);

    /* Implementation of ach_open() */
    enum ach_status
    (*open)( ach_channel_t *chan, const char *channel_name,
             ach_attr_t *attr );

    /** Implementation of ach_flush() */
    enum ach_status
    (*flush)(ach_channel_t*);

    /** Implementation of ach_put() */
    enum ach_status
    (*put)(ach_channel_t*,const void *buf, size_t len);

    /** Implementation of ach_flush() */
    enum ach_status
    (*get)( ach_channel_t *chan, void *buf, size_t size,
            size_t *frame_size,
            const struct timespec *ACH_RESTRICT abstime,
            int options );

    /** Implementation of ach_cancel() */
    enum ach_status
    (*cancel)( ach_channel_t *chan, const ach_cancel_attr_t *attr );

    /** Implementation of ach_close() */
    enum ach_status
    (*close)( ach_channel_t *chan );

    /** Implementation of ach_unlink() */
    enum ach_status
    (*unlink)( const char *name );

    /** Does the channel exist? */
    enum ach_status
    (*exists)( const char *name );

    /** Get the full filename. */
    enum ach_status
    (*filename)( const char *name, char *buf, size_t n );

    /** The mapping for this vtab. */
    enum ach_map map;
};


/** Virtual method table for POSIX shm backed channels */
extern const struct ach_channel_vtab libach_vtab_user;

/** Virtual method table for heap backed channels */
extern const struct ach_channel_vtab libach_vtab_anon;

/** Virtual method table for kernel backed channels */
extern const struct ach_channel_vtab libach_vtab_klinux;


#ifdef __cplusplus
}
#endif

#endif

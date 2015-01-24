/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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


#ifndef LIBACH_PRIVATE_H
#define LIBACH_PRIVATE_H


/** \file libach_private.h
 *
 * \brief This file decalres helper functions and variables to be used
 *        by language bindings; it is used by the Python ctypes
 *        interface.
 *
 *  \author Neil T. Dantam
 */

#ifdef __cplusplus
extern "C" {
#endif


ach_channel_t *ach_channel_alloc(void);
void  ach_channel_free( ach_channel_t *);

extern const int ach_ok;
extern const int ach_overflow;
extern const int ach_invalid_name;
extern const int ach_bad_shm_file;
extern const int ach_failed_syscall;
extern const int ach_stale_frames;
extern const int ach_missed_frame;
extern const int ach_timeout;
extern const int ach_eexist;
extern const int ach_enoent;
extern const int ach_closed;
extern const int ach_bug;
extern const int ach_einval;
extern const int ach_corrupt;
extern const int ach_bad_header;
extern const int ach_eacces;

extern const int ach_o_wait;
extern const int ach_o_last;


/** Size of ach_channel_t */
extern size_t ach_channel_size;
/** Size of ach_attr_t */
extern size_t ach_attr_size;

#ifdef __cplusplus
}
#endif

#endif //LIBACH_PRIVATE_H

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
 *   * Neither the name of the copyright holder nor the names of its
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


void fail_errno( const char *thing );
void fail_ach( const char *thing, enum ach_status r );

void check_ach(const char *thing, ach_status_t r );
void check_errno( const char *thing, int r);

void check_ach(const char *thing, ach_status_t r );

#define CHECK_ACH( thing, exp )                                         \
    {                                                                   \
        enum ach_status check_ach_result = (exp);                       \
        if( ACH_OK != check_ach_result ) {                              \
            fprintf(stderr,                                             \
                    "FAILURE:\t%s: %s, at %s:%d\n"                      \
                    "expression:\t(" #exp ")\n",                        \
                    thing, ach_result_to_string(check_ach_result),      \
                    __FILE__, __LINE__ );                               \
            exit(EXIT_FAILURE);                                         \
        }                                                               \
    }

#define CHECK_TRUE( thing, exp )                                        \
    {                                                                   \
        if( !(exp) ) {                                                  \
            fprintf(stderr,                                             \
                    "FAILURE:\t%s, %s:%d\n"                             \
                    "expression:\t(" #exp ")\n",                        \
                    thing, __FILE__, __LINE__ );                        \
            exit(EXIT_FAILURE);                                         \
        }                                                               \
    }

#define CHECK_ACH_MASK( thing,  allow_mask, exp )                       \
    {                                                                   \
        enum ach_status check_ach_result = (exp);                       \
        if( ! ach_status_match(check_ach_result, allow_mask) )          \
        {                                                               \
            fprintf(stderr,                                             \
                    "FAILURE:\t%s: %s, at %s:%d\n"                      \
                    "expression:\t(" #exp ")\n",                        \
                    thing, ach_result_to_string(check_ach_result),      \
                    __FILE__, __LINE__ );                               \
            exit(EXIT_FAILURE);                                         \
        }                                                               \
    }

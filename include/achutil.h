/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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

#ifndef ACHUTIL_H
#define ACHUTIL_H

#include <signal.h>

/* Routines for ach utilities.  Not for external consumption */

void ach_print_version( const char *name );


#ifdef __GNUC__
#define ACH_ATTR_PRINTF(m,n) __attribute__((format(printf, m, n)))
#else
#define ACH_ATTR_PRINTF(m,n)
#endif

extern int ach_verbosity;

void ach_log( int level, const char fmt[], ...)          ACH_ATTR_PRINTF(2,3);

#define ACH_LOG( priority, ... ) \
    if((priority) <= LOG_NOTICE + ach_verbosity) ach_log((priority), __VA_ARGS__);

extern pid_t ach_pid_notify; /// tell this pid if we die or are ok
void ach_notify(int sig);    /// tell pid something
void ach_die(void);
#define ACH_DIE(...) {ACH_LOG(LOG_ERR,__VA_ARGS__); ach_die(); }


/*-- Signal Handling Helpers --*/
void ach_sig_mask( const int *sig, sigset_t *mask );

int ach_sig_wait( const int *sig );

/* Block Signal, then install dummy signal handler. */
void ach_sig_block_dummy( const int *sig );

/* Restore default handler and unblock the signal */
void ach_sig_dfl_unblock( const int *sig );

pid_t ach_detach( unsigned timeout );

/// Wait this long for notification from child
#define ACH_PARENT_TIMEOUT_SEC 3

/// If child exits with failure before this timeout, give up
#define ACH_CHILD_TIMEOUT_SEC 1

#define ACH_SIG_OK   SIGUSR1
#define ACH_SIG_FAIL SIGUSR2

#endif //ACHUTIL_H

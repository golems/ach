/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
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

/** \file achutil.h
 *
 *  \brief This file declares routines for ach utilities; not for
 *         external consumption.
 *
 *  \author Neil T. Dantam
 */


/** Print version and copyright info */
void ach_print_version( const char *name );


#ifdef __GNUC__
#define ACH_ATTR_PRINTF(m,n) __attribute__((format(printf, m, n)))
#else
#define ACH_ATTR_PRINTF(m,n)
#endif


/** Print to stderr and/or syslog
 * If stderr is a TTY, print to it.
 * If stderr is not a TTY or our parent is init, print to syslog.
 * (may print to both stderr and syslog)
 */
void ach_log( int level, const char fmt[], ...)          ACH_ATTR_PRINTF(2,3);

/** Verbosity of log output.
 *
 * At 0, any log priority greater than LOG_NOTICE is printed.
 */
extern int ach_verbosity;

/** Conditional logging macro.
 *
 * Logs if ach_verbosity sufficiently high.
 */
#define ACH_LOG( priority, ... ) \
    if((priority) <= LOG_NOTICE + ach_verbosity) ach_log((priority), __VA_ARGS__);

/** PID to notify of failure or success to start.
 *
 * This should probably be the PID returned by ach_detach() or the
 * parent PID if running under achcop.
 */
extern pid_t ach_pid_notify;

/** Signal ach_pid_notify if ach_pid_notify is a valid PID */
void ach_notify(int sig);

/** Signal ach_pid_notify that we failed, then exit */
void ach_die(void);

/** Log an error message, then die */
#define ACH_DIE(...) {ACH_LOG(LOG_ERR,__VA_ARGS__); ach_die(); }


/*-- Signal Handling Helpers --*/

#define ACH_SIG_OK   SIGUSR1
#define ACH_SIG_FAIL SIGUSR2

/* These operate on int strings (null terminated arrays) of signal
 * numbers.
 */

/* Empty mask, then add all signals in sig to mask */
void ach_sig_mask( const int *sig, sigset_t *mask );

/* wait for any signal in sig */
int ach_sig_wait( const int *sig );

/* Block Signal, then install dummy signal handler. */
void ach_sig_block_dummy( const int *sig );

/* Restore default handler and unblock the signal */
void ach_sig_dfl_unblock( const int *sig );


/** Detach process and run in background
 *
 *  Returns PID of the original process.
 *
 *  The original process exits with EXIT_SUCCESS if it receives
 *  ACH_SIG_OK (SIGUSR1) before timeout seconds elapse.  Otherwise, it
 *  exits with failure.
 */
pid_t ach_detach( unsigned timeout );

/** Wait this long for notification from child.
 * A default timeout for ach_detach
 */
#define ACH_PARENT_TIMEOUT_SEC 3

#endif //ACHUTIL_H

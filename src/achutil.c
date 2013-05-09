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


/** \file ach.c
 *  \author Neil T. Dantam
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/stat.h>

#include <string.h>
#include <inttypes.h>
#include <stdarg.h>
#include <syslog.h>

#include "ach.h"
#include "achutil.h"

int ach_verbosity = 0;

void ach_print_version( const char *name ) {
    printf( "%s " PACKAGE_VERSION "\n"
            "\n"
            "Copyright (c) 2008-2013, Georgia Tech Research Corporation\n"
            "This is free software; see the source for copying conditions.  There is NO\n"
            "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"
            "\n"
            "Written by Neil T. Dantam\n",
            name
        );
}

void ach_log( int level, const char fmt[], ...) {
    int tty = isatty(STDERR_FILENO);
    if( tty ) {
        va_list argp;
        va_start( argp, fmt );
        vfprintf(stderr, fmt, argp );
        fflush(stderr);
        va_end( argp );
    }
    if( !tty || 1 == getppid() ) {
        va_list argp;
        va_start( argp, fmt );
        vsyslog( level, fmt, argp);
        va_end( argp );
    }
}

sig_atomic_t ach_got_sigterm;
sig_atomic_t ach_got_sigint;
sig_atomic_t ach_got_sigchild;

static void ach_sigflag(int sig, siginfo_t *siginfo, void *context) {
    (void)siginfo; (void)context;
    //ACH_LOG( LOG_DEBUG, "Received signal: %d\n", sig );
    switch(sig) {
    case SIGTERM:
        ach_got_sigterm = 1;
        break;
    case SIGINT:
        ach_got_sigint = 1;
        break;
    case SIGCHLD:
        ach_got_sigchild++;
        break;
    default:
        /* This is unsafe, but maybe worth it to debug */
        ACH_LOG( LOG_WARNING, "Received unexpected signal: %s (%d)\n",
                 strsignal(sig), sig );
    }
}

void ach_install_sigflag( int sig ) {
    struct sigaction act;
    memset( &act, 0, sizeof(act) );

    act.sa_sigaction = &ach_sigflag;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(sig, &act, NULL) < 0) {
        ACH_LOG( LOG_ERR, "Couldn't install signal handler: %s", strerror(errno) );
    }

    /* if( SIG_ERR == signal(SIGPIPE, SIG_IGN) ) { */
    /*     ACH_LOG( LOG_ERR, "Couldn't ignore SIGPIPE: %s", strerror(errno) ); */
    /* } */
}

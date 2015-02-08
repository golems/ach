/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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
            "Copyright (c) 2008-2014, Georgia Tech Research Corporation\n"
            "Copyright (c) 2013-2014, Prevas A/S\n"
            "Copyright (c) 2015, Rice University\n"
            "This is free software; see the source for copying conditions.  There is NO\n"
            "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n",
            name
        );
}

void ach_log( int level, const char fmt[], ...) {
    /* If stderr is a tty, print to it */
    int tty = isatty(STDERR_FILENO);
    if( tty ) {
        va_list argp;
        va_start( argp, fmt );
        vfprintf(stderr, fmt, argp );
        fflush(stderr);
        va_end( argp );
    }
    /* If stderr is not a tty or our parent is init, log to syslog */
    if( !tty || 1 == getppid() ) {
        va_list argp;
        va_start( argp, fmt );
        vsyslog( level, fmt, argp);
        va_end( argp );
    }
}

/* A signal handler that does nothing */
static void ach_sigdummy(int sig) {
    (void)sig;
}


void ach_sig_mask( const int *sig, sigset_t *mask ) {
    if( sigemptyset(mask) ) ACH_DIE("sigemptyset failed: %s\n", strerror(errno));
    size_t i;
    for( i = 0; sig[i]; i ++ ) {
        if( sigaddset(mask, sig[i]) ) {
            ACH_DIE("sigaddset of %s (%d) failed: %s\n",
                    strsignal(sig[i]), sig[i], strerror(errno) );
        }
    }
}


int ach_sig_wait( const int *sigs ) {
    ACH_LOG( LOG_DEBUG, "pid %d waiting for signal\n", getpid() );

    sigset_t waitset;
    ach_sig_mask( sigs, &waitset );

    int sig;
    if( sigwait(&waitset, &sig) ) {
        ACH_DIE("sigwait failed: %s\n", strerror(errno));
    }

    ACH_LOG( LOG_DEBUG, "pid %d signalled: '%s' %d\n",
             getpid(), strsignal(sig), sig );

    return sig;
}

void ach_sig_block_dummy( const int *sig ) {
    /* Block Signal */
    {
        sigset_t blockmask;
        ach_sig_mask( sig, &blockmask );
        if( sigprocmask(SIG_BLOCK, &blockmask, NULL) ) {
            ACH_DIE( "sigprocmask failed: %s\n", strerror(errno) );
        }
    }
    /* Install Dummy Handler */
    size_t i;
    for( i = 0; sig[i]; i ++ ) {
        struct sigaction act;
        memset( &act, 0, sizeof(act) );
        act.sa_handler = &ach_sigdummy;
        if (sigaction(sig[i], &act, NULL) < 0) {
            ACH_LOG( LOG_ERR, "Couldn't install signal handler: %s", strerror(errno) );
        }
    }
}

void ach_sig_dfl_unblock( const int *sig ) {
    size_t i;
    for( i = 0; sig[i]; i ++ ) {
        /* Default Disposition */
        if( SIG_ERR == signal(sig[i], SIG_DFL) ) {
            ACH_LOG( LOG_ERR, "Couldn't set default signal disposition for %s (%d): %s",
                     strsignal(sig[i]), sig[i], strerror(errno) );
        }
    }
    /* Unblock Signal */
    {
        sigset_t blockmask;
        ach_sig_mask( sig, &blockmask );
        if( sigprocmask(SIG_UNBLOCK, &blockmask, NULL) ) {
            ACH_DIE( "sigprocmask failed: %s\n", strerror(errno) );
        }
    }
}

pid_t ach_detach ( unsigned timeout ) {
    pid_t gp_pid = getpid();
    ACH_LOG( LOG_DEBUG, "detach grandparent: %d\n", gp_pid );
    /* Block signals for child status notification */
    const int sigs[] = {ACH_SIG_OK, ACH_SIG_FAIL, SIGALRM, 0};
    ach_sig_block_dummy(sigs);

    /* fork */
    pid_t pid1 = fork();
    if( pid1 < 0 ) {
        ACH_DIE( "First fork failed: %s\n", strerror(errno) );
    } else if ( pid1 ) { /* parent */
        /* wait for a signal */
        alarm( timeout );
        int sig = ach_sig_wait(sigs);
        ACH_LOG( LOG_DEBUG, "Detach grandparent got: '%s' (%d)\n",
                 strsignal(sig), sig );
        switch( sig ) {
        case SIGALRM:
            ACH_LOG( LOG_ERR, "Detached child failed on timeout\n" );
            exit( EXIT_FAILURE );
        case SIGUSR2:
            ACH_LOG( LOG_ERR, "Detached child reported failure\n" );
            exit( EXIT_FAILURE );
        case SIGUSR1:
            ACH_LOG( LOG_DEBUG, "Detached child OK\n" );
            exit(EXIT_SUCCESS);
        default:
            ACH_LOG( LOG_ERR, "Unexpected signal in detach: %s (%d)\n",
                     strsignal(sig), sig );
            exit( EXIT_FAILURE );
        }
        assert(0);
    } /* else child */

    /* Unblock signals that were blocked in the parent */
    ach_sig_dfl_unblock( sigs );

    /* set session id to lose our controlling terminal */
    if( setsid() < 0 ) {
        ACH_LOG( LOG_ERR, "Couldn't set sid: %s\n", strerror(errno) );
    }

    /* refork to prevent future controlling ttys */
    pid_t pid2 = fork();
    if( pid2 < 0 ) {
        ACH_LOG( LOG_ERR, "Second fork failed: %s\n", strerror(errno) );
        /* Don't give up */
    } else if ( pid2 ) { /* parent */
        ACH_LOG( LOG_DEBUG, "detach parent: %d\n", getpid() );
        exit(EXIT_SUCCESS);
    } /* else child */

    ACH_LOG( LOG_DEBUG, "detach child: %d\n", getpid() );

    /* ignore sighup */
    if( SIG_ERR == signal(SIGHUP, SIG_IGN) ) {
        ACH_LOG( LOG_ERR, "Couldn't ignore SIGHUP: %s", strerror(errno) );
    }

    /* cd to root */
    if( chdir("/") ) {
        ACH_LOG( LOG_ERR, "Couldn't cd to /: %s", strerror(errno) );
    }

    /* close stdin */
    if( close(STDIN_FILENO) ) {
        ACH_LOG( LOG_ERR, "Couldn't close stdin: %s", strerror(errno) );
    }

    return gp_pid;
}


pid_t ach_pid_notify = 0;

void ach_notify(int sig) {
    if( ach_pid_notify > 0 ) {
        if( kill(ach_pid_notify, sig) ) {
            ACH_LOG( LOG_ERR, "Could not notify pid %d of failure: %s\n",
                     ach_pid_notify, strerror(errno) );
        }
        ach_pid_notify = 0;
    }
}

void ach_die(void) {
    /* notify failure if not our parent */
    if( getppid() != ach_pid_notify )
        ach_notify(ACH_SIG_FAIL);
    exit(EXIT_FAILURE);
}

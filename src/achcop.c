/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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


/*
 * achcop: Watchdog process for ach-using daemons
 */


/* TODO: don't give up for non-catastrophic errors
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <signal.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>
#include <sys/wait.h>
#include "ach.h"
#include "achutil.h"


/* EVENTS:
 *   Message Timeout
 *   Invalid Message
 *   Process Exits
 */

/* Avoiding signal races:
 *
 * This program needs to wait for SIGTERM and child termination.
 * Waiting for children to terminate with wait() would create a race
 * condition on detection of SIGTERMs.  If SIGTERM is received after
 * we check the flag, but before we enter wait(), we would never
 * realize it.  Instead, we block the signals, and sigwait() to
 * synchronously recieve.
 */

/* File locking note:
 *
 * PID files are locked with lockf().  These locks are released if the
 * holding process exits.  When the parent process sticks around to
 * monitor the child, the parent holds the lock.  When the parent
 * directly exec's to the 'child' (parent and child are really the
 * same process) the exec'ed program inherits the lock.  However, the
 * lock will be lost if the file descriptor is closed.  Therefore,
 * exec'ed programs must not close the lock file descriptor.
 */

/*
 * If child returns 0: exit normally
 * If child returns !0: restart it
 * If child terminated by signal: restart it
 * If sigterm received, signal child and wait for child to exit
 */


/* CLI options */
static void detach(void);
static void redirect(const char *name, int oldfd, int newfd );
static int open_file(const char *name, int options );
static int open_out_file(const char *name );
static void lock_pid(int fd );

static void write_pid( int fd, pid_t pid);
static void child_arg( const char ***args, const char *arg, size_t *n);
static int run( int fd_pid, pid_t *pid, const char *file, const char ** args);
static void start_child( int fd_pid, pid_t *pid, const char *file, const char ** args);
static void waitloop( pid_t *pid, int *status, int *signalled );

static int achcop_signals[] = {SIGINT, SIGTERM, SIGCHLD, 0};

/* Wait for signal to be received, taking care to avoid races
 * Returns the received signal*/
static int wait_for_signal(void);

#define FILE_CLOSE_NAME "-"

#ifdef __GNUC__
#define ACHD_ATTR_PRINTF(m,n) __attribute__((format(printf, m, n)))
#else
#define ACHD_ATTR_PRINTF(m,n)
#endif

sig_atomic_t achcop_sigchild_received = 0;


int main( int argc, char **argv ) {
    /* Command line arguments */
    static struct {
        const char *file_cop_pid;
        const char *file_child_pid;
        const char *file_stderr;
        const char *file_stdout;
        const char **child_args;
        size_t n_child_args;
        int detach;
        int restart;
    } opt = {0};

    /* Global state */
    static struct {
        pid_t pid_child;
        int fd_out;
        int fd_err;
        int fd_cop_pid;
        int fd_child_pid;
    } cx = {0};

    /* Parse Options */
    int c;
    opterr = 0;
    while( (c = getopt( argc, argv, "P:p:o:e:rvdhH?V")) != -1 ) {
        switch(c) {
        case 'P': opt.file_cop_pid = optarg; break;
        case 'p': opt.file_child_pid = optarg; break;
        case 'o': opt.file_stdout = optarg; break;
        case 'e': opt.file_stderr = optarg; break;
        case 'd': opt.detach = 1; break;
        case 'r': opt.restart = 1; break;
        case 'V':   /* version     */
            ach_print_version("achcop");
            exit(EXIT_SUCCESS);
        case 'v': ach_verbosity++; break;
        case '?':   /* help     */
        case 'h':
        case 'H':
            puts( "Usage: achcop [OPTIONS...] -- child-name [CHILD-OPTIONS]\n"
                  "Watchdog to run and restart ach child processes\n"
                  "\n"
                  "Options:\n"
                  "  -P pid-file,      File for pid of cop process (only valid with -r)\n"
                  "  -p pid-file,      File for pid of child process\n"
                  "  -o out-file,      Redirect stdout to this file ("FILE_CLOSE_NAME" to close)\n"
                  "  -e err-file,      Redirect stderr to this file ("FILE_CLOSE_NAME" to close)\n"
                  "  -d,               Detach and run in background\n"
                  "  -r,               Restart failed children\n"
                  //"  -s,             Wait for SIGUSR1 to redirect output and restart child\n"
                  "  -v,               Make output more verbose\n"
                  "  -?,               Give program help list\n"
                  "  -V,               Print program version\n"
                  "\n"
                  "Examples:\n"
                  "  achcop -rd -P /var/run/myppid -p /var/run/mypid -o /var/log/myout -- my-daemon -xyz"
                  "\n"
                  "Report bugs to <ntd@gatech.edu>"
                );
            exit(EXIT_SUCCESS);
        default:
            child_arg(&opt.child_args, optarg, &opt.n_child_args);
        }
    }
    while( optind < argc ) {
        child_arg(&opt.child_args, argv[optind++], &opt.n_child_args);
    }
    /* Check args */
    if( 0 == opt.n_child_args || NULL == opt.child_args ) {
        ACH_DIE("No child process given\n");
    }
    child_arg(&opt.child_args, NULL, &opt.n_child_args); /* Null-terminate child arg array */

    /* open PID files */
    /* do this before the detach(), which may chdir() */
    cx.fd_cop_pid = open_file( opt.file_cop_pid, 0 );
    cx.fd_child_pid = open_file( opt.file_child_pid, 0 );
    cx.fd_out = open_out_file( opt.file_stdout );
    cx.fd_err = open_out_file( opt.file_stderr );

    /* Detach */
    if( opt.detach ) detach();

    /* Open and Lock PID files */
    if( opt.restart ) {
        /* Lock both PID files */
        lock_pid( cx.fd_cop_pid );
        lock_pid( cx.fd_child_pid );
        /* Write parent pid */
        write_pid( cx.fd_cop_pid, getpid() );
    } else {
        /* Lock only 'child' PID file
         * Since we exec the 'child', it's really the same as the parent process
         */
        lock_pid( cx.fd_child_pid );
        /* Write self pid */
        write_pid( cx.fd_child_pid, getpid() );
    }

    /* Redirect */
    redirect(opt.file_stdout, cx.fd_out, STDOUT_FILENO );
    redirect(opt.file_stderr, cx.fd_err, STDERR_FILENO );

    /* TODO: fork a second child to monitor an ach channel.  Restart
     * first child if second child signals or exits */

    /* Fork child */
    if( opt.restart ) {
        return run( cx.fd_child_pid, &cx.pid_child, opt.child_args[0], opt.child_args );
    } else {
        /* No restarts, just exec the child */
        execvp( opt.child_args[0], (char *const*)opt.child_args );
        ACH_DIE( "Could not exec: %s\n", strerror(errno) );
    }

    /* Should never get here */
    return EXIT_FAILURE;
}


static void child_arg(const char ***args, const char *arg, size_t *n) {
    *args = (const char **)realloc( *args, (*n+1)*sizeof(arg) );
    (*args)[(*n)++] = arg;
}

static void detach(void) {
    /* open syslog */
    openlog("achcop", LOG_PID, LOG_DAEMON);

    /* fork */
    pid_t pid1 = fork();
    if( pid1 < 0 ) {
        ACH_DIE( "First fork failed: %s\n", strerror(errno) );
    } else if ( pid1 ) { /* parent */
        exit(EXIT_SUCCESS);
    } /* else child */

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
        exit(EXIT_SUCCESS);
    } /* else child */

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
}

static void redirect(const char *name, int oldfd, int newfd ) {
    /* check no-op */
    if( NULL == name ) return;

    /* check close */
    if( 0 == strcmp(name, FILE_CLOSE_NAME) ) {
        if( close(newfd) ) {
            ACH_LOG( LOG_ERR, "Couldn't close file descriptor: %s\n", strerror(errno) );
        }
        return;
    }

    /* check no-op */
    if( -1 == oldfd ) return;

    /* dup */
    if( -1 == dup2(oldfd, newfd) ) {
        ACH_LOG( LOG_ERR, "Could not dup output: %s\n", strerror(errno) );
    }
    if( close(oldfd) ) {
        ACH_LOG( LOG_ERR, "Couldn't close dup'ed file descriptor: %s\n", strerror(errno) );
    }
}

static int open_out_file(const char *name ) {
    if( NULL == name || 0 == strcmp(FILE_CLOSE_NAME, name) ) {
        return -1;
    }
    return open_file( name, O_APPEND );
}
static int open_file(const char *name, int opts) {
    if( NULL == name ) {
        return -1;
    }
    /* open */
    int fd = open( name, O_RDWR|O_CREAT|opts, 0664 );
    if( fd < 0 ) {
        ACH_DIE( "Could not open file %s: %s\n", name, strerror(errno) );
    }
    return fd;
}

static void lock_pid( int fd ) {
    if( -1 == fd ) return;
    /* lock */
    if( lockf(fd, F_TLOCK, 0) ) {
        ACH_DIE( "Could not lock pid file: %s\n", strerror(errno) );
    }
}

static void write_pid( int fd, pid_t pid ) {
    if( -1 == fd ) return;
    /* truncate */
    if( ftruncate(fd, 0) ) {
        ACH_DIE( "Could truncate pid file: %s\n", strerror(errno) );
    }
    /* seek */
    if( lseek(fd, 0, SEEK_SET) ) {
        ACH_LOG( LOG_ERR, "Could seek pid file: %s\n", strerror(errno) );
    }
    /* format */
    char buf[32] = {0}; /* 64 bits should take 21 decimal digits, including '\0' */
    int size = snprintf( buf, sizeof(buf), "%d", pid );
    if( size < 0 ) {
        ACH_LOG(LOG_ERR, "Error formatting pid\n");
        return;
    } else if ( size >= (int)sizeof(buf)) {
        ACH_LOG(LOG_ERR, "PID buffer overflow for pid=%d\n", pid); /* What, you've got 128 bit PIDs? */
        return;
    }
    /* write */
    size_t n = 0;
    while( n < (size_t)size ) {
        ssize_t r = write( fd, buf+n, (size_t)size-n );
        if( r > 0 ) n += (size_t)r;
        else if (EINTR == errno) continue;
        else {
            ACH_LOG( LOG_ERR, "Could not write pid %d: %s\n", pid, strerror(errno) );
            return;
        }
    } while( n < (size_t)size);
    ACH_LOG( LOG_DEBUG, "Write pid %d\n", pid );
}


/* Now it gets hairy... */

static int run( int fd_pid, pid_t *pid_ptr, const char *file, const char **args) {

    /* Block Signals */
    /* The signal mask is inherited through fork and exec, so we need
     * to unblock these for the child later */
    { size_t i;
        for( i = 0; achcop_signals[i]; i ++ ) {
            ach_sig_block_dummy( achcop_signals[i] );
        }
    }

    int do_start = 1, done = 0, exit_status = EXIT_FAILURE;
    while( !done ) {
        /* start */
        if( do_start ) {
            start_child( fd_pid, pid_ptr, file, args );
            do_start = 0;
        }
        /* wait for signal */
        int sig = wait_for_signal();
        int status=0, signal=0;
        pid_t wpid;
        /* handle signal */
        switch(sig) {
        case SIGTERM:
        case SIGINT: /* We got a sigterm/sigint */
            /* Kill Child */
            ACH_LOG(LOG_DEBUG, "Killing child\n");
            if( kill(*pid_ptr, SIGTERM) ) ACH_DIE( "Couldn't kill child: %s\n", strerror(errno) );
            /* Wait Child */
            do { waitloop(&wpid, &status, &signal);
            } while (wpid != *pid_ptr);
            /* TODO: timeout and SIGKILL child */
            /* Exit */
            exit_status = status;
            done = 1;
            break;
        case SIGCHLD: /* child died */
            /* Get child status and restart or exit */
            do { waitloop(&wpid, &status, &signal);
            } while (wpid != *pid_ptr);
            if( 0 == signal && EXIT_SUCCESS == status ) {
                ACH_LOG(LOG_DEBUG, "Child returned success, exiting\n");
                exit_status = status;
                done = 1;
            } else {
                ACH_LOG(LOG_DEBUG, "Restarting child\n");
                /* else restart */
                do_start = 1;
            }
            break;
        default:
            /* continue and sigwait() */
            ACH_LOG( LOG_WARNING, "Spurious signal received: %s (%d)\n",
                     strsignal(sig), sig );
        }
    }
    /* Done */
    return exit_status;
}

static void start_child( int fd_pid, pid_t *pid_ptr, const char *file, const char **args) {
    pid_t pid = fork();

    if( 0 == pid ) { /* child: exec */
        /* Unblock signals for the child */
        { size_t i;
            for( i = 0; achcop_signals[i]; i ++ ) {
                ach_sig_dfl_unblock( achcop_signals[i] );
            }
        }
        execvp( file, (char *const*)args );
        ACH_DIE( "Could not exec: %s\n", strerror(errno) );
    } else if ( pid > 0 ) { /* parent: record child */
        ACH_LOG( LOG_DEBUG, "Child forked, pid %d\n", pid )
        *pid_ptr = pid;
        write_pid( fd_pid, pid );
    } else {
        /* TODO: handle EAGAIN */
        ACH_DIE( "Could not fork child: %s\n", strerror(errno) );
    }
}

static void waitloop( pid_t *pid, int *exit_status, int *signal ) {
    *pid = 0;
    *exit_status = 0;
    *signal = 0;
    while(1) {
        int status;
        *pid = wait( &status );
        if( *pid < 0 ) { /* Wait failed */
            if( EINTR == errno ) {
                ACH_LOG(LOG_DEBUG, "wait interrupted\n");
            } else if (ECHILD == errno) {
                ACH_DIE("unexpected ECHILD\n");
            } else { /* something bad */
                ACH_DIE( "Couldn't wait for child: %s\n", strerror(errno));
            }
        } else { /* Child did something */
            if( WIFEXITED(status) ) { /* child exited */
                ACH_LOG(LOG_DEBUG, "child exited with %d\n", WEXITSTATUS(status));
                *exit_status = WEXITSTATUS(status);
                return;
            } else if ( WIFSIGNALED(status) ) { /* child signalled */
                *signal = WTERMSIG(status);
                ACH_LOG( LOG_DEBUG, "child signalled: %s (%d)\n",
                         strsignal(*signal), *signal );
                return;
            } else { /* something weird happened */
                ACH_LOG(LOG_WARNING, "Unexpected wait result %d\n", status);
                /* I guess we keep waiting then */
            }
        }
    }
}

static int wait_for_signal() {
    ACH_LOG( LOG_DEBUG, "waiting for signal\n" );

    sigset_t waitset;
    if( sigemptyset(&waitset) ) ACH_DIE("sigemptyset failed: %s\n", strerror(errno));
    { size_t i;
        for( i = 0; achcop_signals[i]; i ++ ) {
            if( sigaddset(&waitset, achcop_signals[i]) ) {
                ACH_DIE("sigaddset failed: %s\n", strerror(errno));
            }
        }
    }

    int sig;
    if( sigwait(&waitset, &sig) ) {
        ACH_DIE("sigwait failed: %s\n", strerror(errno));
    }

    ACH_LOG( LOG_DEBUG, "Signalled: %d\n", sig );

    return sig;
}

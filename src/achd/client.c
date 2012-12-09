/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
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

#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <ctype.h>
#include <signal.h>
#include <regex.h>
#include <assert.h>
#include <stdarg.h>
#include <errno.h>
#include <syslog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "ach.h"
#include "achutil.h"
#include "achd.h"

/*********
* Client *
*********/

static void client_refork(void);
static int socket_connect(void);
static int server_connect( struct achd_conn*);
static void daemonize(void);
static void sleep_till( const struct timespec *t0, int32_t ns );
static pid_t wait_for_child( pid_t pid );

void achd_client() {
    /* First, do some checks to make sure we can process the request */
    struct achd_conn conn;
    memset(&conn, 0, sizeof(conn));

    /* Create request headers */
    conn.vtab = achd_get_vtab( cx.cl_opts.transport, cx.cl_opts.direction );
    assert( conn.vtab && conn.vtab->handler );

    if( cx.cl_opts.remote_chan_name ) {
        conn.send_hdr.chan_name = cx.cl_opts.remote_chan_name;
    } else {
        cx.error( ACH_BAD_HEADER, "No channel name given\n");
        assert(0);
    }

    assert( cx.cl_opts.transport );
    assert( cx.cl_opts.direction == ACHD_DIRECTION_PUSH ||
            cx.cl_opts.direction == ACHD_DIRECTION_PULL );
    conn.send_hdr.transport = cx.cl_opts.transport;

    /* Check the channel */
    {
        int r = ach_open(&conn.channel, cx.cl_opts.chan_name, NULL );
        if( ACH_ENOENT == r ) {
            achd_log(LOG_INFO, "Local channel %s not found, creating\n", cx.cl_opts.chan_name);
        } else if (ACH_OK != r) {
            /* Something is wrong with the channel, probably permissions */
            cx.error( r, "Couldn't open channel %s\n", cx.cl_opts.chan_name );
            assert(0);
        } else {
            ach_flush(&conn.channel);
        }
    }

    sighandler_install();

    /* maybe daemonize */
    if( cx.daemonize ) {
        daemonize();
    }

    /* fork a worker if we are to retry connectons */
    client_refork();

    /* Finally, start the show, possibly in a child process */

    /* Open initial connection */
    int fd = server_connect( &conn );
    if( fd < 0 ) fd = achd_reconnect( &conn );

    /* Allocate buffers */
    conn.pipeframe_size =
        conn.channel.shm->data_size / conn.channel.shm->index_cnt;
    conn.pipeframe = ach_pipe_alloc( conn.pipeframe_size );

    /* TODO: If we lose and then re-establish a connections, frames
     * may be missed or duplicated.
     *
     * This can be more easily fixed with a pushing client, since it
     * can maintain channel context and try to reconnect.  We just
     * need to buffer the last read frame till we reconnect, or re-get
     * it with ACH_O_COPY.  Other frames can remain buffered in the
     * channel.  If the client dies, though, we lose context.
     *
     * Fixing this for pulling requires the server process to persist
     * after lost connection.  That seems harder to implement.
     */

    /* Start running */
    if ( fd >= 0 && !cx.sig_received ) {
        achd_log(LOG_INFO, "Client running\n");
        conn.vtab->handler( &conn );
        achd_log(LOG_INFO, "Client done\n");
    }
}

static int server_connect( struct achd_conn *conn) {
    achd_log( LOG_NOTICE, "Connecting to %s:%d\n", cx.cl_opts.remote_host, cx.port );
    conn->in = conn->out = -1;

    /* Note the time */
    clock_gettime( CLOCK_MONOTONIC, &conn->t0 );

    /* Connect to server */
    int fd = socket_connect();
    if (fd < 0) {
        return fd;
    } else {
        achd_log( LOG_DEBUG, "Socket connected\n");
    }

    /* Write request */
    {
        conn->in = conn->out = fd;
        if( conn->vtab->connect ) conn->vtab->connect(conn);
        conn->in = conn->out = -1;
        enum ach_status r =
            achd_printf(fd,
                        "channel-name: %s\n"
                        "transport: %s\n"
                        "direction: %s\n"
                        ".\n",
                        conn->send_hdr.chan_name,
                        conn->send_hdr.transport,
                        ( (cx.cl_opts.direction == ACHD_DIRECTION_PULL) ?
                          "push" : "pull" )
                        /* remote end does the opposite */ );
        if( ACH_OK != r ) {
            achd_log(LOG_DEBUG, "couldn't send headers\n");
            close(fd);
            return -1;
        } else {
            achd_log(LOG_DEBUG, "headers sent\n");
        }
    }

    /* Get Response */
    conn->recv_hdr.status = ACH_BUG;
    {
        int r = achd_parse_headers( fd, &conn->recv_hdr );
        if( ACH_OK != r ) {
            if( errno ) {
                cx.error( r, "Bad response from server: %s\n", strerror(errno) );
            } else {
                cx.error( r, "Bad response from server\n");
            }
            assert(0);
        } else if ( ACH_OK != conn->recv_hdr.status ) {
            if( conn->recv_hdr.message ) {
                cx.error( conn->recv_hdr.status, "Server error: %s\n", conn->recv_hdr.message );
                assert(0);
            } else {
                cx.error( conn->recv_hdr.status, "Bad response from server\n" );
                assert(0);
            }
        }
    }
    achd_log(LOG_DEBUG, "Server response received\n");

    /* Try to create channel if needed */
    if( ! conn->channel.shm ) {
        int frame_size = conn->recv_hdr.frame_size ? conn->recv_hdr.frame_size : ACH_DEFAULT_FRAME_SIZE;
        int frame_count = conn->recv_hdr.frame_count ? conn->recv_hdr.frame_count : ACH_DEFAULT_FRAME_COUNT;
        /* Fixme: should sanity check these counts */
        int r = ach_create( cx.cl_opts.chan_name, (size_t)frame_count, (size_t)frame_size, NULL );
        if( ACH_OK != r )  cx.error( r, "Couldn't create channel\n");
        r = ach_open(&conn->channel, cx.cl_opts.chan_name, NULL );
        if( ACH_OK != r )  cx.error( r, "Couldn't open channel\n");
    }

    return conn->in = conn->out = fd;
}

int achd_reconnect( struct achd_conn *conn) {
    int fd = -1;
    while( cx.reconnect && fd < 0 && !cx.sig_received ) {
        achd_log(LOG_DEBUG, "Reconnect attempt\n");
        sleep_till( &conn->t0, ACHD_RECONNECT_NS );
        fd = server_connect( conn );
    }

    return fd;
}

static int socket_connect() {
    /* Make socket */
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        achd_log(LOG_ERR, "Couldn't create socket: %s\n", strerror(errno));
        return sockfd;
    }

    /* Lookup Host */
    assert( cx.cl_opts.remote_host );
    struct hostent *server = gethostbyname( cx.cl_opts.remote_host );
    if( NULL == server ) {
        achd_log(LOG_ERR, "Host '%s' not found\n", cx.cl_opts.remote_host);
        close(sockfd);
        return -1;
    }

    /* Make socket address */
    struct sockaddr_in serv_addr;
    memset( &serv_addr, 0, sizeof(serv_addr) );
    serv_addr.sin_family = AF_INET;
    memcpy( &serv_addr.sin_addr.s_addr,
            server->h_addr,
            (size_t)server->h_length);
    serv_addr.sin_port = htons(cx.port); /* Well, ipv4 only for now */

    /* Connect */
    if( connect(sockfd,  (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0 ) {
        achd_log(LOG_ERR, "Couldn't connect: %s\n", strerror(errno));
        close(sockfd);
        return -1;
    }

    return sockfd;
}


static void sleep_till( const struct timespec *t0, int32_t ns ) {
    int64_t ns1 = t0->tv_nsec + ns;
    struct timespec t = {.tv_sec = t0->tv_sec + ns1 / 1000000000,
                         .tv_nsec = ns1 % 1000000000 };

    while(!cx.sig_received) {
        int r = clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME,
                             &t, NULL );
        if (r && !cx.sig_received && EINTR == errno ) {
            /* signalled */
            continue;
        } else if (!r) {
            /* time elapsed */
            break;
        } else {
            /* weird error */
            achd_log( LOG_ERR, "clock_nanosleep failed: %s\n", strerror(errno) );
            break;
        }
        assert(0);
    }
}

/* Wait till child returns or signal received */
static pid_t wait_for_child( pid_t pid ) {
    assert( pid > 0 );
    while(1)
    {
        int status;
        achd_log(LOG_DEBUG, "now waiting: %d...\n", pid);
        pid_t wpid = wait( &status );
        achd_log(LOG_DEBUG, "wait(): %d, errno:  %s\n", wpid, strerror(errno));
        if( wpid == pid ) {
            achd_log(LOG_DEBUG, "WIFEXITED: %d, WIFSIGNALED: %d, WIFSTOPPED: %d\n",
                     WIFEXITED(status),
                     WIFSIGNALED(status),
                     WIFSTOPPED(status));
            /* Child did something */
            if( WIFEXITED(status) ) {
                achd_log(LOG_DEBUG, "child exited with %d\n", WEXITSTATUS(status));
                return 0;
            } else if ( WIFSIGNALED(status) ) {
                achd_log(LOG_DEBUG, "child signalled with %d\n", WTERMSIG(status));
                return 0;
            } else {
                achd_log(LOG_WARNING, "Unexpected wait result %d\n", status);
                continue; /* I guess we keep waiting then */
            }
        } else if ( wpid < 0 ) {
            /* Wait failed */
            if (cx.sig_received ) {
                achd_log(LOG_DEBUG, "signal during wait\n");
                return pid; /* process still running */
            } else if( EINTR == errno ) {
                achd_log(LOG_DEBUG, "wait interrupted\n");
                continue; /* interrupted */
            } else if (ECHILD == errno) {
                achd_log(LOG_WARNING, "unexpected ECHILD\n");
                return 0; /* child somehow died */
            } else { /* something bad */
                cx.error(ACH_FAILED_SYSCALL, "Couldn't wait for child\n", strerror(errno));
            }
        } else {
            /* Wrong child somehow */
            cx.error(ACH_BUG, "Got unexpected PID, child %d, wait %d\n", pid, wpid);
        }
        assert(0);
    }
}

static void client_refork() {
    /* TODO: When we refork the child, we "forget" our position in
     * the channel.  This means some frames may be missed or resent.
     * It would be better to have the child attempt to reconnect if
     * the connection gets broken.
     */
    if( !cx.reconnect ) return;

    pid_t pid = 0;

    /* Loop to fork children when the die */
    while( ! cx.sig_received ) {
        assert( 0 == pid );
        struct timespec t;
        clock_gettime( CLOCK_MONOTONIC, &t );
        pid = fork();
        if( 0 == pid ) { /* Child case */
            return;
        } else if ( pid > 0 ) {
            achd_log( LOG_DEBUG, "Forked child %d\n", pid );
            /* Parent Case */
            pid = wait_for_child(pid);
            if( 0 == pid && !cx.sig_received ) {
                achd_log( LOG_WARNING, "Child died, reforking\n" );
            }
            /* Maybe sleep to prevent forking too fast */
            sleep_till(&t, ACHD_REFORK_NS);
        } else {
            assert( pid < 0 );
            /* Error Case */
            /* TODO: retry on EAGAIN */
            cx.error(ACH_FAILED_SYSCALL, "Couldn't fork: %s\n", strerror(errno));
        }
    }

    /* still in parent */
    if( pid > 0 ) {
        /* kill child if running */
        if( kill( pid, SIGTERM ) ) {
            achd_log( LOG_ERR, "Couldn't kill child: %s\n", strerror(errno) );
        }
    }

    exit(EXIT_SUCCESS);
}


static void daemonize() {
    /* fork */
    pid_t grandparent = getpid();
    pid_t pid1 = fork();
    if( pid1 < 0 ) {
        achd_log( LOG_CRIT, "First fork failed: %s\n", strerror(errno) );
        exit(EXIT_FAILURE);
    } else if ( pid1 ) { /* parent */
        exit(EXIT_SUCCESS);
    } /* else child */
    pid1 = getppid();

    /* set session id to lose our controlling terminal */
    if( setsid() < 0 ) {
        achd_log( LOG_WARNING, "Couldn't set sid: %s\n", strerror(errno) );
    }

    /* refork to prevent future controlling ttys */
    pid_t parent = getpid();
    pid_t pid2 = fork();
    if( pid2 < 0 ) {
        achd_log( LOG_ERR, "Second fork failed: %s\n", strerror(errno) );
        /* Don't give up */
    } else if ( pid2 ) { /* parent */
        exit(EXIT_SUCCESS);
    } /* else child */

    /* ignore sighup */
    if( SIG_ERR == signal(SIGHUP, SIG_IGN) ) {
        achd_log( LOG_ERR, "Couldn't ignore SIGHUP: %s", strerror(errno) );
    }

    /* cd to root */
    if( chdir("/") ) {
        achd_log( LOG_ERR, "Couldn't cd to /: %s", strerror(errno) );
    }

    /* file mask */
    umask(0);

    /* close stdin/stdout/stderr */
    if( close(STDIN_FILENO) ) {
        achd_log( LOG_ERR, "Couldn't close stdin: %s", strerror(errno) );
    }
    if( close(STDOUT_FILENO) ) {
        achd_log( LOG_ERR, "Couldn't close stdout: %s", strerror(errno) );
    }
    if( close(STDERR_FILENO) ) {
        achd_log( LOG_ERR, "Couldn't close stderr: %s", strerror(errno) );
    }
}

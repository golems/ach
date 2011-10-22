/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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

/** \file achchand.c
 *  \author Neil T. Dantam
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <argp.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "ach.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <syslog.h>
#include <signal.h>
#include <poll.h>
#include <inttypes.h>

// from linux man page
#define UNIX_PATH_MAX    108

#define TIMEOUT_MS_POLL 1000


/* Threads: RT thread waits in channel.  When a message is posted, it
 * signals all subscribing processes and also the Net thread.  This
 * interrupts the net thread's poll() call, so it can get the message
 * from the channel and write it to all network receivers.
 *
 * An alternative would be to have publishers signal the channel
 * daemon; however, this would require publishers to either run with
 * elevated permissions, or to run as the same user as the channel
 * daemon.
 *
 * Store subscriber PIDs in an array.  RT thread CAS in NULL, iterates
 * over, CAS back.  Net thread creates new array, CAS in, and frees old.
 */

/* Memory Allocation:
 * - Should track max size of fd arrays and not realloc if there's still space
 * - Should pool ach_fd structs, use array of pointers
 * - Should pool control input/output buffers, fixed size ok since commands are short
 * - Should pool ach frame buffers, fixed size ok since messages limited by allocated channel size
 */

enum achd_fdtype {
    ACHD_FD_CTRL_BAD = 0,
    ACHD_FD_CTRL_SRV,
    ACHD_FD_CTRL_CONN
};

struct achd_fd{
    enum achd_fdtype type;
    uint8_t *in;
    size_t n_in;
    size_t max_in;

    uint8_t *out;
    size_t n_out;
    size_t max_out;
};

struct achd_cx {
    size_t n;                   ///< number of fds
    struct achd_fd *fds;        ///< info on open fds
    struct pollfd *pfds;        ///< array for poll()
    struct sockaddr_un addr;    ///< address for local-domain server sock


    ach_channel_t chan;         ///< channel we're monitoring
    pthread_t chan_thread;      ///< thread that's monitoring the channel
    double period;              ///< nominal seconds between messages

    int quit_flag;              ///< daemon has been told to to quit
};

static struct achd_cx d_cx;


static void init(void);
static void *run_channel(void *);
static void run_io(void);
static void destroy(void);

static void* run_channel_thread(void*);




int main(int argc, char **argv) {
    (void) argc;
    (void) argv;
    // initialize
    init();
    // start RT thread
    int r = pthread_create( &d_cx.chan_thread, NULL, run_channel, NULL );
    if( 0 != r ) exit(-1);
    // run IO
    run_io();
    // stop RT-thread
    // destroy
    destroy();

    return 0;
}

static void sighandler(int sig, siginfo_t *siginfo, void *context) {
    (void) context;
    fprintf (stderr, "Received Signal: %s(%d), Sending PID: %ld, UID: %ld\n",
             strsignal(sig), sig, (long)siginfo->si_pid, (long)siginfo->si_uid);
    switch( sig ) {
    case SIGTERM:
    case SIGINT:
        d_cx.quit_flag = 1;
        break;
    default:
        syslog(LOG_ERR, "unknown singal: %d", sig );
    }

}


void init() {
    openlog("achchand", LOG_CONS | LOG_NDELAY | LOG_PERROR, LOG_USER );

    // open channel

    // allocate
    d_cx.n = 1;
    d_cx.pfds = AA_NEW0(struct pollfd);
    d_cx.fds = AA_NEW0( struct achd_fd );
    d_cx.fds[0].type = ACHD_FD_CTRL_SRV;

    // setup control socket
    d_cx.pfds[0].fd = socket(PF_UNIX, SOCK_STREAM, 0);
    if( d_cx.pfds[0].fd < 0 ) {
        syslog(LOG_EMERG,"socket failed: %s", strerror(errno));
        exit(-1);
    }
    // bind
    memset(&d_cx.addr, 0, sizeof d_cx.addr);
    d_cx.addr.sun_family = AF_LOCAL;
    snprintf(d_cx.addr.sun_path, UNIX_PATH_MAX, "%s", "/tmp/asock");
    if( 0 > unlink(d_cx.addr.sun_path) &&
        errno != ENOENT ) {
        syslog(LOG_EMERG,"unlink failed: %s", strerror(errno));
        exit(-1);
    }
    if( 0 > bind(d_cx.pfds[0].fd, (struct sockaddr *)&d_cx.addr,
                 (socklen_t)(strlen(d_cx.addr.sun_path) +
                             sizeof(d_cx.addr.sun_family))) ) {
        syslog(LOG_EMERG,"bind failed: %s", strerror(errno));
        exit(-1);
    }
    // listen
    if( 0 > listen(d_cx.pfds[0].fd,5) ) {
        syslog(LOG_EMERG,"listen failed: %s", strerror(errno));
        exit(-1);
    }

    // install signal handler
    struct sigaction act;
    memset(&act, 0, sizeof(act));

    act.sa_sigaction = &sighandler;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(SIGTERM, &act, NULL) ||
        sigaction(SIGINT, &act, NULL) < 0) {
        syslog(LOG_EMERG,"sigaction failed: %s", strerror(errno));
        exit(-1);
    }
}




void destroy() {
    closelog();
}

void *run_channel(void *cx) {
    (void)(cx);
    // do we need a cleanup handler to make sure the mutex is unlocked?

    while( 1 /*something*/ ) {
        struct timespec abstime = aa_tm_future( aa_tm_sec2timespec(5*d_cx.period) );
        // wait on channel (wakeup every so often to check stuff)
        size_t frame_size;

        //ach_wait_next( &d_cx.chan,

        // signal subscribers
    }
    return NULL;
}

void dump_pollevents( int x ) {
    if( x & POLLIN )
        printf("POLLIN:");
    if( x & POLLOUT )
        printf("POLLOUT:");
    if( x & POLLERR )
        printf("POLLERR:");
    if( x & POLLHUP )
        printf("POLLHUP:");
    if( x & POLLNVAL )
        printf("POLLNVAL:");
}

void run_io() {

    void new_fd(int fd, int type) {
        // check for fd growth
        int fd_max =  3 + 1 + (int)d_cx.n;
        if( fd > fd_max ) {
            syslog( LOG_WARNING, "possible unbounded fd growth, fd %d > %d",
                    fd, fd_max );
        }
        d_cx.n++;
        // reallocate
        d_cx.pfds = (struct pollfd*)realloc(d_cx.pfds, d_cx.n * sizeof(*d_cx.pfds) );
        memset(&d_cx.pfds[d_cx.n-1], 0, sizeof(*d_cx.pfds));
        d_cx.fds = (struct achd_fd*)realloc(d_cx.fds, d_cx.n * sizeof(*d_cx.fds) );
        memset(&d_cx.fds[d_cx.n-1], 0, sizeof(*d_cx.fds));
        // set
        size_t i = d_cx.n - 1;
        d_cx.pfds[i].fd = fd;
        d_cx.pfds[i].events = POLLIN;
        d_cx.fds[i].type = type;
        d_cx.pfds[i].revents = 0;
        // create buffers
        d_cx.fds[i].in = (uint8_t*)malloc(4000);
        d_cx.fds[i].out = (uint8_t*)malloc(4000);
        d_cx.fds[i].max_in = 4000;
        d_cx.fds[i].max_out = 4000;
        d_cx.fds[i].n_in = 0;
        d_cx.fds[i].n_out = 0;
    }

    void rm_fd( size_t i ) {
        // close
        int r;
        do { r = close(d_cx.pfds[i].fd ); } while( r < 0 && EINTR == errno );
        if( r < 0 ) { syslog(LOG_ERR, "close failed: %s", strerror(errno)); }
        // free buffers
        aa_free_if_valid( d_cx.fds[i].in );
        aa_free_if_valid( d_cx.fds[i].out );
        // shift down
        memmove( &d_cx.pfds[i], &d_cx.pfds[i+1],
                 sizeof(d_cx.pfds[0])*(d_cx.n - i - 1) );
        memmove( &d_cx.fds[i], &d_cx.fds[i+1],
                 sizeof(d_cx.fds[0])*(d_cx.n - i - 1) );
        d_cx.n--;
    }
    void queue_output( size_t i, const char *str ) {
        struct achd_fd *afd = d_cx.fds+i;
        size_t len = strlen(str);
        strncpy( (char*) afd->out + afd->n_out, str,
                 AA_MIN( afd->max_out - afd->n_out,
                         len ) );
        afd->n_out += len;
        d_cx.pfds[i].events |= POLLOUT;
    }

    static char help_string[] =
        "COMMANDS: \n"
        "  help:  print this message\n"
        "  helo:  server responds with `elho'\n"
        "  quit:  server closes connection\n"
        "  achnotify CHANNEL:  send notification messages to CHANNEL\n"
        "";



    int process_ctrl( size_t i ) {
        struct achd_fd *afd = d_cx.fds+i;
        char *buf = (char*)afd->in;
        assert(NULL == strchr(buf, '\n'));
        // delimit string
        // tokenize
        printf("line: \n");
        char *tok = strtok(buf, " \t");
        if( 0 == strcasecmp( tok, "helo") ) {
            queue_output( i, "elho\n");
        } else if( 0 == strcasecmp( tok, "help") ) {
            queue_output( i, help_string );
        } else if( 0 == strcasecmp( tok, "achnotify") ) {
            // open channel, add to notification set
        } else if( 0 == strcasecmp( tok, "quit") ) {
            return -1;
        } else {
            queue_output( i, "bad command\n");
        }
        return 0;
    }

    d_cx.pfds[0].events = POLLIN;

    int r;

    while(! d_cx.quit_flag ) {
        r = poll(d_cx.pfds, d_cx.n, TIMEOUT_MS_POLL);
        if( r > 0 ) {
            printf("poll %d\n", r);
            for( size_t i = 0; i < d_cx.n; i ++ ) {
                assert( d_cx.fds[i].n_in <= d_cx.fds[i].max_in );
                assert( d_cx.fds[i].n_out <= d_cx.fds[i].max_out );
                switch( d_cx.fds[i].type ) {
                case ACHD_FD_CTRL_SRV:
                    // check new connections
                    //printf("SRV:  ");
                    //dump_pollevents( d_cx.pfds[i].revents );
                    //fputc('\n',stdout);
                    if( d_cx.pfds[i].revents & POLLIN ) {
                        int newconn;
                        do { newconn = accept( d_cx.pfds[i].fd, NULL, NULL ); }
                        while( newconn < 0 && EINTR == newconn );
                        if( newconn < 0 ) {
                            syslog(LOG_ERR, "failed accept: %s", strerror(errno));
                        } else {
                            new_fd( newconn, ACHD_FD_CTRL_CONN );
                        }
                    }
                    // check for weird events
                    if( d_cx.pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL|POLLOUT) ) {
                        syslog(LOG_ERR, "unknown event on srv sock: "
                               "err: %d, hup: %d, nval: %d, out: %d",
                               d_cx.pfds[i].revents&POLLERR, d_cx.pfds[i].revents&POLLHUP,
                               d_cx.pfds[i].revents&POLLNVAL, d_cx.pfds[i].revents&POLLOUT);
                    }
                    break;
                case ACHD_FD_CTRL_CONN:
                    printf("CTRL:  ");
                    dump_pollevents( d_cx.pfds[i].revents );
                    fputc('\n',stdout);
                    // check ctrl connections for input
                    if( d_cx.pfds[i].revents & POLLIN ) {
                        int s = aa_read_realloc( d_cx.pfds[i].fd,(void**)&d_cx.fds[i].in,
                                                 d_cx.fds[i].n_in, &d_cx.fds[i].max_in);
                        if( s < 0 && errno != EINTR ) {
                            syslog( LOG_ERR, "read failed: %s", strerror(errno) );
                        } else if (s > 0) {
                            // got some data
                            size_t j = d_cx.fds[i].n_in;
                            d_cx.fds[i].n_in += (size_t)s;
                            // look for newline
                            while( j < d_cx.fds[i].n_in ) {
                                char c = (char)d_cx.fds[i].in[j];
                                if( '\n' == c || '\r' == c ) {
                                    d_cx.fds[i].in[j] = '\0'; // mark null
                                    if( 0 == process_ctrl(i)  ) {
                                        // move
                                        d_cx.fds[i].n_in = d_cx.fds[i].n_in - j - 1;
                                        memmove( d_cx.fds[i].in, &d_cx.fds[i].in[j+1],
                                                 d_cx.fds[i].n_in );
                                        j = 0;
                                    } else {
                                        rm_fd(i);
                                        i--;
                                        break;
                                    }
                                } else if (!isascii(c) || !isprint(c) || '\0' == c) {
                                    // bad character, close the connection
                                    syslog( LOG_NOTICE, "bad char from client: %d", c );
                                    rm_fd(i);
                                    i--;
                                    break;
                                } else { j++; }
                            }
                        }
                    }

                    if( d_cx.pfds[i].revents & POLLOUT ) {
                        if( d_cx.fds[i].n_out ) {
                            ssize_t s = write( d_cx.pfds[i].fd,
                                               d_cx.fds[i].out, d_cx.fds[i].n_out );
                            if( s < 0 && errno != EINTR ) {
                                syslog( LOG_ERR, "write failed: %s", strerror(errno) );
                            } else if( s > 0 ) {
                                assert( (size_t)s <= d_cx.fds[i].n_out );
                                memmove( d_cx.fds[i].out, d_cx.fds[i].out + s,
                                         d_cx.fds[i].n_out - (size_t)s );
                                d_cx.fds[i].n_out -= (size_t)s;
                            }
                        }
                        if( 0 == d_cx.fds[i].n_out ) {
                            d_cx.pfds[i].events &= ~POLLOUT;
                        }
                    }
                    if( d_cx.pfds[i].revents & (POLLHUP|POLLERR|POLLNVAL) ) {
                        // remove
                        rm_fd(i);
                        // modify i,
                        i--;
                    }
                    // check ctrl connections for output
                    break;
                case ACHD_FD_CTRL_BAD:
                    // check ctrl connections for output
                    syslog(LOG_ERR, "bad fd type at index %"PRIuPTR, i );
                }
                assert( d_cx.fds[i].n_in <= d_cx.fds[i].max_in );
                assert( d_cx.fds[i].n_out <= d_cx.fds[i].max_out );
            }
        } else if (r < 0 && EINTR != errno && EAGAIN != errno ) {
            syslog(LOG_EMERG, "poll failed: %s", strerror(errno) );
            exit(-1);
        } else {
            printf("poll nop\n");
        }
    }
}

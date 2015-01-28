/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/wait.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <errno.h>
#include "ach.h"
#include "achtest.h"

static size_t opt_n_sub  = 1;
static size_t opt_n_pub  = 1;
static size_t opt_n_chan = 1;
static size_t opt_n_sig  = 0;
static int opt_verbosity = 0;
static enum ach_map opt_map = ACH_MAP_DEFAULT;

static pid_t *g_pid_sub = NULL;
static pid_t *g_pid_pub = NULL;
static pid_t *g_pid_sig = NULL;
static struct ach_channel g_chan_begin;
static struct ach_channel *g_chans = NULL;

static volatile sig_atomic_t g_cancel = 0;
static volatile sig_atomic_t g_pid_ready = 0;


#define DEBUG( priority, ... ) if((priority) <= opt_verbosity) fprintf(stderr,__VA_ARGS__);

static void sighandler_nop(int sig, siginfo_t *siginfo, void *context);
static void sighandler_cancel(int sig, siginfo_t *siginfo, void *context);
static void sighandler_simplecancel(int sig, siginfo_t *siginfo, void *context);
static void sighandler_ready(int sig, siginfo_t *siginfo, void *context);
static void sighandler_install( int sig, void (*sighandler)(int, siginfo_t *, void *) );

static void wait_ready( pid_t pid );

static void create_channels(void);
static void open_channels();

static void start_workers(void);

static void worker_pub(unsigned int seed);
static void worker_sub(void);
static void worker_sig(void);

static uint16_t
crc_16_itu_t( uint16_t crc, const uint8_t *buf, size_t n );

int main( int argc, char **argv ){
    int c;
    sigset_t blockset, emptyset;
    while( (c = getopt( argc, argv, "p:s:c:i:vuk")) != -1 ) {
        switch(c) {
        case 'p':
            opt_n_pub = (size_t)atoi(optarg);
            break;
        case 's':
            opt_n_sub = (size_t)atoi(optarg);
            break;
        case 'i':
            opt_n_sig = (size_t)atoi(optarg);
            break;
        case 'c':
            opt_n_chan =(size_t) atoi(optarg);
            break;
        case 'v':
            opt_verbosity++;
            break;
        case 'k':
            opt_map = ACH_MAP_KERNEL;
            break;
        case 'u':
            opt_map = ACH_MAP_USER;
            break;
        case '?':
        case 'h':
        case 'H':
            puts( "Usage: achstress [OPTION...]\n"
                  "Ach stress test.\n"
                  "Create many channels, fork many processes, send checksummed messages,\n"
                  "and validate the checksums."
                  "\n"
                  "\n"
                  "  -p PUBLISHER-COUNT    number of publishers\n"
                  "  -s SUBSCRIBER-COUNT   number of subscribers\n"
                  "  -c CHANNEL-COUNT      number of channels\n"
                  /* "  -i SIG-COUNT          number of interrupters\n" */
                  "  -k                    use kernel-mapped channels\n"
                  "  -u                    use user-space-mapped channels\n"
                  "  -?                    display this help and exit\n"
                );
            exit(EXIT_SUCCESS);
        }
    }

    DEBUG(1, "publishers:  %lu\n", opt_n_pub );
    DEBUG(1, "subscribers: %lu\n", opt_n_sub );
    DEBUG(1, "signallers:  %lu\n", opt_n_sig );
    DEBUG(1, "channels:    %lu\n", opt_n_chan );
    DEBUG(1, "map:         %d\n",  opt_map );

    create_channels();


    /* Set up signal handling */
    sighandler_install(SIGINT, sighandler_nop);
    sighandler_install(SIGCHLD, sighandler_nop);
    sighandler_install(SIGUSR1, sighandler_ready);
    check_errno( "sigemptyset", sigemptyset(&blockset) );
    check_errno( "sigemptyset", sigemptyset(&emptyset) );
    check_errno( "sigaddset",   sigaddset(&blockset, SIGCHLD) );
    check_errno( "sigaddset",   sigaddset(&blockset, SIGUSR1) );
    check_errno( "sigprocmask", sigprocmask(SIG_SETMASK, &blockset, NULL) );

    start_workers();

    /* Wait for signals */
    {
        unsigned exit_cnt = 0;
        int status;
        pid_t p;
        while(exit_cnt < (opt_n_pub + opt_n_sig + opt_n_sig ) ) {
            sigsuspend( &emptyset );
            if( EINTR != errno ) {
                perror("sigsuspend");
                goto FAIL;
            }
            while( 0 < (p = waitpid( -1, &status, WNOHANG) ))
            {
                DEBUG(1, "wait result: %d\n", p);
                /* TODO: validate PID */
                if( WIFSIGNALED(status) ) {
                    fprintf(stderr, "Exit by signal %s (%d), that shouldn't happen\n",
                            strsignal(WTERMSIG(status)), WTERMSIG(status));
                    goto FAIL;
                }
                if( WIFEXITED(status) ) {
                    exit_cnt++;
                    if( WEXITSTATUS(status) ) {
                        fprintf(stderr, "Pid %d died with %d\n",
                                p, WEXITSTATUS(status));
                        goto FAIL;
                    }
                }
            }
        }
        DEBUG(1, "finished, exit_cnt: %u\n", exit_cnt);
    }
    /* check worker return status */

    return 0;

FAIL:
    {
        size_t i = 0;
        while( i < opt_n_pub ) kill( g_pid_pub[i++], SIGTERM );
        i = 0;
        while( i < opt_n_sub ) kill( g_pid_sub[i++], SIGTERM );
    }
    return EXIT_FAILURE;
}


static void start_workers(void)
{
    size_t i;
    unsigned int seed = 42;
    pid_t parent = getpid();

    /* create subscribers */
    g_pid_sub = (pid_t*)malloc( opt_n_sub * sizeof(g_pid_sub[0]) );
    for( i = 0; i < opt_n_sub; i++ ) {
        g_pid_sub[i] = fork();
        CHECK_TRUE( "fork sub", g_pid_sub[i] >= 0 );
        if( 0 == g_pid_sub[i] ) worker_sub();
        else wait_ready(g_pid_sub[i]);
    }

    CHECK_TRUE("in parent", parent == getpid() );

    /* create publishers */
    g_pid_pub = (pid_t*)malloc( opt_n_pub * sizeof(g_pid_pub[0]) );
    for( i = 0; i < opt_n_pub; i++ ) {
        rand_r(&seed);
        g_pid_pub[i] = fork();
        CHECK_TRUE( "fork pub", g_pid_pub[i] >= 0 );
        if( 0 == g_pid_pub[i] ) worker_pub(seed);
        else wait_ready(g_pid_pub[i]);
    }

    CHECK_TRUE("in parent", parent == getpid() );

    /* create signallers */
    g_pid_sig = (pid_t*)malloc( opt_n_sig * sizeof(g_pid_sig[0]) );
    for( i = 0; i < opt_n_sig; i++ ) {
        g_pid_sig[i] = fork();
        CHECK_TRUE( "fork sig", g_pid_sig[i] >= 0 );
        if( 0 == g_pid_sig[i] ) worker_sig();
        else wait_ready(g_pid_sig[i]);
    }

    CHECK_TRUE("in parent", parent == getpid() );


    /* send start message */

    CHECK_ACH( "open", ach_open(&g_chan_begin, "achtorture-begin", NULL) );
    char buf[] = "begin";

    CHECK_ACH( "begin", ach_put( &g_chan_begin, buf, strlen(buf) ) );
}


static void wait_ready( pid_t pid )
{
    sigset_t emptyset;
    check_errno( "sigemptyset", sigemptyset(&emptyset) );
    DEBUG(1, "initial pid_ready: %d, child %d\n", g_pid_ready, pid)
    g_pid_ready = 0;
    sigsuspend( &emptyset );
    if( EINTR != errno ) {
        perror("sigsuspend");
        exit(EXIT_FAILURE);
    }
    if( g_pid_ready != pid ) {
        fprintf(stderr, "Child %d not ready after signal: %d\n",
                pid, g_pid_ready);
        exit(EXIT_FAILURE);
    }
    DEBUG(1, "final pid_ready: %d\n", g_pid_ready)
    g_pid_ready = 0;
}

static void sighandler_nop(int sig, siginfo_t *siginfo, void *context)
{
    (void) sig; (void) siginfo; (void)context;
    /* const char *msg = "foo\n"; */
    /* write(STDERR_FILENO, msg, strlen(msg) ); */
}


static void sighandler_ready(int sig, siginfo_t *siginfo, void *context)
{
    (void) sig; (void) context;
    g_pid_ready = siginfo->si_pid;
}

static void sighandler_simplecancel(int sig, siginfo_t *siginfo, void *context)
{
    (void) sig; (void) siginfo; (void)context;
    g_cancel = 1;
}

static void sighandler_cancel(int sig, siginfo_t *siginfo, void *context)
{
    size_t i=0;
    enum ach_status r;
    (void) sig; (void) siginfo; (void)context;

    g_cancel = 1;

    r = ach_cancel( &g_chan_begin, NULL );
    if( r != ACH_OK ) goto FAIL;

    while(i<opt_n_chan)
    {
        r = ach_cancel( &g_chans[i++], NULL );
        if( r != ACH_OK ) {
            goto FAIL;
        }
    }

    return;

FAIL:
    {
        const char *msg = "cancel failed\n";
        size_t n = strlen(msg);
        ssize_t ww = write(STDERR_FILENO, msg, n );
        exit( (ww==(ssize_t)n) ? -1 : -2 );
    }
}

void sighandler_install( int sig, void (*sighandler)(int, siginfo_t *, void *) )
{
    struct sigaction act;
    memset(&act, 0, sizeof(act));

    act.sa_sigaction = sighandler;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(sig, &act, NULL) < 0) {
        fprintf( stderr, "Couldn't install signal handler: %s", strerror(errno) );
        exit(EXIT_FAILURE);
    }
}



static void create_channels(void)
{
    size_t i = 0;
    for( i=0; i < opt_n_chan; i++ ) {
        char buf[ACH_CHAN_NAME_MAX] = {0};
        struct ach_create_attr attr;
        snprintf(buf, ACH_CHAN_NAME_MAX, "achtorture-%lu", i );

        CHECK_ACH_MASK( "unlink before create",
                        (ACH_MASK_OK | ACH_MASK_ENOENT),
                        ach_unlink(buf) );

        ach_create_attr_init( &attr );
        CHECK_ACH("set create map", ach_create_attr_set_map(&attr, opt_map) );

        CHECK_ACH( "create",
                   ach_create( buf, 0, 0, &attr ) );
    }

    CHECK_ACH_MASK( "unlink before create",
                    (ACH_MASK_OK | ACH_MASK_ENOENT),
                    ach_unlink("achtorture-begin") );
    CHECK_ACH( "create",
               ach_create( "achtorture-begin", 0, 0, NULL ) );

}

static void open_channels(void)
{
    g_chans = (struct ach_channel*)malloc( opt_n_chan * sizeof(g_chans[0]) );
    size_t i = 0;
    for( i=0; i < opt_n_chan; i++ ) {
        char buf[ACH_CHAN_NAME_MAX] = {0};
        snprintf(buf, ACH_CHAN_NAME_MAX, "achtorture-%lu", i );

        for(;;) {
            enum ach_status r = ach_open( &g_chans[i], buf, NULL );
            if( ACH_EACCES == r || ACH_ENOENT == r ) {
            /* Race against udev */
                usleep(1000);
                continue;
            } else if (ACH_OK == r) break;
            else fail_ach("ach_open", r);
        }
    }
    CHECK_ACH( "open", ach_open(&g_chan_begin, "achtorture-begin", NULL) );
}

static void worker_pub(unsigned int seed)
{
    int buf[32];
    size_t n_max = sizeof(buf) / sizeof(buf[0]) - 1;
    size_t frame_size;

    DEBUG(1, "worker_pub %d: start\n", getpid());

    sighandler_install(SIGUSR2, sighandler_nop);
    sighandler_install(SIGINT,  sighandler_cancel);
    sighandler_install(SIGTERM, sighandler_cancel);

    check_errno( "kill parent", kill(getppid(), SIGUSR1) );

    open_channels();

    DEBUG(1, "worker_pub %d: opened channels\n", getpid());

    CHECK_ACH_MASK( "get begin create", (ACH_MASK_OK | ACH_MASK_CANCELED),
                    ach_get( &g_chan_begin, buf, sizeof(buf), &frame_size,
                             NULL, ACH_O_WAIT ) );

    DEBUG(1, "worker_pub %d: got begin command\n", getpid());

    while( !g_cancel ) {
        /* Random data */
        size_t i,j;
        size_t n = (size_t)rand_r(&seed) % n_max;
        DEBUG(2, "pub n: %lu\n", n );
        for( i=0; i < n; i ++ ) {
            buf[i] = rand_r(&seed);
        }
        buf[n] = crc_16_itu_t( 0, (uint8_t*)buf, n*sizeof(buf[0]) );

        for( j=0; j < opt_n_chan; j++ ) {
            CHECK_ACH_MASK( "pub put", (ACH_MASK_OK | ACH_MASK_CANCELED),
                            ach_put( &g_chans[j], buf, (n+1)*sizeof(buf[0]) ) );
        }
    }
    exit(EXIT_SUCCESS);
}

static void worker_sub(void)
{
    DEBUG(1, "worker_sub %d\n", getpid());

    sighandler_install(SIGUSR2, sighandler_nop);
    sighandler_install(SIGINT, sighandler_cancel);
    sighandler_install(SIGTERM, sighandler_cancel);
    check_errno( "kill parent", kill(getppid(), SIGUSR1) );

    open_channels();

    for(;;) {
        size_t j;
        for( j=0; j < opt_n_chan; j++ ) {
            int buf[32];
            size_t frame_size;
            enum ach_status r;
            r = ach_get( &g_chans[j], buf, sizeof(buf), &frame_size, NULL,
                         ACH_O_WAIT | ACH_O_LAST );
            switch(r) {
            case ACH_CANCELED: exit(EXIT_SUCCESS);
            case ACH_OK:
            case ACH_MISSED_FRAME:
                /* TODO: validate */
                break;
            default:
                fail_ach( "subscriber ach_get", r );
            }

            {
                size_t n = frame_size / sizeof(int) - 1;
                unsigned csum = crc_16_itu_t( 0, (uint8_t*)buf, n * sizeof(int) );
                unsigned buf_cksum = (unsigned)buf[n];
                DEBUG(2, "sub frame_size: %lu\n", frame_size);
                DEBUG(2, "sub n:          %lu\n", n);
                CHECK_TRUE( "get checksum", buf_cksum == csum );

            }
        }
    }
}

static void worker_sig_kill(pid_t *p)
{
    if(*p) {
        int r =  kill(*p, SIGUSR2);
        if( r < 0 ) {
            /* It's ok if the sibling process doesn't exist. The
             * parent will get it's status.  We keep going. */
            if( ESRCH == errno ) *p = 0;
            else fail_errno("kill worker");
        }
    }
}

static void worker_sig(void)
{
    char buf[64] = {0};
    size_t frame_size;
    size_t i;

    sighandler_install(SIGINT, sighandler_simplecancel);
    sighandler_install(SIGTERM, sighandler_simplecancel);
    check_errno( "kill parent", kill(getppid(), SIGUSR1) );


    CHECK_ACH( "open", ach_open(&g_chan_begin, "achtorture-begin", NULL) );
    CHECK_ACH_MASK( "get begin create", (ACH_MASK_OK | ACH_MASK_CANCELED),
                    ach_get( &g_chan_begin, buf, sizeof(buf), &frame_size,
                             NULL, ACH_O_WAIT ) );
    while( !g_cancel ) {
        i = 0;
        while( i < opt_n_pub ) {
            worker_sig_kill( &g_pid_pub[i++] );
        }
        i = 0;
        while( i < opt_n_sub ) {
            worker_sig_kill( &g_pid_sub[i++] );
        }
    }

    exit(EXIT_SUCCESS);
}


/* static uint16_t crc_16_ccitt( uint16_t crc, const uint8_t *buf, size_t n ) { */
/*     while( n ) { */
/*         uint8_t d = *buf++; */
/*         uint8_t e = (uint8_t)crc ^ d; */
/*         uint8_t f = e ^ (uint8_t)(e << 4); */
/*         crc = (uint16_t)((crc >> 8) ^ */
/*                          (f << 8)   ^ */
/*                          (f << 3)   ^ */
/*                          (f >> 4)); */
/*         n--; */
/*     } */
/*     return crc; */
/* } */

static uint16_t crc_16_itu_t( uint16_t crc, const uint8_t *buf, size_t n ) {
    while( n ) {
        uint8_t d = *buf++;
        uint8_t e = d ^ (uint8_t)(crc >> 8);
        uint8_t f = e ^ (e >> 4);
        crc = (uint16_t)((crc << 8) ^
                          f         ^
                         (f <<  5)  ^
                         (f << 12));
        n--;
    }
    return crc;
}

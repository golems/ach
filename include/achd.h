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
#ifndef ACHD_H
#define ACHD_H

/** \file achd.h
 *
 * \brief This file contains declarations for the achd utility.
 *
 * \author Neil T. Dantam
 */

#define ACHD_PORT 8076
#define INIT_BUF_SIZE 512

#define ACHD_REFORK_NS 1000000000

#define ACHD_RECONNECT_NS (250 * 1000 * 1000)

#define ACHD_LINE_LENGTH 1024

#ifdef __GNUC__
#define ACHD_ATTR_PRINTF(m,n) __attribute__((format(printf, m, n)))
#else
#define ACHD_ATTR_PRINTF(m,n)
#endif


/* Prototypes */
enum achd_direction {
    ACHD_DIRECTION_VOID = 0,
    ACHD_DIRECTION_PUSH,
    ACHD_DIRECTION_PULL
};

enum achd_mode {
    ACHD_MODE_VOID = 0,
    ACHD_MODE_SERVE,
    ACHD_MODE_PUSH,
    ACHD_MODE_PULL
};

struct achd_headers {
    const char *chan_name;
    const char *remote_chan_name;
    int frame_count;
    int frame_size;
    int local_port;
    int remote_port;
    int tcp_nodelay;
    int retry;
    int get_last;
    int retry_delay_us;
    unsigned long period_ns;
    const char *remote_host;
    const char *transport;
    enum achd_direction direction;
    enum ach_status status;
    const char *message;
};

struct achd_conn;

typedef void (*achd_io_handler_t) (struct achd_conn*);
typedef int (*achd_io_connect_t) (struct achd_conn*);



struct achd_conn_vtab {
    const char *transport;
    enum achd_direction direction;
    achd_io_connect_t connect;
    achd_io_handler_t handler;
};

const struct achd_conn_vtab *achd_get_vtab( const char *transport, enum achd_direction direction );

struct achd_conn {
    enum achd_mode mode;

    struct timespec t0;
    struct timespec ts_last;
    struct achd_headers send_hdr;
    struct achd_headers recv_hdr;

    int in;   ///< input file descriptor  (stdin or TCP)
    int out;  ///< output file descriptor (stdout or TCP)
    int aux;  ///< auxillary file descriptor (e.g., UDP)

    size_t pipeframe_size;
    ach_pipe_frame_t *pipeframe;

    const struct achd_conn_vtab *vtab;

    void *cx;
};

int achd_reconnect( struct achd_conn *conn );


enum ach_status achd_parse_headers(int fd, struct achd_headers *headers);

void achd_serve(void);
void achd_client(void);

void achd_sleep_till( const struct timespec *t0, unsigned long ns );

/* logging and error handlers */
void achd_log( int level, const char fmt[], ...)          ACHD_ATTR_PRINTF(2,3);
void achd_error_header( enum ach_status code, const char fmt[], ... ) ACHD_ATTR_PRINTF(2,3);
void achd_error_log( enum ach_status code, const char fmt[], ... )    ACHD_ATTR_PRINTF(2,3);


/* basic i/o */
ssize_t achd_read(int fd, void *buf, size_t cnt );
ssize_t achd_write(int fd, const void *buf, size_t cnt );
enum ach_status achd_readline(int fd, char *buf, size_t n );
enum ach_status achd_printf(int fd, const char fmt[], ...) ACHD_ATTR_PRINTF(2,3);

/* i/o handlers */

int achd_connect_nop( struct achd_conn *conn );
int achd_udp_sock( struct achd_conn *conn );

void achd_push_tcp( struct achd_conn *);
void achd_pull_tcp( struct achd_conn *);
void achd_push_udp( struct achd_conn *);
void achd_pull_udp( struct achd_conn *);


struct achd_cx {
    struct achd_headers cl_opts; /** Options from command line */
    int mode;
    int port;
    int reconnect;
    int detach;
    const char *pidfile;
    sig_atomic_t sig_received;
    ach_channel_t channel;
    void (*error)(enum ach_status code, const char fmt[], ...);
};

extern struct achd_cx cx;
void sighandler_install(void);
extern const char *opt_posarg[2];

#endif //ACHD_H

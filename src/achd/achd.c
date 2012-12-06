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
#include <arpa/inet.h>
#include <netdb.h>

#include "ach.h"
#include "achutil.h"
#include "achd.h"


static void achd_posarg(int i, const char *arg);
static void achd_write_pid();

void achd_make_realtime();

/* signal handlers */
static void sighandler(int sig, siginfo_t *siginfo, void *context);

/* global data */
struct achd_cx cx;

static const struct {
    const char *transport;
    enum achd_direction direction;
    achd_io_handler_t handler;
} handlers[] = {
    {.transport = "tcp",
     .direction = ACHD_DIRECTION_PUSH,
     .handler = achd_push_tcp },
    {.transport = "tcp",
     .direction = ACHD_DIRECTION_PULL,
     .handler = achd_pull_tcp },
    {.transport = "udp",
     .direction = ACHD_DIRECTION_PUSH,
     .handler = achd_push_udp },
    {.transport = "udp",
     .direction = ACHD_DIRECTION_PULL,
     .handler = achd_pull_udp },
    {.transport = NULL,
     .direction = 0,
     .handler = NULL }
};

/* Main */
int main(int argc, char **argv) {
    achd_log( LOG_DEBUG, "achd started\n");

    /* set some defaults */
    memset( &cx, 0, sizeof(cx) );
    cx.cl_opts.transport = "tcp";
    cx.cl_opts.frame_size = ACH_DEFAULT_FRAME_SIZE;
    cx.cl_opts.frame_count = ACH_DEFAULT_FRAME_COUNT;
    cx.error = achd_error_log;
    cx.port = ACHD_PORT;

    /* process options */
    int c = 0, i = 0;
    while( -1 != c ) {
        while( (c = getopt( argc, argv, "dp:t:f:z:qrvV?")) != -1 ) {
            switch(c) {
            case 'z':
                cx.cl_opts.remote_chan_name = strdup(optarg);
                break;
            case 'd':
                cx.daemonize = 1;
                break;
            case 'p':
                cx.port = atoi(optarg);
                if( !optarg ) {
                    achd_log(LOG_ERR, "Invalid port: %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            case 'f':
                cx.pidfile = strdup(optarg);
                break;
            case 'r':
                cx.reconnect = 1;
                break;
            case 't':
                cx.cl_opts.transport = strdup(optarg);
                break;
            case 'q':
                cx.verbosity --;
                break;
            case 'v':
                cx.verbosity ++;
                break;
            case 'V':   /* version     */
                ach_print_version("achd");
                exit(EXIT_SUCCESS);
            case '?':
                puts( "Usage: achd [OPTIONS...] [serve|push|pull] [HOST  CHANNEL] \n"
                      "Daemon process to forward ach channels over network and dump to files\n"
                      "\n"
                      "Options:\n"
                      "  -d,                          daemonize (client-mode only)\n"
                      "  -p PORT,                     port\n"
                      "  -f FILE,                     TODO: lock FILE and write pid\n"
                      "  -t (tcp|udp),                TODO: transport (default tcp)\n"
                      "  -z CHANNEL_NAME,             remote channel name\n"
                      "  -r,                          reconnect if connection is lost\n"
                      "  -q,                          be quiet\n"
                      "  -v,                          be verbose\n"
                      "  -V,                          version\n"
                      "  -?,                          show help\n"
                      "\n"
                      "Files:\n"
                      "  /etc/inetd.conf              Use to enable network serving of ach channels.\n"
                      "                               Use a line like this:\n"
                      "                               '8076  stream  tcp  nowait  nobody  /usr/bin/achd  /usr/bin/achd serve'\n"
                      "\n"
                      "Examples:\n"
                      "  achd serve                   Server process reading from stdin/stdout.\n"
                      "                               This can be run from inetd.\n"
                      "  achd pull golem state-chan   Forward frames via TCP from remote channel\n"
                      "                               'state-chan' on host 'golem' to local channel\n"
                      "                               (a pull from the remote server).\n"
                      "                               An achd server must be listening on the remote\n"
                      "                               host.\n"
                      "  achd -r push golem cmd-chan  Forward frames via TCP from local channel\n"
                      "                               'cmd-chan' to remote channel on host 'golem'\n"
                      "                               (a push to the remote server).\n"
                      "                               Retry dropped connections.\n"
                      "                               An achd server must be listening the remote host.\n"
                      "\n"
                      "Report bugs to <ntd@gatech.edu>"
                       );

                exit(EXIT_SUCCESS);
                break;
            default:
                achd_posarg(i++, optarg);
                break;
            }
        }
        while( optind < argc ) {
            achd_posarg(i++, argv[optind++]);
        }
    }

    /* Set some other default options */
    if( !cx.cl_opts.chan_name ) {
        cx.cl_opts.chan_name  = cx.cl_opts.remote_chan_name;
    }
    if( !cx.cl_opts.remote_chan_name ) {
        cx.cl_opts.remote_chan_name  = cx.cl_opts.chan_name;
    }

    /* dispatch based on mode */
    /* serve */
    if ( ACHD_MODE_SERVE == cx.mode ) {
        if( isatty(STDIN_FILENO) || isatty(STDOUT_FILENO) ) {
            achd_log(LOG_ERR, "We don't serve TTYs here!\n");
            exit(EXIT_FAILURE);
        }
        cx.error = achd_error_header;
        achd_serve();
        return 0;
    } else {
        achd_client();
        return 0;
    }

    /* maybe print stuff */
    /* if( cx.verbosity ) { */
    /*     fprintf( stderr, "channel     = %s\n", cx.cl_opts.chan_name ); */
    /*     /\* fprintf( stderr, "config file = %s\n", opt_config_file); *\/ */
    /* } */

    /* /\* parse config file *\/ */
    /* FILE *fconfig = fopen(opt_config_file, "r" ); */
    /* struct achd_config config; */

    /* memset(&config,0,sizeof(config)); */
    /* if( fconfig ) { */
    /*     achd_parse_config(fconfig, &config ); */
    /*     fclose(fconfig); */
    /* } else { */
    /*     fprintf(stderr, "Couldn't open config file %s\n", opt_config_file); */
    /* } */


    /* if( opt_verbosity >= 2 ) { */
    /*     fprintf(stderr, */
    /*             "channel-name = %s\n" */
    /*             "frame-size = %d\n" */
    /*             "frame-count = %d\n" */
    /*             "transport = %s\n" */
    /*             "direction = %s\n" */
    /*             "remote-host = %s\n" */
    /*             "remote-port = %d\n" */
    /*             "local-port = %d\n" */
    /*             "tcp-nodelay = %d\n", */
    /*             config.chan_name, */
    /*             config.frame_size, */
    /*             config.frame_count, */
    /*             config.transport, */
    /*             ((ACHD_DIRECTION_PUSH == config.direction) ? "PUSH" : */
    /*              ((ACHD_DIRECTION_PULL == config.direction) ? "PULL" : "unknown")), */
    /*             config.remote_host, config.remote_port, config.local_port, */
    /*             config.tcp_nodelay ); */
    /* } */

    return 0;
}

void achd_posarg(int i, const char *arg) {
    switch(i) {
    case 0:
        achd_log(LOG_DEBUG, "mode %s\n", arg);
        if( 0 == strcasecmp(arg, "serve") ) {
            cx.mode = ACHD_MODE_SERVE;
        } else if( 0 == strcasecmp(arg, "push") ) {
            cx.mode = ACHD_MODE_PUSH;
            cx.cl_opts.direction = ACHD_DIRECTION_PUSH;
        } else if( 0 == strcasecmp(arg, "pull") ) {
            cx.mode = ACHD_MODE_PULL;
            cx.cl_opts.direction = ACHD_DIRECTION_PULL;
        } else {
            achd_log(LOG_ERR, "Invalid command: %s\n", arg);
            exit(EXIT_FAILURE);
        }
        break;
    case 1:
        achd_log(LOG_DEBUG, "host %s\n", arg);
        cx.cl_opts.remote_host = strdup(arg);
        break;
    case 2:
        achd_log(LOG_DEBUG, "channel %s\n", arg);
        cx.cl_opts.chan_name = strdup(arg);
        break;
    default:
        achd_log(LOG_ERR, "Spurious argument: %s\n", arg);
        exit(EXIT_FAILURE);
    }
}

/*********
* Server *
*********/

void achd_serve() {
    sighandler_install();

    /* Get peer */
    struct sockaddr_in addr;
    {
        socklen_t len = sizeof(addr);
        if( getpeername( STDIN_FILENO, (struct sockaddr *) &addr, &len ) ) {
            cx.error( ACH_FAILED_SYSCALL, "Server couldn't determine name of peer: %s\n", strerror(errno));
        }
    }

    struct achd_conn conn;
    memset( &conn, 0, sizeof(conn) );
    conn.in = STDIN_FILENO;
    conn.out = STDOUT_FILENO;
    conn.mode = ACHD_MODE_SERVE;
    {
        enum ach_status r = achd_parse_headers( conn.in, &conn.request );
        if( ACH_OK != r ) {
            cx.error(r, "Bad headers\n");
        }
    }

    /* check transport headers */
    if( !conn.request.chan_name ) conn.request.chan_name = conn.request.remote_chan_name;

    if( !conn.request.chan_name ) {
        cx.error( ACH_BAD_HEADER, "%s:%d no channel header\n", inet_ntoa(addr.sin_addr), addr.sin_port);
    } else if( ! conn.request.transport ) {
        cx.error( ACH_BAD_HEADER, "%s:%d no transport header\n", inet_ntoa(addr.sin_addr), addr.sin_port);
    } else if( !((ACHD_DIRECTION_PULL == conn.request.direction) ||
                 (ACHD_DIRECTION_PUSH == conn.request.direction)) )
    {
        cx.error( ACH_BAD_HEADER, "%s:%d no direction header\n", inet_ntoa(addr.sin_addr), addr.sin_port);
    } else {
        achd_log( LOG_NOTICE, "serving %s:%d channel %s via %s %s\n",
                  inet_ntoa(addr.sin_addr), addr.sin_port, conn.request.chan_name,
                  conn.request.transport,
                  (ACHD_DIRECTION_PUSH == conn.request.direction) ? "push" : "pull" );
    }

    /* open channel */
    {
        int r = ach_open( &conn.channel, conn.request.chan_name, NULL );
        if( ACH_OK != r ) {
            cx.error( r, "Couldn't open channel %s - %s\n", conn.request.chan_name, strerror(errno) );
            assert(0);
        } else {
            ach_flush(&conn.channel);
        }
    }

    /* dispatch to the requested mode */
    conn.handler = achd_get_handler( conn.request.transport,
                                     conn.request.direction );
    assert( conn.handler );

    /* print headers */
    achd_printf(conn.out,
                "frame-count: %" PRIuPTR "\n"
                "frame-size: %" PRIuPTR "\n"
                "status: %d # %s\n"
                ".\n",
                conn.channel.shm->index_cnt,
                conn.channel.shm->data_size / conn.channel.shm->index_cnt,
                ACH_OK, ach_result_to_string(ACH_OK)
        );

    /* Set error handler */
    cx.error = achd_error_log;

    /* start i/o */
    conn.handler( &conn );
    achd_log( LOG_INFO, "Finished serving %s:%d\n", inet_ntoa(addr.sin_addr), addr.sin_port );
    return;
}


/**********
* HEADERS *
**********/
static void achd_set_header
(const char *key, const char *val, struct achd_headers *headers);
static void achd_set_int(int *pint, const char *name, const char *val);

#define REGEX_WORD "([^:=\n]*)"
#define REGEX_SPACE "[[:blank:]\n\r]*"

enum ach_status achd_parse_headers(int fd, struct achd_headers *headers) {
    regex_t line_regex, dot_regex;
    regmatch_t match[3];
    if (regcomp(&line_regex,
                "^"REGEX_SPACE"$|"     /* empty line */
                "^"REGEX_SPACE         /* beginning space */
                REGEX_WORD             /* key */
                REGEX_SPACE            /* mid space */
                "[:=]"
                REGEX_SPACE            /* mid space */
                REGEX_WORD             /* value */
                REGEX_SPACE            /* end space */
                "$",
                REG_EXTENDED ) ) {
        cx.error(ACH_BUG, "couldn't compile regex\n");
        assert(0);
    }
    if( regcomp(&dot_regex,
                "^"REGEX_SPACE "." REGEX_SPACE "$",
                REG_EXTENDED) ) {
        cx.error(ACH_BUG, "couldn't compile regex\n");
        assert(0);
    }

    int line = 0;
    size_t n = ACHD_LINE_LENGTH;
    char lineptr[n];
    ssize_t r;
    while( ACH_OK == (r = achd_readline(fd, lineptr, n)) ) {
        line++;
        /* Break on ".\n" */
        if (! regexec(&dot_regex, lineptr, 0, NULL, 0)) break;
        achd_log(LOG_DEBUG, "header line %d: %s\n", line, lineptr);
        /* kill comments and translate */
        char *cmt = strchr(lineptr, '#');
        if( cmt ) *cmt = '\0';
        /* match key/value */
        int i = regexec(&line_regex, lineptr, sizeof(match)/sizeof(match[0]), match, 0);
        if( i ) {
            cx.error( ACH_BAD_HEADER, "malformed header\n");
            assert(0);
        }
        assert( ! strchr(lineptr, '#') );
        if( match[1].rm_so >= 0 && match[2].rm_so >=0 ) {
            lineptr[match[1].rm_eo] = '\0';
            lineptr[match[2].rm_eo] = '\0';
            char *key = lineptr+match[1].rm_so;
            char *val = lineptr+match[2].rm_so;
            achd_log( LOG_DEBUG, "header line %d parsed `%s' : `%s'\n", line, key, val );
            achd_set_header(key, val, headers);
        }

    }
    regfree(&line_regex);
    regfree(&dot_regex);
    return r;
}

void achd_set_int(int *pint, const char *name, const char *val) {
    errno = 0;
    long i = strtol( val, NULL, 10 );
    if( errno ) {
        cx.error( ACH_BAD_HEADER, "Invalid %s %s: %s\n", name, val, strerror(errno) );
        assert(0);
    }
    *pint = (int) i;
}

int achd_parse_boolean( const char *value ) {
    const char *yes[] = {"yes", "true", "1", "t", "y", "+", "aye", NULL};
    const char *no[] = {"no", "false", "0", "f", "n", "-", "nay", NULL};
    const char** s;
    for( s = yes; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 1;
    for( s = no; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 0;
    cx.error(ACH_BAD_HEADER, "Invalid boolean: %s\n", value);
    assert(0);
}

void achd_set_header (const char *key, const char *val, struct achd_headers *headers) {
    if       ( 0 == strcasecmp(key, "channel-name")) {
        headers->chan_name = strdup(val);
    } else if( 0 == strcasecmp(key, "frame-size")) {
        achd_set_int( &headers->frame_size, "frame size", val );
    } else if( 0 == strcasecmp(key, "frame-count")) {
        achd_set_int( &headers->frame_count, "frame count", val );
    } else if( 0 == strcasecmp(key, "remote-port")) {
        achd_set_int( &headers->remote_port, "remote port", val );
    } else if( 0 == strcasecmp(key, "local-port")) {
        achd_set_int( &headers->local_port, "local port", val );
    } else if( 0 == strcasecmp(key, "remote-host")) {
        headers->remote_host = strdup(val);
    } else if( 0 == strcasecmp(key, "transport")) {
        headers->transport = strdup(val);
    } else if( 0 == strcasecmp(key, "tcp-nodelay")) {
        headers->tcp_nodelay = achd_parse_boolean( val );
    } else if( 0 == strcasecmp(key, "retry")) {
        headers->retry = achd_parse_boolean( val );
    } else if( 0 == strcasecmp(key, "direction")) {
        if( 0 == strcasecmp(val, "push") ) headers->direction = ACHD_DIRECTION_PUSH;
        else if( 0 == strcasecmp(val, "pull") ) headers->direction = ACHD_DIRECTION_PULL;
        else {
            cx.error( ACH_BAD_HEADER, "Invalid direction: %s\n", val);
            assert(0);
        }
    } else if ( 0 == strcasecmp(key, "status") ) {
        achd_set_int( &headers->status, "status", val );
    } else if ( 0 == strcasecmp(key, "message") ) {
        headers->message = strdup(val);
    } else {
        cx.error( ACH_BAD_HEADER, "Invalid header: %s\n", key );
    }
}


/* handler definitions */
achd_io_handler_t achd_get_handler( const char *transport, enum achd_direction direction ) {
    /* check transport headers */
    if( ! transport ) {
        cx.error( ACH_BAD_HEADER, "No transport header");
        assert(0);
    } else if( ! direction ) {
        cx.error( ACH_BAD_HEADER, "No direction header");
        assert(0);
    } else {
        int i = 0;
        for( i = 0; handlers[i].transport; i ++ ) {
            if( direction ==  handlers[i].direction &&
                0 == strcasecmp( handlers[i].transport, transport ) )
            {
                return handlers[i].handler;
            }
        }
    }
    /* Couldn't find handler */
    cx.error( ACH_BAD_HEADER, "Requested transport or direction not found");
    assert(0);
}


/* error handlers */

void achd_exit_failure( int code ) {
    assert( 0 == ACH_OK );
    exit( code ? code : EXIT_FAILURE );
}

/* void achd_error_interactive( int code, const char fmt[], ... ) { */
/*     if( cx.verbosity >= 0 ) { */
/*         va_list argp; */
/*         va_start( argp, fmt ); */
/*         if( ACH_OK != code ) { */
/*             fprintf(stderr, "status: %s\n", ach_result_to_string(code)); */
/*         } */
/*         vfprintf( stderr, fmt, argp ); */
/*         va_end( argp ); */
/*     } */
/*     achd_exit_failure(code); */
/* } */



void achd_error_vsyslog( int code, const char fmt[], va_list argp ) {
    if( ACH_OK != code ) {
        const char *scode = ach_result_to_string(code);
        char fmt_buf[ strlen(scode) + 3 + strlen(fmt) + 1 ];
        strcpy( fmt_buf, scode );
        strcat( fmt_buf, " - " );
        strcat( fmt_buf, fmt );
        vsyslog( LOG_CRIT, fmt_buf, argp );
    } else {
        vsyslog(LOG_CRIT, fmt, argp);
    }
}

void achd_error_header( int code, const char fmt[], ... ) {
    va_list argp;
    /* Log */
    va_start( argp, fmt );
    achd_error_vsyslog( code, fmt, argp );
    va_end( argp );

    /* TODO: be smarter about the FILE* here*/
    /* Header */
    fprintf(stdout,
            "status: %d # %s\n"
            "message: ",
            code, ach_result_to_string(code));
    va_start( argp, fmt );
    vfprintf( stdout, fmt, argp );
    fprintf(stdout,"\n.\n");
    fflush( stdout );
    va_end( argp );

    achd_exit_failure(code);
}

/* void achd_error_syslog( int code, const char fmt[],  ...) { */
/*     va_list argp; */
/*     va_start( argp, fmt ); */
/*     achd_error_vsyslog( code, fmt, argp ); */
/*     va_end( argp ); */
/*     achd_exit_failure(code); */
/* } */

void achd_error_log( int code, const char fmt[],  ...) {
    int tty = isatty(STDERR_FILENO);
    if( tty && cx.verbosity >= -1) {
        if( ACH_OK != code ) {
            fprintf(stderr, "status: %s\n", ach_result_to_string(code));
        }
        va_list argp;
        va_start( argp, fmt );
        vfprintf(stderr, fmt, argp );
        fflush(stderr);
        va_end( argp );
    }
    if( !tty || ACHD_MODE_SERVE == cx.mode || 1 == getppid() ) {
        va_list argp;
        va_start( argp, fmt );
        achd_error_vsyslog( code, fmt, argp );
        va_end( argp );
    }
    exit(EXIT_FAILURE);
}

void achd_log( int level, const char fmt[], ...) {

    switch( level ) {
    case LOG_EMERG:
        goto dolog;
    case LOG_ALERT:
        goto dolog;
    case LOG_CRIT:
        goto dolog;
    case LOG_ERR:
        goto dolog;
    case LOG_WARNING:
        if( cx.verbosity >= -1 ) goto dolog;
        else return;
    case LOG_NOTICE:
        if( cx.verbosity >= 0 ) goto dolog;
        else return;
    case LOG_INFO:
        if( cx.verbosity >= 1 ) goto dolog;
        else return;
    case LOG_DEBUG:
        if( cx.verbosity >= 2 ) goto dolog;
        else return;
    default: assert(0);
    }

    int tty;
dolog:
    tty = isatty(STDERR_FILENO);
    if( tty ) {
        va_list argp;
        va_start( argp, fmt );
        vfprintf(stderr, fmt, argp );
        fflush(stderr);
        va_end( argp );
    }
    if( !tty || ACHD_MODE_SERVE == cx.mode || 1 == getppid() ) {
        va_list argp;
        va_start( argp, fmt );
        vsyslog(level, fmt, argp);
        va_end( argp );
    }
}

/* TODO:
 *   For client daemons, have SIGHUP kill and restart the subprocess.
 */

static void sighandler(int sig, siginfo_t *siginfo, void *context) {
    (void)siginfo; (void)context;
    achd_log( LOG_DEBUG, "Received signal: %d\n", sig );
    switch(sig) {
    case SIGTERM:
    case SIGINT:
        cx.sig_received = 1;
        break;
    default:
        achd_log( LOG_WARNING, "Received unexpected signal: %d\n", sig );
    }
}

/** setup the signal handler */
void sighandler_install() {
    struct sigaction act;
    memset(&act, 0, sizeof(act));

    act.sa_sigaction = &sighandler;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(SIGTERM, &act, NULL) < 0) {
        achd_log( LOG_ERR, "Couldn't install signal handler: %s", strerror(errno) );
    }

    if (sigaction(SIGINT, &act, NULL) < 0) {
        achd_log( LOG_ERR, "Couldn't install signal handler: %s", strerror(errno) );
    }

    if( SIG_ERR == signal(SIGPIPE, SIG_IGN) ) {
        achd_log( LOG_ERR, "Couldn't ignore SIGPIPE: %s", strerror(errno) );
    }
}

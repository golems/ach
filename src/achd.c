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
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <ctype.h>
#include <signal.h>
#include <regex.h>
#include <assert.h>
#include <stdarg.h>
#include <syslog.h>

#include "ach.h"
#include "achutil.h"

#define ACHD_PORT 8076

/* Prototypes */
enum achd_direction {
    ACHD_DIRECTION_VOID = 0,
    ACHD_DIRECTION_PUSH,
    ACHD_DIRECTION_PULL
};

struct achd_headers {
    const char *chan_name;
    int frame_count;
    int frame_size;
    int local_port;
    int remote_port;
    int tcp_nodelay;
    int retry;
    int retry_delay_us;
    const char *remote_host;
    const char *transport;
    enum achd_direction direction;
};

void achd_make_realtime();
void achd_daemonize();
void achd_parse_headers(FILE *fptr, struct achd_headers *headers);
void achd_set_header (const char *key, const char *val, struct achd_headers *headers);
void achd_write_pid();
void achd_set_int(int *pint, const char *name, const char *val);

void achd_serve();
void achd_open( ach_channel_t *channel, const struct achd_headers *headers );

/* error handlers */
void achd_error_interactive( int code, const char fmt[], ... );
void achd_error_header( int code, const char fmt[], ... );
void achd_error_syslog( int code, const char fmt[],  ...);

/* i/o handlers */
void achd_push_tcp( const struct achd_headers *headers, ach_channel_t *channel );
void achd_pull_tcp( const struct achd_headers *headers, ach_channel_t *channel );
void achd_push_udp( const struct achd_headers *headers, ach_channel_t *channel );
void achd_pull_udp( const struct achd_headers *headers, ach_channel_t *channel );

/* global data */
struct {
    struct achd_headers cl_opts; /** Options from command line */
    int serve;
    int verbosity;
    int daemonize;
    int port;
    const char *pidfile;
    sig_atomic_t received_sigterm;
    void (*error)(int code, const char fmt[], ...);
} cx;

struct achd_handler {
    const char *transport;
    enum achd_direction direction;
    void (*handler)( const struct achd_headers *headers, ach_channel_t *channel );
};

struct achd_handler handlers[] = {
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

    /* set some defaults */
    cx.cl_opts.transport = "tcp";
    cx.cl_opts.frame_size = ACH_DEFAULT_FRAME_SIZE;
    cx.cl_opts.frame_count = ACH_DEFAULT_FRAME_COUNT;
    cx.serve = 1;
    cx.error = achd_error_interactive;

    /* process options */
    int c = 0;
    while( -1 != c ) {
        while( (c = getopt( argc, argv, "S:P:dp:t:f:qvV?")) != -1 ) {
            switch(c) {
            case 'S':
                cx.cl_opts.remote_host = strdup(optarg);
                cx.cl_opts.direction = ACHD_DIRECTION_PUSH;
                cx.serve = 0;
                break;
            case 'P':
                cx.cl_opts.remote_host = strdup(optarg);
                cx.cl_opts.direction = ACHD_DIRECTION_PULL;
                cx.serve = 0;
                break;
            case 'd':
                cx.daemonize = 1;
                break;
            case 'p':
                cx.port = atoi(optarg);
                if( !optarg ) {
                    fprintf(stderr, "Invalid port: %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            case 'f':
                cx.pidfile = strdup(optarg);
                break;
            case 't':
                cx.cl_opts.transport = strdup(optarg);
                break;
            case 'q':
                cx.verbosity ++;
                break;
            case 'v':
                cx.verbosity --;
                break;
            case 'V':   /* version     */
                ach_print_version("achd");
                exit(EXIT_SUCCESS);
            case '?':
                puts( "Usage: achd [OPTIONS...] CHANNEL-NAME\n"
                      "Daemon process to forward ach channels over network and dump to files\n"
                      "\n"
                      "Options:\n"
                      "  -S HOST,                    push messages to HOST\n"
                      "  -P HOST,                    pull messages from HOST\n"
                      "  -d,                         daemonize (client-mode only)\n"
                      "  -p PORT,                    port\n"
                      "  -f FILE,                    lock FILE and write pid\n"
                      "  -t (tcp|udp),               transport (default tcp)\n"
                      "  -q,                         be quiet\n"
                      "  -v,                         be verbose\n"
                      "  -V,                         version\n"
                      "  -?,                         show help\n"
                      "\n"
                      "Files:\n"
                      "  /etc/inetd.conf             Use to enable network serving of ach channels.\n"
                      "                              Use a line like this:\n"
                      "                              '8075  stream  tcp  nowait  nobody  /usr/bin/achd  /usr/bin/achd'\n"
                      "\n"
                      "Examples:\n"
                      "  achd                        Server process reading from stdin/stdout.\n"
                      "                              This can be run from inetd\n"
                      "  achd -S golem cmd-chan      Forward frames via TCP from local channel\n"
                      "                              'cmd-chan' to remote channel on host 'golem'.\n"
                      "                              An achd server must be listening the remote host.\n"
                      "  achd -P golem state-chan    Forward frames via TCP from remote channel\n"
                      "                              'state-chan' on host golem to local channel 'cmd'.\n"
                      "                              An achd server must be listening on the remote\n"
                      "                              host.\n"
                      "\n"
                      "Report bugs to <ntd@gatech.edu>"
                       );

                exit(EXIT_SUCCESS);
                break;
            default:
                cx.cl_opts.chan_name = strdup(optarg);
                break;
            }
        }
        if( optind < argc ) {
            if( cx.cl_opts.chan_name ) {
                fprintf(stderr, "Multiple channel names given\n");
                exit(EXIT_FAILURE);
            }
            cx.cl_opts.chan_name = strdup(argv[optind]);
        }
    }

    /* dispatch based on mode */
    /* serve */
    if ( cx.serve ) {
        if( isatty(STDIN_FILENO) || isatty(STDOUT_FILENO) ) {
            /*fprintf(stderr, "We don't serve TTYs here!\n");*/
            /*exit(EXIT_FAILURE);*/
        }
        cx.error = achd_error_header;
        achd_serve();
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

/* Defs */

void achd_serve() {
    struct achd_headers srv_headers;
    memset( &srv_headers, 0, sizeof(srv_headers) );
    achd_parse_headers( stdin, &srv_headers );
    /* open channel */
    ach_channel_t channel;
    achd_open( &channel, &srv_headers );

    /* check transport headers */
    if( ! srv_headers.transport ) {
        cx.error( ACH_BAD_HEADER, "No transport header");
        assert(0);
    }
    if( ! srv_headers.direction ) {
        cx.error( ACH_BAD_HEADER, "No direction header");
        assert(0);
    }
    /* dispatch to the requested mode */
    size_t i;
    for( i = 0; handlers[i].transport; i ++ ) {
        if( srv_headers.direction ==  handlers[i].direction &&
            0 == strcasecmp( handlers[i].transport, srv_headers.transport ) )
        {
            /* print headers */
            fprintf(stdout,
                    "status: %d # %s\n"
                    ".\n",
                    ACH_OK, ach_result_to_string(ACH_OK)
                );
            fflush(stdout);

            /* start i/o */
            return handlers[i].handler( &srv_headers, &channel );
        }
    }
    /* Couldn't find handler */
    cx.error( ACH_BAD_HEADER, "Requested transport or direction not found");
    assert(0);
}

#define REGEX_WORD "([[:alnum:]_\\-]*)"
#define REGEX_SPACE "[ \t\n\r]*"

void achd_parse_headers(FILE *fptr, struct achd_headers *headers) {
    regex_t line_regex, dot_regex;
    regmatch_t match[3];
    if (regcomp(&line_regex,
                "^"REGEX_SPACE"$|"     /* empty line */
                "^"REGEX_SPACE         /* beginning space */
                REGEX_WORD             /* key */
                REGEX_SPACE            /* mid space */
                ":"
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
    size_t n = 1024;
    char *lineptr = (char*)malloc(n);
    ssize_t r;
    while( (r = getline(&lineptr, &n, fptr)) > 0 ) {
        line++;
        /* Break on ".\n" */
        if (! regexec(&dot_regex, lineptr, 0, NULL, 0)) break;
        if( cx.verbosity >= 3 ) printf("line %d: `%s'", line, lineptr);
        /* kill comments and translate */
        char *cmt = strchr(lineptr, '#');
        if( cmt ) *cmt = '\0';
        /* match key/value */
        int i = regexec(&line_regex, lineptr, sizeof(match)/sizeof(match[0]), match, 0);
        if( i ) {
            cx.error( ACH_BAD_HEADER, "malformed header\n");
            assert(0);
        }
        if( match[1].rm_so >= 0 && match[2].rm_so >=0 ) {
            lineptr[match[1].rm_eo] = '\0';
            lineptr[match[2].rm_eo] = '\0';
            char *key = lineptr+match[1].rm_so;
            char *val = lineptr+match[2].rm_so;
            if( cx.verbosity >= 3 ) printf("`%s' : `%s'\n", key, val );
            achd_set_header(key, val, headers);
        }

    }
    regfree(&line_regex);
    regfree(&dot_regex);
    free(lineptr);
}

void achd_set_int(int *pint, const char *name, const char *val) {
    *pint = atoi(val);
    if( !*pint ) {
        fprintf(stderr, "Invalid %s %s\n", name, val);
        exit(EXIT_FAILURE);
    }
}

int achd_parse_boolean( const char *value ) {
    const char *yes[] = {"yes", "true", "1", "t", "y", "+", "aye", NULL};
    const char *no[] = {"no", "false", "0", "f", "n", "-", "nay", NULL};
    const char** s;
    for( s = yes; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 1;
    for( s = no; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 0;
    fprintf(stderr, "Invalid boolean: %s\n", value);
    exit(EXIT_FAILURE);
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
    } else {
        fprintf(stderr, "Invalid configuration key: %s\n", key );
    }
}


void achd_open( ach_channel_t *channel, const struct achd_headers *headers ) {
    if( headers->chan_name ) {
        int r = ach_open( channel, headers->chan_name, NULL );
        if( ACH_OK != r)
            cx.error( r, "Couldn't open channel %s\n", headers->chan_name );
    } else {
        cx.error( ACH_BAD_HEADER, "No channel name header");
    }
}


/* handler definitions */
void achd_push_tcp( const struct achd_headers *headers, ach_channel_t *channel ) {
    assert(0); /* unimplemented */
}

void achd_pull_tcp( const struct achd_headers *headers, ach_channel_t *channel ) {
    assert(0); /* unimplemented */
}

void achd_push_udp( const struct achd_headers *headers, ach_channel_t *channel ) {
    /* check port set */
    assert(0); /* unimplemented */
}

void achd_pull_udp( const struct achd_headers *headers, ach_channel_t *channel ) {
    assert(0); /* unimplemented */
}


/* error handlers */

void achd_exit_failure( int code ) {
    assert( 0 == ACH_OK );
    exit( code ? code : EXIT_FAILURE );
}

void achd_error_interactive( int code, const char fmt[], ... ) {
    if( cx.verbosity >= 0 ) {
        va_list argp;
        va_start( argp, fmt );
        if( ACH_OK != code ) {
            fprintf(stderr, "status: %s\n", ach_result_to_string(code));
        }
        vfprintf( stderr, fmt, argp );
        va_end( argp );
    }
    achd_exit_failure(code);
}

void achd_error_vsyslog( int code, const char fmt[], va_list argp ) {
    if( ACH_OK != code ) {
        const char *scode = ach_result_to_string(code);
        char fmt_buf[ strlen(scode) + 3 + strlen(fmt) + 1 ];
        strcpy( fmt_buf, scode );
        strcat( fmt_buf, " - " );
        strcat( fmt_buf, fmt );
        vsyslog( LOG_ERR, fmt_buf, argp );
    } else {
        vsyslog(LOG_ERR, fmt, argp);
    }
}

void achd_error_header( int code, const char fmt[], ... ) {
    va_list argp;
    va_start( argp, fmt );
    achd_error_vsyslog( code, fmt, argp );
    va_end( argp );

    va_start( argp, fmt );
    fprintf(stdout, "status: %d # %s - ", code, ach_result_to_string(code) );
    vfprintf( stdout, fmt, argp );
    va_end( argp );
    achd_exit_failure(code);
}

void achd_error_syslog( int code, const char fmt[],  ...) {
    va_list argp;
    va_start( argp, fmt );
    achd_error_vsyslog( code, fmt, argp );
    va_end( argp );
    achd_exit_failure(code);
}

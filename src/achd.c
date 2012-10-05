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
#include <regex.h>

#include "ach.h"

/* Prototypes */
enum achd_direction {
    DIRECTION_VOID = 0,
    DIRECTION_PUSH,
    DIRECTION_PULL
};

struct achd_config {
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
void achd_parse_config(FILE *fptr, struct achd_config *config);
void achd_set_config (const char *key, const char *val, struct achd_config *config);
void achd_write_pid();
void achd_push_tcp();
void achd_pull_tcp();
void achd_push_udp();
void achd_pull_udp();

void achd_help(FILE *fptr);

struct achd_config chan_config = {
    .chan_name = 0,
    .frame_count = ACH_DEFAULT_FRAME_COUNT,
    .frame_size = ACH_DEFAULT_FRAME_SIZE
};


static const char *opt_config_file = "/etc/ach.conf";
static const char *opt_channel = NULL;
static int opt_verbosity = 0;

/* Main */
int main(int argc, char **argv) {
    // process options
    int c = 0;
    while( -1 != c ) {
        while( (c = getopt( argc, argv, "vc:thH?")) != -1 ) {
            switch(c) {
            case 'c':
                opt_config_file = strdup(optarg);
                break;
            case 'v':
                opt_verbosity ++;
                break;
            case 'h':
            case 'H':
            case '?':
                achd_help(stdout);
                exit(EXIT_SUCCESS);
                break;
            default:
                opt_channel = strdup(optarg);
                break;
            }
        }
        if( optind < argc ) {
            if( opt_channel ) {
                fprintf(stderr, "Multiple channel names given\n");
                exit(EXIT_FAILURE);
            }
            opt_channel = strdup(argv[optind]);
        }
    }

    // maybe print stuff
    if( opt_verbosity ) {
        fprintf( stderr, "channel     = %s\n", opt_channel);
        fprintf( stderr, "config file = %s\n", opt_config_file);
    }

    // parse config file
    FILE *fconfig = fopen(opt_config_file, "r" );
    struct achd_config config;

    memset(&config,0,sizeof(config));
    if( fconfig ) {
        achd_parse_config(fconfig, &config );
        fclose(fconfig);
    } else {
        fprintf(stderr, "Couldn't open config file %s\n", opt_config_file);
    }


    if( opt_verbosity >= 2 ) {
        fprintf(stderr,
                "channel-name = %s\n"
                "frame-size = %d\n"
                "frame-count = %d\n"
                "transport = %s\n"
                "direction = %s\n"
                "remote-host = %s\n"
                "remote-port = %d\n"
                "local-port = %d\n"
                "tcp-nodelay = %d\n",
                config.chan_name,
                config.frame_size,
                config.frame_count,
                config.transport,
                ((DIRECTION_PUSH == config.direction) ? "PUSH" :
                 ((DIRECTION_PULL == config.direction) ? "PULL" : "unknown")),
                config.remote_host, config.remote_port, config.local_port,
                config.tcp_nodelay );
    }

    return 0;
}

/* Defs */

void achd_help(FILE *fptr) {
    fputs( "Usage: achd [OPTIONS...] CHANNEL-NAME\n"
           ,
        fptr);
}

#define REGEX_WORD "([[:alnum:]_\\-]*)"
#define REGEX_SPACE "[ \t\n\r]*"

void achd_parse_config(FILE *fptr, struct achd_config *config) {
    regex_t line_regex;
    regmatch_t match[3];
    if (regcomp(&line_regex,
                "^"REGEX_SPACE"$|"     // empty line
                "^"REGEX_SPACE         // beginning space
                REGEX_WORD             // key
                REGEX_SPACE            // mid space
                "="
                REGEX_SPACE            // mid space
                REGEX_WORD             // value
                REGEX_SPACE            // end space
                "$",
                REG_EXTENDED ) ) {
        fprintf(stderr,"couldn't compile regex\n");
        exit(EXIT_FAILURE);
    }

    int line = 0;
    size_t n = 1024;
    char *lineptr = (char*)malloc(n);
    ssize_t r;
    while( (r = getline(&lineptr, &n, fptr)) > 0 ) {
        line++;
        if( opt_verbosity >= 3 ) printf("line %d: `%s'", line, lineptr);
        // kill comments and translate
        char *p = lineptr;
        while( *p && '#' != *p )  {
            switch(*p) {
            case '_':
                *p = '-';
                break;
            default:
                *p = tolower(*p);
                break;
            }

            p++;
        }
        *p = '\0';
        // match key/value
        int i = regexec(&line_regex, lineptr, sizeof(match)/sizeof(match[0]), match, 0);
        if( i ) {
            fprintf(stderr, "Bad config file, error on line %d\n", line);
            exit(EXIT_FAILURE);
        }
        if( match[1].rm_so >= 0 && match[2].rm_so >=0 ) {
            lineptr[match[1].rm_eo] = '\0';
            lineptr[match[2].rm_eo] = '\0';
            char *key = lineptr+match[1].rm_so;
            char *val = lineptr+match[2].rm_so;
            if( opt_verbosity >= 3 ) printf("`%s' : `%s'\n", key, val );
            achd_set_config(key, val, config);
        }

    }
    regfree(&line_regex);
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
    const char *yes[] = {"yes", "true", "1", NULL};
    const char *no[] = {"no", "false", "0", NULL};
    const char** s;
    for( s = yes; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 1;
    for( s = no; *s; s++ )
        if( 0 == strcasecmp(*s, value) ) return 0;
    fprintf(stderr, "Invalid boolean: %s\n", value);
    exit(EXIT_FAILURE);
}

void achd_set_config (const char *key, const char *val, struct achd_config *config) {
    if       ( 0 == strcmp(key, "channel-name")) {
        config->chan_name = strdup(val);
    } else if( 0 == strcmp(key, "frame-size")) {
        achd_set_int( &config->frame_size, "frame size", val );
    } else if( 0 == strcmp(key, "frame-count")) {
        achd_set_int( &config->frame_count, "frame count", val );
    } else if( 0 == strcmp(key, "remote-port")) {
        achd_set_int( &config->remote_port, "remote port", val );
    } else if( 0 == strcmp(key, "local-port")) {
        achd_set_int( &config->local_port, "local port", val );
    } else if( 0 == strcmp(key, "remote-host")) {
        config->remote_host = strdup(val);
    } else if( 0 == strcmp(key, "transport")) {
        config->transport = strdup(val);
    } else if( 0 == strcmp(key, "tcp-nodelay")) {
        config->tcp_nodelay = achd_parse_boolean( val );
    } else if( 0 == strcmp(key, "retry")) {
        config->retry = achd_parse_boolean( val );
    } else if( 0 == strcmp(key, "direction")) {
        if( 0 == strcmp(val, "push") ) config->direction = DIRECTION_PUSH;
        else if( 0 == strcmp(val, "pull") ) config->direction = DIRECTION_PULL;
        else {
            fprintf(stderr, "Invalid direction: %s\n", val);
            exit(EXIT_FAILURE);
        }
    } else {
        fprintf(stderr, "Invalid configuration key: %s\n", key );
    }
}

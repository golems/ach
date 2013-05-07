/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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

/** \file achproxy.h
 *  \author Neil T. Dantam
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
#include "ach.h"
#include "achutil.h"
#include "achd.h"

size_t opt_msg_cnt = ACH_DEFAULT_FRAME_COUNT;
int opt_truncate = 0;
size_t opt_msg_size = ACH_DEFAULT_FRAME_SIZE;
char *opt_chan_name = NULL;
int opt_verbosity = 0;
int opt_1 = 0;
int opt_mode = -1;
int (*opt_command)(void) = NULL;


static void check_status(ach_status_t r, const char fmt[], ...);
static void parse_cmd( int (*cmd_fun)(void), char *arg );
static void set_cmd( int (*cmd_fun)(void) );

static int parse_mode( const char *arg ) {
    errno = 0;
    long i = strtol( arg, NULL, 8 );
    if( errno ) {
        fprintf( stderr, "Invalid mode %s: (%d) %s\n",
                 arg, errno, strerror(errno) );
        exit(EXIT_FAILURE);
    }
    return (int) i;
}

/* Commands */
int cmd_file(void);
int cmd_dump(void);
int cmd_unlink(void);
int cmd_create(void);
int cmd_chmod(void);

void cleanup() {
    if(opt_chan_name) free(opt_chan_name);
    opt_chan_name = NULL;
}

static void posarg(int i, const char *arg) {
    (void)i;
    switch(i) {
    case 0:
        if( opt_verbosity ) {
            fprintf( stderr, "Setting command: %s\n", arg );
        }
        if( 0 == strcasecmp(arg, "chmod") ) {
            set_cmd( cmd_chmod );
        } else if( 0 == strcasecmp(arg, "create")   ||
                   0 == strcasecmp(arg, "mk")       ||
                   0 == strcasecmp(arg, "make") )
        {
            set_cmd( cmd_create );
        } else if( 0 == strcasecmp(arg, "unlink")   ||
                   0 == strcasecmp(arg, "rm")       ||
                   0 == strcasecmp(arg, "remove") )
        {
            set_cmd( cmd_unlink );
        } else if( 0 == strcasecmp(arg, "dump") ) {
            set_cmd( cmd_dump );
        } else if( 0 == strcasecmp(arg, "file") ) {
            set_cmd( cmd_file );
        } else {
            goto INVALID;
        }
        break;
    case 1:
        if( cmd_chmod == opt_command ) {
            opt_mode = parse_mode( arg );
        } else {
            opt_chan_name = strdup( arg );
        }
        break;
    case 2:
        if( cmd_chmod == opt_command ) {
            opt_chan_name = strdup( arg );
        } else {
            goto INVALID;
        }
        break;
    default:
    INVALID:
        fprintf( stderr, "Invalid argument: %s\n", arg );
        exit(EXIT_FAILURE);
    }
}

static void set_cmd( int (*cmd_fun)(void) ) {
    if( NULL == opt_command ) {
        opt_command = cmd_fun;
    }else {
        fprintf(stderr, "Can only specify one command\n");
        cleanup();
        exit(EXIT_FAILURE);
    }
}

static void parse_cmd( int (*cmd_fun)(void), char *arg ) {
    set_cmd( cmd_fun );
    opt_chan_name = strdup( arg );
}

int main( int argc, char **argv ) {
    /* Parse Options */
    int c, i = 0;
    opterr = 0;
    while( (c = getopt( argc, argv, "C:U:D:F:vn:m:o:1thH?V")) != -1 ) {
        switch(c) {
        case 'C':   /* create   */
            parse_cmd( cmd_create, optarg );
            break;
        case 'U':   /* unlink   */
            parse_cmd( cmd_unlink, optarg );
            break;
        case 'D':   /* dump     */
            parse_cmd( cmd_dump, optarg );
            break;
        case 'F':   /* file     */
            parse_cmd( cmd_file, optarg );
            break;
        case 'n':   /* msg-size */
            opt_msg_size = (size_t)atoi( optarg );
            break;
        case 'm':   /* msg-cnt  */
            opt_msg_cnt = (size_t)atoi( optarg );
            break;
        case 'o':   /* mode     */
            opt_mode = parse_mode( optarg );
            break;
        case 't':   /* truncate */
            opt_truncate++;
            break;
        case 'v':   /* verbose  */
            opt_verbosity++;
            break;
        case '1':   /* once     */
            opt_1++;
            break;
        case 'V':   /* version     */
            ach_print_version("ach");
            exit(EXIT_SUCCESS);
        case '?':   /* help     */
        case 'h':
        case 'H':
            puts( "Usage: ach [OPTION...] [mk|rm|chmod|dump|file] [mode] [channel-name]\n"
                  "General tool to interact with ach channels\n"
                  "\n"
                  "Options:\n"
                  /* "  -C CHANNEL-NAME,          Create a new channel\n" */
                  /* "  -U CHANNEL-NAME,          Unlink (delete) a channel\n" */
                  /* "  -D CHANNEL-NAME,          Dump info about channel\n" */
                  "  -1,                       With 'mk', accept an already created channel\n"
                  /* "  -F CHANNEL-NAME,          Print filename for channel (Linux-only)\n" */
                  "  -m MSG-COUNT,             Number of messages to buffer\n"
                  "  -n MSG-SIZE,              Nominal size of a message\n"
                  "  -o OCTAL,                 Mode for created channel\n"
                  "  -t,                       Truncate and reinit newly create channel.\n"
                  "                            WARNING: this will clobber processes\n"
                  "                            Currently using the channel.\n"
                  "  -v,                       Make output more verbose\n"
                  "  -?,                       Give program help list\n"
                  "  -V,                       Print program version\n"
                  "\n"
                  "Examples:\n"
                  "  ach mk foo                Create channel 'foo' with default buffer sizes.\n"
                  "  ach mk foo -m 10 -n 256   Create channel 'foo' which can buffer up to 10\n"
                  "                            messages of nominal size 256 bytes.\n"
                  "                            Bigger/smaller messages are OK, up to 10*256\n"
                  "                            bytes max (only one message of that size can be\n"
                  "                            buffered at a time).\n"
                  "  ach rm foo                Remove channel 'foo'\n"
                  "  ach mk -1 foo             Create channel 'foo' unless it already exists,\n"
                  "                            in which case leave it alone.\n"
                  "  ach mk foo -o 600         Create channel 'foo' with octal permissions\n"
                  "                            '600'. Note that r/w (6) permission necessary\n"
                  "                            for channel access in order to properly\n"
                  "                            synchronize.\n"
                  "  ach chmod 666 foo         Set permissions of channel 'foo' to '666'\n"
                  "\n"
                  "Report bugs to <ntd@gatech.edu>"
                );
            exit(EXIT_SUCCESS);
        default:
            posarg(i++, optarg);
        }
    }
    while( optind < argc ) {
        posarg(i++, argv[optind++]);
    }

    /* Be Verbose */
    if( opt_verbosity >= 2 ) {
        fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
        fprintf(stderr, "Channel Name: %s\n", opt_chan_name);
        fprintf(stderr, "Message Size: %"PRIuPTR"\n", opt_msg_size);
        fprintf(stderr, "Message Cnt:  %"PRIuPTR"\n", opt_msg_cnt);
    }
    /* Do Something */
    int r;
    errno = 0;
    if( opt_command ) {
        r = opt_command();
    } else if ( opt_chan_name && opt_mode >= 0 ) {
        r = cmd_chmod();
    } else{
        fprintf(stderr, "Must specify a command. Say `ach -H' for help.\n");
        r = EXIT_FAILURE;
    }
    cleanup();
    return r;
}

int cmd_create(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Creating Channel %s\n", opt_chan_name);
    }
    if( opt_msg_cnt < 1 ) {
        fprintf(stderr, "Message count must be greater than zero, not %"PRIuPTR".\n", opt_msg_cnt);
        return -1;
    }
    if( opt_msg_size < 1 ) {
        fprintf(stderr, "Message size must be greater than zero, not %"PRIuPTR".\n", opt_msg_size);
        return -1;
    }
    ach_status_t i;
    {
        ach_create_attr_t attr;
        ach_create_attr_init(&attr);
        if( opt_truncate ) attr.truncate = 1;
        i = ach_create( opt_chan_name, opt_msg_cnt, opt_msg_size, &attr );
    }

    if( ! (opt_1 && i == ACH_EEXIST) ) {
        check_status( i, "Error creating channel '%s'", opt_chan_name );
    } else i = ACH_OK;

    if( opt_mode > 0 ) {
        i = cmd_chmod();
    }

    return i;
}
int cmd_unlink(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Unlinking Channel %s\n", opt_chan_name);
    }

    enum ach_status r = ach_unlink(opt_chan_name);

    check_status( r, "Failed to remove channel '%s'", opt_chan_name );

    return 0;
}

int cmd_dump(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Dumping Channel %s\n", opt_chan_name);
    }
    ach_channel_t chan;
    ach_status_t r = ach_open( &chan, opt_chan_name, NULL );
    check_status( r, "Error opening ach channel '%s'", opt_chan_name );

    ach_dump( chan.shm );

    r = ach_close( &chan );
    check_status( r, "Error closing ach channel '%s'", opt_chan_name );

    return r;
}


int cmd_file(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Printing file for %s\n", opt_chan_name);
    }
    printf("/dev/shm/" ACH_CHAN_NAME_PREFIX "%s\n", opt_chan_name );
    return 0;
}

int cmd_chmod(void) {
    assert(opt_mode >=0 );
    if( opt_verbosity > 0 ) {
        fprintf( stderr, "Changing mode of %s to %o\n",
                 opt_chan_name, (unsigned)opt_mode );
    }

    /* open */
    ach_channel_t chan;
    errno = 0;
    ach_status_t r = ach_open( &chan, opt_chan_name, NULL );
    check_status( r, "Error opening channel '%s'", opt_chan_name );

    /* chmod */
    r = ach_chmod( &chan, (mode_t)opt_mode );
    check_status( r, "Error chmodding channel '%s'", opt_chan_name );

    /* close */
    r = ach_close( &chan );
    check_status( r, "Error closing channel '%s'", opt_chan_name );

    return r;
}

static void check_status(ach_status_t r, const char fmt[], ...) {
    if( ACH_OK != r ) {
        va_list ap;
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end( ap );

        if( errno ) {
            fprintf( stderr, ": %s, errno (%d) %s\n",
                     ach_result_to_string(r), errno, strerror(errno) );
        } else {
            fprintf( stderr, ": %s\n", ach_result_to_string(r) );
        }
        exit( EXIT_FAILURE );
    }
}

/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2011, Georgia Tech Research Corporation
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
#include "ach.h"

size_t opt_msg_cnt = 16;
int opt_truncate = 0;
size_t opt_msg_size = 8192;
char *opt_chan_name = NULL;
int opt_verbosity = 0;
int opt_1 = 0;
int opt_mode = -1;
int (*opt_command)(void) = NULL;

/* Commands */
int cmd_file(void);
int cmd_dump(void);
int cmd_unlink(void);
int cmd_create(void);

void cmd_help(void);

void cleanup() {
    if(opt_chan_name) free(opt_chan_name);
    opt_chan_name = NULL;
}


static void parse_cmd( int (*cmd_fun)(void), char *arg ) {
    if( NULL == opt_command ) {
        opt_command = cmd_fun;
        opt_chan_name = strdup( arg );
    }else {
        fprintf(stderr, "Can only specify one command\n");
        cleanup();
        exit(EXIT_FAILURE);
    }
}

int main( int argc, char **argv ) {
    /* Parse Options */
    int c;
    opterr = 0;
    while( (c = getopt( argc, argv, "C:U:D:F:vn:m:o:1thH?")) != -1 ) {
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
            opt_mode = (int)strtol( optarg, NULL, 8 );
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
        case '?':   /* help     */
        case 'h':
        case 'H':
            puts( "Usage: ach [OPTION...]\n"
                  "General tool to interact with ach channels"
                  "\n"
                  "  -C CHANNEL-NAME,     Create a new channel\n"
                  "  -U CHANNEL-NAME,     Unlink (delete) a channel\n"
                  "  -D CHANNEL-NAME,     Dump info about channel\n"
                  "  -1,                  With -C, accept an already created channel\n"
                  "  -F CHANNEL-NAME,     Print filename for channel\n"
                  "  -m MSG-COUNT,        Number of messages to buffer\n"
                  "  -n MSG-SIZE,         Nominal size of a message\n"
                  "  -o OCTAL,            Mode for created channel\n"
                  "  -t,                  Truncate and reinit newly create channel (use only\n"
                  "                       with -C).  WARNING: this will clobber processes\n"
                  "                       Currently using the channel.\n"
                  "  -v,                  Make output more verbose\n"
                  "  -?,                  Give this help list\n"
                  "  -V,                  Print program version\n" );
            exit(EXIT_SUCCESS);
        default:
            printf("unknown: %c\n", c);
        }
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
    if( opt_command ) {
        r = opt_command();
    }else{
        fprintf(stderr, "Must specify a command. Say `ach -H' for help.\n");
        r = 1;
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
    if( i != ACH_OK && !opt_1 ) {
        fprintf(stderr, "Error creating channel %s: %s\n",
                opt_chan_name, ach_result_to_string(i) );
        return -1;
    } else i = ACH_OK;
    if( opt_mode > 0 ) {
        ach_channel_t chan;
        if( ACH_OK != (i=ach_open(&chan, opt_chan_name, NULL)) ) {
            fprintf(stderr, "Couldn't open channel for chmod: %s\n", ach_result_to_string(i));
            return -1;
        }
        if( ACH_OK != (i=ach_chmod(&chan, (mode_t)opt_mode)) ) {
            fprintf(stderr, "Couldn't chmod: %s", ach_result_to_string(i));
            if( ACH_FAILED_SYSCALL == i ) {
                fprintf(stderr, ", %s\n",strerror(errno));
            } else {
                fprintf(stderr, "\n");
            }
        }
        if( ACH_OK != (i= ach_close(&chan)) ) {
            fprintf(stderr, "Couldn't close channel after chmod: %s\n", ach_result_to_string(i));
            return -1;
        }
    }
    return i;
}
int cmd_unlink(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Unlinking Channel %s\n", opt_chan_name);
    }

    int r = ach_unlink(opt_chan_name);

    if( ACH_OK != r ) {
        fprintf(stderr, "Failed to remove channel `%s': %s\n", opt_chan_name, strerror(errno));
        exit(-1);
    }

    return 0;
}

int cmd_dump(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Dumping Channel %s\n", opt_chan_name);
    }
    ach_channel_t chan;
    ach_status_t r = ach_open( &chan, opt_chan_name, NULL );
    if( ACH_OK == r ) {
        ach_dump( chan.shm );
    } else {
        fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
        return r;
    }
    r = ach_close( &chan );
    if( ACH_OK != r ) {
        fprintf(stderr, "Error closing ach channel: %s\n", ach_result_to_string( r ));
    }
    return r;
}


int cmd_file(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Printing file for %s\n", opt_chan_name);
    }
    printf("/dev/shm/" ACH_CHAN_NAME_PREFIX "%s\n", opt_chan_name );
    return 0;
}

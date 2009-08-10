/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \file achproxy.h
 *  \author Neil T. Dantam
 */

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <argp.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "ach.h"

int opt_msg_cnt = -1;
int opt_msg_size = -1;
char *opt_chan_name = NULL;
int opt_verbosity = 0;

int (*opt_command)(void) = NULL;

void cleanup() {
    if(opt_chan_name) free(opt_chan_name);
    opt_chan_name = NULL;
}

/// argp junk

static struct argp_option options[] = {
    {
        .name = "create",
        .key = 'C',
        .arg = "channel_name",
        .flags = 0,
        .doc = "Create a new channel"
    },
    {
        .name = "unlink",
        .key = 'U',
        .arg = "channel_name",
        .flags = 0,
        .doc = "Unlink a channel"
    },
    {
        .name = "dump",
        .key = 'D',
        .arg = "channel_name",
        .flags = 0,
        .doc = "Dump info about channel"
    },

    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Make output more verbose"
    },

    {
        .name = "msg_size",
        .key = 'n',
        .arg = "bytes",
        .flags = 0,
        .doc = "Nominal size of a message"
    },
    {
        .name = "msg_cnt",
        .key = 'm',
        .arg = "bytes",
        .flags = 0,
        .doc = "Number of messages to buffer"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    },

};

/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "ach-0";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "general tool to interact with ach channels";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


int main( int argc, char **argv ) {
    argp_parse (&argp, argc, argv, 0, NULL, NULL);


    if( opt_verbosity >= 2 ) {
        fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
        fprintf(stderr, "Channel Name: %s\n", opt_chan_name);
        fprintf(stderr, "Message Size: %d\n", opt_msg_size);
        fprintf(stderr, "Message Cnt:  %d\n", opt_msg_cnt);
    }
    int r;
    if( opt_command ) {
        r = opt_command();
    }else{
        fprintf(stderr, "Must specify a command");
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
        fprintf(stderr, "Message count must be greater than zero, not %d.\n", opt_msg_cnt);
        return -1;
    }
    if( opt_msg_size < 1 ) {
        fprintf(stderr, "Message size must be greater than zero, not %d.\n", opt_msg_size);
        return -1;
    }
    int i = ach_create( opt_chan_name, opt_msg_cnt, opt_msg_size, NULL );

    return i;
}
int cmd_unlink(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Unlinking Channel %s\n", opt_chan_name);
    }

    return 0;
}

int cmd_dump(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Dumping Channel %s\n", opt_chan_name);
    }
    ach_channel_t chan;
    int r = ach_open( &chan, opt_chan_name, NULL );
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

static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; // ignore unused parameter
    switch(key) {
    case 'C':
        parse_cmd( cmd_create, arg );
        break;
    case 'U':
        parse_cmd( cmd_unlink, arg );
        break;
    case 'D':
        parse_cmd( cmd_dump, arg );
        break;
    case 'n':
        opt_msg_size = atoi( arg );
        break;
    case 'm':
        opt_msg_cnt = atoi( arg );
        break;
    case 'v':
        opt_verbosity++;
        break;
    case 0:
        break;
    }
    return 0;
}

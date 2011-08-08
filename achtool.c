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
#include <inttypes.h>
#include "ach.h"

size_t opt_msg_cnt = 0;
int opt_truncate = 0;
size_t opt_msg_size = 0;
char *opt_chan_name = NULL;
int opt_verbosity = 0;
int opt_1 = 0;
int opt_mode = -1;

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
        .name = "file",
        .key = 'F',
        .arg = "channel_name",
        .flags = 0,
        .doc = "Print filename for channel"
    },
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Make output more verbose"
    },

    {
        .name = "msg-size",
        .key = 'n',
        .arg = "bytes",
        .flags = 0,
        .doc = "Nominal size of a message"
    },
    {
        .name = "msg-cnt",
        .key = 'm',
        .arg = "count",
        .flags = 0,
        .doc = "Number of messages to buffer"
    },
    {
        .name = "mode",
        .key = 'o',
        .arg = "octal",
        .flags = 0,
        .doc = "mode for created channel"
    },
    {
        .name = "once",
        .key = '1',
        .arg = NULL,
        .flags = 0,
        .doc = "with -C, accept an already create channel"
    },
    {
        .name = "truncate",
        .key = 't',
        .arg = NULL,
        .flags = 0,
        .doc = "truncate and reinit newly create channel (use only with -C).  WARNING: this will clobber processes currently using the channel."
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
const char *argp_program_version = "ach-" ACH_VERSION_STRING;
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
        fprintf(stderr, "Message Size: %"PRIuPTR"\n", opt_msg_size);
        fprintf(stderr, "Message Cnt:  %"PRIuPTR"\n", opt_msg_cnt);
    }
    int r;
    if( opt_command ) {
        r = opt_command();
    }else{
        fprintf(stderr, "Must specify a command\n");
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
    int i;
    {
        ach_create_attr_t attr;
        ach_create_attr_init(&attr);
        if( opt_truncate ) attr.truncate = 1;
        i = ach_create( opt_chan_name, opt_msg_cnt, opt_msg_size, &attr );
    }
    if( i != ACH_OK && !opt_1 ) {
        fprintf(stderr, "Error creating channel %s: %s\n",
                opt_chan_name, ach_result_to_string(i) );
    }
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


int cmd_file(void) {
    if( opt_verbosity > 0 ) {
        fprintf(stderr, "Printing file for %s\n", opt_chan_name);
    }
    printf("/dev/shm/" ACH_CHAN_NAME_PREFIX "%s\n", opt_chan_name );
    return 0;
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
    case 'F':
        parse_cmd( cmd_file, arg );
        break;
    case 'n':
        opt_msg_size = (size_t)atoi( arg );
        break;
    case 'm':
        opt_msg_cnt = (size_t)atoi( arg );
        break;
    case 'o':
        opt_mode = (int)strtol( arg, NULL, 8 );
        break;
    case 't':
        opt_truncate++;
        break;
    case 'v':
        opt_verbosity++;
        break;
    case '1':
        opt_1++;
        break;
    case 0:
        break;
    }
    return 0;
}

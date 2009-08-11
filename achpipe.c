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
#include "ach.h"


// lets pick the number POSIX specifies for atomic reads/writes
#define INIT_BUF_SIZE 512

//#define DEBUGF(fmt, a... )
#define DEBUGF(fmt, a... )                      \
    fprintf(stderr, (fmt), ## a )

char *opt_chan_name = NULL;
int opt_pub = 0;
int opt_sub = 0;

/// argp junk

static struct argp_option options[] = {
    {
        .name = "publish",
        .key = 'p',
        .arg = NULL,
        .flags = 0,
        .doc = "Read input and publish to a channel"
    },
    {
        .name = "subscribe",
        .key = 's',
        .arg = NULL,
        .flags = 0,
        .doc = "Subscribe to a channel and write to output"
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
const char *argp_program_version = "achpipe-" ACH_VERSION_STRING;
/// argp program arguments documention
static char args_doc[] = "[-p|-s] channel";
/// argp program doc line
static char doc[] = "copy ach frames to/from stdio";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


void publish( int fd, char *chan_name )  {
    DEBUGF("publish()\n");
    ach_channel_t chan;
    int r;

    { // open channel
        r = ach_open( &chan, chan_name, NULL );
        if( ACH_OK != r ) {
            fprintf(stderr, "Failed to open channel %s for publish: %s\n",
                    chan_name, ach_result_to_string(r) );
            return;
        }
    }

    { // publish loop
        int max = INIT_BUF_SIZE;
        int cnt;
        char *buf = malloc( max );
        assert(buf);

        while(1) {
            // get size
            r = ach_stream_read_msg_size( fd, &cnt );
            if( r <= 0 ) break;
            // make sure buf can hold it
            if( cnt > max ) {
                max = cnt;
                free( buf );
                buf = malloc( max );
                assert(buf);
            }
            // get data
            r = ach_stream_read_msg_data( fd, buf, cnt, max );
            if( r <= 0 ) break;
            assert( cnt == r );
            // put data
            r = ach_put( &chan, buf, cnt );
        }
        free(buf);

    }
    ach_close( &chan );

}

void subscribe(int fd, char *chan_name) {
    DEBUGF("subscribe()\n");
    // get channel
    ach_channel_t chan;
    {
        int r = ach_open( &chan, chan_name, NULL );
        if( ACH_OK != r ) {
            fprintf(stderr, "Failed to open channel %s for publish: %s\n",
                    chan_name, ach_result_to_string(r) );
            return;
        }
    }
    int max = INIT_BUF_SIZE;
    char *buf = malloc(max);
    int t0 = 1;

    // read loop
    while(1) {
        size_t frame_size;
        int r = ach_wait_next(&chan, buf, max, &frame_size,  NULL );
        if( ACH_OK != r )  {
            if( ! (t0 && r == ACH_MISSED_FRAME) ) {
                if( ACH_CLOSED != r ) {
                    fprintf(stderr, "sub: ach_error: %s\n",
                            ach_result_to_string(r));
                }
            }
            if( ACH_OVERFLOW == r ) {
                int fs = frame_size;
                assert(fs > max );
                free(buf);
                max = frame_size;
                buf = malloc( max );
                continue;
            }
            if( r != ACH_MISSED_FRAME ) break;
        }
        r = ach_stream_write_msg( fd, buf, frame_size );
        t0 = 0;
    }
    free(buf);
    ach_close( &chan );
}


int main( int argc, char **argv ) {
    argp_parse (&argp, argc, argv, 0, NULL, NULL);

    // validate arguments
    if( ! opt_chan_name ) {
        fprintf(stderr, "Error: must specify channel\n");
        return 1;
    }else if( ! opt_pub && ! opt_sub ) {
        fprintf(stderr, "Error: must specify publish or subscribe mode\n");
        return 1;
    }else if(  opt_pub && opt_sub ) {
        fprintf(stderr, "Error: must specify publish xor subscribe mode\n");
        return 1;
    } else if (opt_pub) {
        publish( STDIN_FILENO, opt_chan_name );
    } else if (opt_sub) {
        subscribe( STDOUT_FILENO, opt_chan_name );
    } else {
        assert(0);
    }
    return 0;
}


static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; // ignore unused parameter
    switch(key) {
    case 'p':
        opt_pub = 1;
        break;
    case 's':
        opt_sub = 1;
        break;
    case 0:
        opt_chan_name = arg;
        break;
    }
    return 0;
}

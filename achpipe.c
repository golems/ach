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

/** \file achpipe.c
 *  \author Neil T. Dantam
 *
 * \todo Extend protocol so that it only sends frames when requested
 */


/** \page "Pipe Protocol"
 *
 * Simple Mode: Ach frames are sent across the pipe as the four ascii
 * bytes "size", and big-endian 32-bit integer indicating the size in
 * bytes of the frame, the four ascii bytes "data", and the proper
 * number of data bytes.
 *
 *
 * Note that ach normally communicates using shared memory.  The pipe
 * protocol is only for intermachine communication or increasing
 * robustness if one does not want a process to touch shared memory
 * directly.
 *
 * <tt>
 * -----------------------------------\n
 * | "size" | int32 | "data" | $DATA |\n
 * -----------------------------------\n
 * </tt>
 *
 * \sa Todo List
 */
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <argp.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <stdarg.h>
#include "ach.h"


// lets pick the number POSIX specifies for atomic reads/writes
/// Initial size of ach frame buffer
#define INIT_BUF_SIZE 512

//#define DEBUGF(fmt, a... )
//#define DEBUGF(fmt, a... )
//fprintf(stderr, (fmt), ## a )

/// CLI option: channel name
char *opt_chan_name = NULL;
/// CLI option: publish mode
int opt_pub = 0;
/// CLI option: subscribe mode
int opt_sub = 0;
/// CLI option: verbosity level
int opt_verbosity = 0;
/// CLI option: send only most recent frames
int opt_last = 0;

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
        .name = "last",
        .key = 'l',
        .arg = NULL,
        .flags = 0,
        .doc = "gets the most recent message in subscribe mode (default is next)"
    },
    {
        .name = "verbosity",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "say more stuff"
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


/// print stuff based on verbosity level
void verbprintf( int level, const char fmt[], ... ) {
    va_list argp;
    va_start( argp, fmt );
    if( level <= opt_verbosity ) {
        fprintf(stderr, "srd: ");
        vfprintf( stderr, fmt, argp );
    }
    va_end( argp );
}


static void *xmalloc( size_t size ) {
    void *p = malloc( size );
    if( NULL == p ) {
        perror("malloc");
        abort();
    }
}

/// publishing loop
void publish( int fd, char *chan_name )  {
    verbprintf(1, "Publishing()\n");
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
        char *buf = xmalloc( max );

        while(1) {
            // get size
            r = ach_stream_read_msg_size( fd, &cnt );
            if( r <= 0 ) break;
            // make sure buf can hold it
            if( cnt > max ) {
                max = cnt;
                free( buf );
                buf = xmalloc( max );
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

/// subscribing loop
void subscribe(int fd, char *chan_name) {
    verbprintf(1, "Subscribing()\n");
    // get channel
    ach_channel_t chan;
    {
        int r = ach_open( &chan, chan_name, NULL );
        if( ACH_OK != r ) {
            fprintf(stderr, "Failed to open channel %s for subscribe: %s\n",
                    chan_name, ach_result_to_string(r) );
            return;
        }
    }
    int max = INIT_BUF_SIZE;
    char *buf = xmalloc(max);
    int t0 = 1;

    // read loop
    while(1) {
        size_t frame_size = -1;
        int r = opt_last ?
            ach_wait_last(&chan, buf, max, &frame_size,  NULL ) :
            ach_wait_next(&chan, buf, max, &frame_size,  NULL ) ;

        // check return code
        if( ACH_OK != r )  {
            if( ACH_OVERFLOW == r ) {
                int fs = frame_size;
                assert(fs > max );
                free(buf);
                max = frame_size;
                buf = xmalloc( max );
                continue;
            }
            if( ! (t0 && r == ACH_MISSED_FRAME) ) {
                if( ACH_CLOSED != r ) {
                    fprintf(stderr, "sub: ach_error: %s\n",
                            ach_result_to_string(r));
                }
            }
            if( r != ACH_MISSED_FRAME ) break;
        }
        r = ach_stream_write_msg( fd, buf, frame_size );
        t0 = 0;
    }
    free(buf);
    ach_close( &chan );
}

/// main
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
    case 'v':
        opt_verbosity ++;
        break;
    case 'l':
        opt_last = 1;
        break;
    case 0:
        opt_chan_name = arg;
        break;
    }
    return 0;
}

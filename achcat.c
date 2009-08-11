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

/** \file achcat.c
 *  \author Neil T. Dantam
 *
 *  This program copies lines of text from std{in,out} to or from an
 *  ach channel.
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <argp.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include "ach.h"


/// argp junk

static struct argp_option options[] = {
    {
        .name = "publish",
        .key = 'p',
        .arg = NULL,
        .flags = 0,
        .doc = "Publish to a channel"
    },
    {
        .name = "subscribe",
        .key = 's',
        .arg = NULL,
        .flags = 0,
        .doc = "Subscribe to a channel"
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
const char *argp_program_version = "achcat-0";
/// argp program arguments documention
static char args_doc[] = "[-p|-s] channel";
/// argp program doc line
static char doc[] = "Copies ach frames as string lines to/from stdio.  Mostly intended as an ach testing tool.";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


void *publish_loop(void *);
int publish(ach_channel_t *);
int subscribe(ach_channel_t *);
int pub_chan(ach_channel_t *, ach_attr_t *);
int sub_chan(ach_channel_t *, ach_attr_t *);

char pbuffer[4096];
char sbuffer[4096];


// options
int opt_msg_size = 256;
int opt_msg_cnt = 10;
char *opt_chan_name = NULL;
int opt_pub = 0;
int opt_sub = 0;


FILE *fin;
FILE *fout;

void *publish_loop(void* pub) {
    publish(pub);
    return NULL;
}

int pub_chan( ach_channel_t *chan, ach_attr_t *attr ) {
    int r = ach_open( chan, opt_chan_name, attr );
    if( ACH_OK != r ) return 1;
    return 0;
}

int publish( ach_channel_t *chan) {
    int r;
    while(1) {
        char *fr;
        // get size
        fr = fgets( pbuffer, sizeof(pbuffer), fin );
        if( !fr ) break;
        assert( pbuffer == fr );
        // put data
        //printf("read: %s", pbuffer );
        r = ach_put( chan, pbuffer, strlen(pbuffer) );
        if( r != ACH_OK ) break;
        //printf("put: %s", pbuffer );

        //ach_dump( chan.shm );
    }
    ach_close( chan );
    //fprintf(stderr,"end of publish\n");
    return r;
}

int sub_chan( ach_channel_t *chan, ach_attr_t *attr ) {
    int r = ach_open( chan, opt_chan_name, attr );
    if( r != ACH_OK ) return 1;
    return 0;
}

int subscribe( ach_channel_t *chan) {
    int r;
    int t0 = 1;
    while(1) {
        size_t frame_size = 0;
        size_t fr;
        r = ach_wait_next(chan, sbuffer, sizeof(sbuffer), &frame_size,  NULL );
        if( ACH_OK != r )  {
            if( ! (t0 && r == ACH_MISSED_FRAME) )
                if( ACH_CLOSED != r ) {
                    fprintf(stderr, "sub: ach_error: %s\n",
                            ach_result_to_string(r));
                }
            if( r != ACH_MISSED_FRAME ) break;

        }
        //fprintf(stderr, "sub: got %d bytes\n", frame_size);
        //fprintf(stderr, "sub: %s\n", sbuffer);

        fr = fwrite( sbuffer, sizeof(char), frame_size, fout );
        if ( fr != frame_size )  {
            r = 0;
            break;
        }
        fflush(fout);
    }
    //fprintf(stderr,"end of subscribe\n");
    ach_close( chan );
    return r;
}


int main( int argc, char **argv ) {
    fin = stdin;
    fout = stdout;
    argp_parse (&argp, argc, argv, 0, NULL, NULL);
    //printf("chan: %s\n", opt_chan_name );
    //printf("pub:  %d\n", opt_pub );
    //printf("sub:  %d\n", opt_sub );

    // validate arguments
    if( ! opt_chan_name ) {
        fprintf(stderr, "Error: must specify channel\n");
        return 1;
    }
    if( ! opt_pub && ! opt_sub ) {
        fprintf(stderr, "Error: must specify publish or subscribe mode\n");
        return 1;
    }

    ach_channel_t pub, sub;
    {
        int r;
        ach_attr_t attr;
        ach_attr_init( &attr );
        //attr.map_anon = opt_pub && opt_sub;
        if( opt_pub ) {
            r = pub_chan( &pub, &attr );
            assert( 0 == r );
            //attr.shm = pub.shm;
        }
        if( opt_sub ) {
            r = sub_chan( &sub, &attr );
            assert( 0 == r );
        }
    }

    // check for io case
    if(  opt_pub &&  opt_sub ) {
        pthread_t pub_thread;
        pthread_create( &pub_thread, NULL, publish_loop, &pub );

        subscribe(&sub);
        void *v;
        pthread_join(pub_thread, &v);
        return 0;
    }
    // normal cases
    if( opt_sub ) return subscribe(&sub);
    if( opt_pub ) return publish(&pub);
    assert( 0 );
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

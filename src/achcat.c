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

/** \file achcat.c
 *  \author Neil T. Dantam
 *
 *  This program copies lines of text from std{in,out} to or from an
 *  ach channel.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <inttypes.h>
#include "ach.h"


void *publish_loop(void *);
int publish(ach_channel_t *);
int subscribe(ach_channel_t *);
int pub_chan(ach_channel_t *, ach_attr_t *);
int sub_chan(ach_channel_t *, ach_attr_t *);

char pbuffer[4096];
char sbuffer[4096];


/* options */
int opt_msg_size = 256;
int opt_msg_cnt = 10;
char *opt_chan_name = NULL;
int opt_pub = 0;
int opt_sub = 0;


FILE *fin;
FILE *fout;

void *publish_loop(void* pub) {
    publish((ach_channel_t*)pub);
    exit(0);
    return NULL;
}


int publish( ach_channel_t *chan) {
    int r=0;
    while(1) {
        char *fr;
        /* get size */
        fr = fgets( pbuffer, (int)sizeof(pbuffer), fin );
        if( !fr ) break;
        assert( pbuffer == fr );
        /* put data */
        /*printf("read: %s", pbuffer );*/
        r = ach_put( chan, pbuffer, strlen(pbuffer) );
        if( r != ACH_OK ) break;
        /*printf("put: %s", pbuffer );*/

        /*ach_dump( chan.shm );*/
    }
    ach_close( chan );
    /*fprintf(stderr,"end of publish\n");*/
    return r;
}


int subscribe( ach_channel_t *chan) {
    ach_status_t r;
    int t0 = 1;
    while(1) {
        size_t frame_size = 0;
        size_t fr;
        r = ach_get( chan, sbuffer, sizeof(sbuffer), &frame_size, NULL, 0 );
        if( ACH_OK != r )  {
            if( ACH_STALE_FRAMES == r ) {
                usleep(1000);
            } else {
                if( ! (t0 && r == ACH_MISSED_FRAME) )
                    if( ACH_CLOSED != r ) {
                        fprintf(stderr, "sub: ach_error: %s\n",
                                ach_result_to_string(r));
                    }
                if( !  ( ACH_MISSED_FRAME == r ||
                         ACH_STALE_FRAMES == r ) ) break;
            }

        }
        /*fprintf(stderr, "sub: got %d bytes\n", frame_size);*/
        /*fprintf(stderr, "sub: %s\n", sbuffer);*/

        fr = fwrite( sbuffer, sizeof(char), frame_size, fout );
        if ( fr != frame_size )  {
            r = ACH_OK;
            break;
        }
        fflush(fout);
    }
    /*fprintf(stderr,"end of subscribe\n");*/
    ach_close( chan );
    return r;
}


int main( int argc, char **argv ) {
    fin = stdin;
    fout = stdout;


    int c;
    while( (c = getopt( argc, argv, "p:s:hH?")) != -1 ) {
        switch(c) {
        case 'p':
            opt_pub = 1;
            opt_chan_name = optarg;
            break;
        case 's':
            opt_sub = 1;
            opt_chan_name = optarg;
            break;
        case 'h':
        case 'H':
        case '?':
            puts( "Usage: achcat [OPTION...] [-p|-s] channel\n"
                  "Copies ach frames as string lines to/from stdio.  Mostly intended as an ach\n"
                  "testing tool.\n"
                  "\n"
                  "  -p CHANNEL-NAME,   Publish to a channel\n"
                  "  -s CHANNEL-NAME,   Subscribe to a channel\n"
                  "  -?,                Give this help list\n" );
            exit(EXIT_SUCCESS);
        }
    }

    /*printf("chan: %s\n", opt_chan_name );*/
    /*printf("pub:  %d\n", opt_pub );*/
    /*printf("sub:  %d\n", opt_sub );*/

    /* validate arguments */
    if( ! opt_chan_name ) {
        fprintf(stderr, "Error: must specify channel\n");
        return 1;
    }
    if( ! opt_pub && ! opt_sub ) {
        fprintf(stderr, "Error: must specify publish or subscribe mode\n");
        return 1;
    }

    ach_channel_t pub, sub;
    memset( &pub, 0, sizeof(pub));
    memset( &sub, 0, sizeof(sub));
    {
        int r;
        ach_attr_t attr;
        ach_attr_init( &attr );
        /*attr.map_anon = opt_pub && opt_sub;*/
        if( opt_pub && ! opt_sub ) {
            r = ach_open( &pub, opt_chan_name, &attr );
            assert( 0 == r );
        } else if( opt_sub && !opt_pub ) {
            r = ach_open( &sub, opt_chan_name, &attr );
            assert( 0 == r );
        }else if (opt_pub && opt_sub ) {
            ach_create_attr_t cattr;
            ach_create_attr_init( &cattr );
            cattr.map_anon = 1;
            ach_create( opt_chan_name, 10ul, 512ul, &cattr );
            assert( cattr.shm );
            attr.map_anon = 1;
            attr.shm = cattr.shm;
            r = ach_open( &sub, opt_chan_name, &attr );
            assert( 0 == r );
            r = ach_open( &pub, opt_chan_name, &attr );
            assert( 0 == r );
        } else {
            assert(0);
        }
    }

    /* check for io case */
    if(  opt_pub &&  opt_sub ) {
        pthread_t pub_thread;
        pthread_create( &pub_thread, NULL, publish_loop, &pub );

        subscribe(&sub);
        void *v;
        pthread_join(pub_thread, &v);
        return 0;
    }
    /* normal cases */
    if( opt_sub ) return subscribe(&sub);
    if( opt_pub ) return publish(&pub);
    assert( 0 );
    return 0;
}

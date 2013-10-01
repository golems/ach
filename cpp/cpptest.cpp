/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
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
#include "Ach.hpp"


int publish(ach::Channel *);
int subscribe(ach::Channel  *);

char pbuffer[4096];


/* options */
int opt_msg_size = 256;
int opt_msg_cnt = 10;
char *opt_chan_name = NULL;
int opt_pub = 0;
int opt_sub = 0;


FILE *fin;
FILE *fout;

int publish( ach::Channel *chan) {
    int r=0;
    while(1) {
        char *fr;
        /* get size */
        fr = fgets( pbuffer, (int)sizeof(pbuffer), fin );
        if( !fr ) break;
        assert( pbuffer == fr );
        /* put data */
        r = chan->put( pbuffer, strlen(pbuffer) );
        if( r != ACH_OK ) break;
    }
    chan->close( );
    return r;
}


int subscribe( ach::Channel *chan) {
    ach_status_t r;
    int t0 = 1;
    std::vector<uint8_t> buf;
    while(1) {
        size_t frame_size = 0;
        size_t fr;
        r  = chan->get ( &buf, 0, &frame_size, NULL, 0,
                         ACH_MASK_OK | ACH_MASK_STALE_FRAMES, ACH_MASK_MISSED_FRAME );
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

        fr = fwrite( &buf[0], sizeof(char), frame_size, fout );
        if ( fr != frame_size )  {
            r = ACH_OK;
            break;
        }
        fflush(fout);
    }
    /*fprintf(stderr,"end of subscribe\n");*/
    chan->close( );
    return r;
}


int main( int argc, char **argv ) {
    fin = stdin;
    fout = stdout;


    int c;
    while( (c = getopt( argc, argv, "p:s:hH?V")) != -1 ) {
        switch(c) {
        case 'p':
            opt_pub = 1;
            opt_chan_name = optarg;
            break;
        case 's':
            opt_sub = 1;
            opt_chan_name = optarg;
            break;
        case 'V':   /* version     */
            puts("cpptest");
            exit(EXIT_SUCCESS);
        case 'h':
        case 'H':
        case '?':
            puts( "Usage: cpptest [OPTION...] [-p|-s] channel\n"
                  "Copies ach frames as string lines to/from stdio.  Mostly intended as an ach\n"
                  "testing tool.\n"
                  "\n"
                  "  -p CHANNEL-NAME,   Publish to a channel\n"
                  "  -s CHANNEL-NAME,   Subscribe to a channel\n"
                  "  -?,                Give this help list\n" );
            exit(EXIT_SUCCESS);
        }
    }
    /* validate arguments */
    if( ! opt_chan_name ) {
        fprintf(stderr, "Error: must specify channel\n");
        exit(EXIT_FAILURE);
    }
    if( ! opt_pub && ! opt_sub ) {
        fprintf(stderr, "Error: must specify publish or subscribe mode\n");
        exit(EXIT_FAILURE);
    }
    if(  opt_pub &&  opt_sub ) {
        fprintf(stderr, "Error: cannot publish and subscribe\n");
        exit(EXIT_FAILURE);
    }

    ach::CerrChannel channel;
    channel.open( opt_chan_name );

    /* normal cases */
    if( opt_sub ) return subscribe(&channel);
    else if( opt_pub ) return publish(&channel);
    assert( 0 );
    return 0;
}


/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* indent-tabs-mode:  nil                    */
/* c-basic-offset: 4                         */
/* c-file-offsets: ((innamespace . 0))       */
/* End:                                      */

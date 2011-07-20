/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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

#include <amino.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include "ach.h"

#define CHAN  "ach-test-clobber"

#define N_SUB 16 // number of subscribers
#define N_PUB 16 // number of publishers

#define MAX 1024

static int publisher( int32_t i ) {
    ach_channel_t chan;
    int r = ach_open( &chan, CHAN, NULL );
    if( r != ACH_OK ) {
        fprintf(stderr, "publisher %d couldn't ach_open: %s\n",
                i, ach_result_to_string(r) );
        return -1;
    }

    int32_t data[2] = {i, 0};
    for( size_t j = 0; j < MAX; j++,data[1]++ ) {
        r = ach_put( &chan, data, sizeof(data) );
        if( r != ACH_OK ) {
            fprintf(stderr, "publisher %d couldn't ach_put: %s\n",
                    i, ach_result_to_string(r) );
            return -1;
        }
        usleep(500);
    }
    r = ach_close(&chan);
    if( ACH_OK != r ) {
        fprintf(stderr, "publisher %d couldn't ach_close: %s\n",
                i, ach_result_to_string(r) );
        return -1;
    } else {
        fprintf(stderr, "publisher %d ok\n", i);
        return 0;
    }
}

static int subscriber( int i ) {
    ach_channel_t chan;
    int32_t ctr[N_PUB] = {0};
    memset(ctr,0,sizeof(ctr));
    int r = ach_open( &chan, CHAN, NULL );
    if( r != ACH_OK ) {
        fprintf(stderr, "subscriber %d couldn't ach_open: %s",
                i, ach_result_to_string(r) );
        return -1;
    }

    int32_t data[2];
    for( int j = 0; j < N_PUB*MAX; j++ ) {
        struct timespec abstime = aa_tm_future( aa_tm_sec2timespec(1) );
        size_t frame_size;
        r = ach_wait_next( &chan, data, sizeof(data), &frame_size,
                           &abstime );
        if( ACH_OK != r && ACH_MISSED_FRAME != r) {
            fprintf(stderr, "subscriber %d couldn't ach_get: %s\n",
                    i, ach_result_to_string(r) );
            return -1;
        }
        if( sizeof(data) != frame_size ) {
            fprintf(stderr, "subscriber %d bad frame size: %d\n",
                    i, frame_size);
            return -1;
        }
        if( 0 > data[0] || N_PUB <= data[0] ) {
            fprintf(stderr, "subscriber %d bad pub id: %d\n",
                    i, data[0]);
            return -1;
        }
        if( ctr[ data[0] ] != data[1] ) {
            fprintf(stderr, "subscriber %d missed %d frames from %d, (it's probaly ok)\n", i, data[1] - ctr[data[0]], data[0]);
        }
        if( ctr[ data[0] ] > data[1] ) {
            fprintf(stderr, "subscriber %d, count %d, got [%d, %d]\n",
                    i, ctr[data[0]], data[1], data[0] );
            return -1;
        } else {
            j += data[1] - ctr[ data[0] ] ;
            ctr[ data[0] ] = data[1]+1;
        }
    }
    r = ach_close(&chan);
    if( ACH_OK != r ) {
        fprintf(stderr, "subscriber %d couldn't ach_close: %s",
                i, ach_result_to_string(r) );
        return -1;
    }
    else {
        fprintf(stderr, "subscriber %d ok\n", i);
        return 0;
    }
}

int main( int argc, char **argv ){
    (void)argc; (void)argv;

    int r = system("rm -f /dev/shm/achshm-" CHAN);
    r = system("./ach -n 32 -m 256 -C " CHAN);

    pid_t sub_pid[N_SUB];
    pid_t pub_pid[N_PUB];

    // create subscribers
    for( int i = 0; i < N_SUB; i++ ) {
        pid_t p = fork();
        if( p < 0 ) exit(-1);
        else if( 0 == p ) return subscriber(i);
        else sub_pid[i] = p;

    }

    // create publishers
    for( int i = 0; i < N_PUB; i++ ) {
        pid_t p = fork();
        if( p < 0 ) exit(-1);
        else if( 0 == p ) return publisher(i);
        else pub_pid[i] = p;
    }

    // wait
    for( int i = 0; i < N_SUB+N_PUB; i++ ) {
        int s;
        pid_t pid = wait(&s);
        (void)pid;
        if( 0 != s ) return -1;
    }
    return 0;
}

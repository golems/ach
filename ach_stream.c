/*
 * Copyright (c) 2009, Neil T. Dantam
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *     * Neither the name of the copyright holder(s) nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER(S) ''AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER(S) BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */


/** \file ach_stream.c
 *  \author Neil T. Dantam
 */



#include <stdint.h>
#include <endconv.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

char ach_stream_presize[5] = "size";
char ach_stream_postsize[5] = "data";

static int read_fill(int fd, char *buf, int cnt ) {
    int r;
    int n = 0;
    do {
        r = read( fd, buf+n, cnt-n );
        if( r > 0 ) n += r;
        else return r;
    }while( n < cnt);
    return cnt;
}

static int write_fill(int fd, char *buf, int cnt ) {
    int r;
    int n = 0;
    do {
        r = write( fd, buf+n, cnt-n );
        if( r > 0 ) n += r;
        else return r;
    }while( n < cnt);
    return cnt;
}

int ach_stream_write_msg( int fd, char *buf, int cnt) {
    char sizebuf[4+4+4];
    int r;

    // make size field
    memcpy( & sizebuf[0], ach_stream_presize, 4);
    endconv_st_be_i32( & sizebuf[4], cnt );
    memcpy( & sizebuf[8], ach_stream_postsize, 4);

    // send size
    r = write_fill( fd, sizebuf, sizeof(sizebuf) );
    if( r <= 0 ) return r;
    assert( sizeof(sizebuf) == r );

    // send data
    r = write_fill( fd, buf, cnt );
    assert( cnt == r || r <= 0 );

    return r + sizeof(sizebuf);
}


int ach_stream_read_msg_size( int fd, int *cnt) {
    char buf[4+4+4];
    int r;

    r = read_fill( fd, buf, sizeof(buf) );
    if( r <= 0 ) return r;
    assert( sizeof(buf) == r );

    if( 0 == memcpy( ach_stream_presize,  & buf[0], 4 ) &&
        0 == memcpy( ach_stream_postsize, & buf[8], 4 ) ) {
        *cnt = -1;
    } else {
        *cnt = endconv_ld_be_i32( & buf[4] );
    }
    return r;
}

int ach_stream_read_msg_data( int fd, char *buf, int msg_size, int buf_size) {
    int r = 0;

    assert( buf_size >= msg_size );
    if( buf_size < msg_size ) return -1;

    r = read_fill( fd, buf, msg_size );
    if( r <= 0 ) return r;
    assert( msg_size == r );

    return r;
}

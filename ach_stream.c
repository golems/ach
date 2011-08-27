/* -*- mode: C; c-basic-offset: 4  -*- */
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
#include <amino.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <sys/types.h>

#include <pthread.h>
#include "ach.h"

char ach_stream_presize[5] = "size";
char ach_stream_postsize[5] = "data";

ssize_t ach_stream_read_fill(int fd, char *buf, size_t cnt ) {
    ssize_t r;
    size_t n = 0;
    do {
        r = read( fd, buf+n, cnt-n );
        if( r > 0 ) n += (size_t)r;
        else return r;
    }while( n < cnt);
    return (int)cnt;
}

static ssize_t read_fill(int fd, char *buf, size_t cnt ) {
    return ach_stream_read_fill( fd, buf, cnt );
}

ssize_t ach_stream_write_fill(int fd, const char *buf, size_t cnt ) {
    ssize_t r;
    size_t n = 0;
    do {
        r = write( fd, buf+n, cnt-n );
        if( r > 0 ) n += (size_t)r;
        else return r;
    }while( n < cnt);
    return (int)cnt;
}

static ssize_t write_fill(int fd, const char *buf, size_t cnt ) {
    return ach_stream_write_fill( fd, buf, cnt );
}

ssize_t ach_stream_write_msg( int fd, const char *buf, size_t cnt) {
    char sizebuf[4+4+4];
    ssize_t r;

    // make size field
    memcpy( & sizebuf[0], ach_stream_presize, 4);
    aa_endconv_st_be_u32( & sizebuf[4], (uint32_t)cnt );
    memcpy( & sizebuf[8], ach_stream_postsize, 4);

    // send size
    r = write_fill( fd, sizebuf, sizeof(sizebuf) );
    if( r <= 0 ) return r;
    assert( sizeof(sizebuf) == r );

    // send data
    r = write_fill( fd, buf, cnt );
    assert( (int)cnt == r || r <= 0 );

    return r + (int)sizeof(sizebuf);
}


ssize_t ach_stream_read_msg_size( int fd, int *cnt) {
    char buf[4+4+4];
    ssize_t r;

    *cnt = -1;

    r = read_fill( fd, buf, sizeof(buf) );

    if( r <= 0 ) return r;

    assert( sizeof(buf) == r );

    if( 0 == memcmp( ach_stream_presize,  & buf[0], 4 ) &&
        0 == memcmp( ach_stream_postsize, & buf[8], 4 ) ) {
        *cnt = aa_endconv_ld_be_i32( & buf[4] );
    } else {
        *cnt = -1;
    }
    return r;
}

ssize_t ach_stream_read_msg_data( int fd, char *buf, size_t msg_size, size_t buf_size) {
    ssize_t r = 0;

    assert( buf_size >= msg_size );
    if( buf_size < msg_size ) return -1;

    r = read_fill( fd, buf, msg_size );
    if( r <= 0 ) return r;
    assert( msg_size == (size_t) r );

    return r;
}

ssize_t ach_read_line( int fd, char *buf, size_t n ) {
    // bit-bang cause it doesn't really matter
    ssize_t r = 0;
    size_t i = 0;
    while(i < n - 1) {
        int tries = 0;
        // get a byte
        do {
            r = read( fd, buf + i, 1 );
        } while( r != 1 &&
                 EINTR == errno &&
                 tries++ < ACH_INTR_RETRY );
        // check for end
        if( r != 1 || '\n' == buf[i] ){
            buf[++i] = '\0';
            return (int)i;
        }
        // increment
        i++;
    }
    assert( buf[i - 1] != '\n' );
    buf[i] = '\0';
    return (int)i;
}


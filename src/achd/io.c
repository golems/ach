/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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



#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <ctype.h>
#include <signal.h>
#include <regex.h>
#include <assert.h>
#include <stdarg.h>
#include <errno.h>
#include <syslog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "ach.h"
#include "ach/private_posix.h"
#include "achutil.h"
#include "achd.h"


ssize_t achd_read(int fd, void *buf, size_t cnt ) {
    size_t n = 0;
    while( !cx.sig_received && n < cnt ) {
        errno = 0;
        ssize_t r = read( fd, (uint8_t*)buf+n, cnt-n );
        if( r > 0 ) n += (size_t)r;
        else if (r < 0 && EINTR == errno && !cx.sig_received) continue;
        else break;
    };
    return (ssize_t)n;
}

ssize_t achd_write(int fd, const void *buf, size_t cnt ) {
    size_t n = 0;
    while( !cx.sig_received && n < cnt ) {
        ssize_t r = write( fd, (uint8_t*)buf+n, cnt-n );
        if( r > 0 ) n += (size_t)r;
        else if (EINTR == errno && !cx.sig_received) continue;
        else return r;
    } while( n < cnt);
    return (ssize_t)cnt;
}

int achd_getc(int fd) {
    char c;
    ssize_t r = achd_read(fd, &c, 1);
    if( 1 == r ) return c;
    else return EOF;
}

enum ach_status achd_readline(int fd, char *buf, size_t cnt ) {
    int c;
    size_t i = 0;
    /* Yeah, this loop is super-inefficnet, but it's only for
     * connection setup */
    while( i < cnt ) {
        c = achd_getc(fd);
        switch(c) {
        case '\n':
            buf[i++] = '\0';
            return ACH_OK;
        case '\r':
            /* TODO: is eating '\r' reasonable? */
            break;
        case EOF:
            return ACH_FAILED_SYSCALL;
        default:
            buf[i++] = (char)c;
        }
    }
    return ACH_OVERFLOW;
}

enum ach_status achd_printf(int fd, const char fmt[], ...) {
    int n = ACHD_LINE_LENGTH-1;
    do {
        char buf[n+1];
        va_list ap;
        va_start(ap, fmt);
        n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end( ap );
        if( n < 0 ) {
            ACH_LOG( LOG_CRIT, "Error in printf\n");
            return ACH_BUG;
        } else if( (size_t)n < sizeof(buf) ) {
            if ( (ssize_t)n == achd_write( fd, buf, (size_t)n ) ) {
                return ACH_OK;
            } else {
                return ACH_FAILED_SYSCALL;
            }
        } /* else continue */
    } while(!cx.sig_received);
    return ACH_FAILED_SYSCALL;
}

/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation
 * Copyright (C) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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


/** \file ach_posix.c
 *  \author Neil T. Dantam
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/stat.h>

#include <string.h>
#include <inttypes.h>

#include "ach.h"
#include "ach/private_posix.h"
#include "ach/klinux_generic.h"

#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef NDEBUG

#define IFDEBUG(test, x )
#define DEBUGF( ... )
#define DEBUG_PERROR(a)

#else /* enable debugging */

#define IFDEBUG( test, x ) if(test) { (x); }
#define ACH_ERRF( ... ) fprintf(stderr, __VA_ARGS__ )
#define DEBUG_PERROR(a) perror(a)

#endif /* NDEBUG */


#include "ach/impl_generic.h"

#define SYSCALL_RETRY( expr, test )                     \
    {                                                   \
        int ach_intr_cnt = 0;                           \
        do { (expr); }                                  \
        while( (test) &&                                \
               EINTR == errno &&                        \
               ach_intr_cnt++  < ACH_INTR_RETRY         \
            );                                          \
    }


static enum ach_status
channel_exists_as_shm_device(const char* name);

static struct timespec
ts_mk( time_t s, long ns );

static struct timespec
ts_add(struct timespec a, struct timespec b);

static struct timespec
ts_sub(struct timespec a, struct timespec b);

static struct timespec
abs_time(struct timespec delta);

static int
channel_name_ok( const char *name );

static enum ach_status
channel_exists_as_kernel_device(const char* name);

static enum ach_status
channel_exists_as_shm_device(const char* name);

static enum ach_status
shmfile_for_channel_name( const char *name, char *buf, size_t n );

static enum ach_status
charfile_for_channel_name( const char *name, char *buf, size_t n );

#include "libach_posix.c"
#include "libach_klinux.c"

size_t ach_channel_size = sizeof(ach_channel_t);
size_t ach_attr_size = sizeof(ach_attr_t);

const char *ach_result_to_string(ach_status_t result) {

    switch(result) {
    case ACH_OK: return "ACH_OK";
    case ACH_OVERFLOW: return "ACH_OVERFLOW";
    case ACH_INVALID_NAME: return "ACH_INVALID_NAME";
    case ACH_BAD_SHM_FILE: return "ACH_BAD_SHM_FILE";
    case ACH_FAILED_SYSCALL: return "ACH_FAILED_SYSCALL";
    case ACH_STALE_FRAMES: return "ACH_STALE_FRAMES";
    case ACH_MISSED_FRAME: return "ACH_MISSED_FRAME";
    case ACH_TIMEOUT: return "ACH_TIMEOUT";
    case ACH_CLOSED: return "ACH_CLOSED";
    case ACH_EEXIST: return "ACH_EEXIST";
    case ACH_ENOENT: return "ACH_ENOENT";
    case ACH_BUG: return "ACH_BUG";
    case ACH_EINVAL: return "ACH_EINVAL";
    case ACH_CORRUPT: return "ACH_CORRUPT";
    case ACH_BAD_HEADER: return "ACH_BAD_HEADER";
    case ACH_EACCES: return "ACH_EACCES";
    case ACH_CANCELED: return "ACH_CANCELED";
    case ACH_EFAULT: return "ACH_EFAULT";
    case ACH_EINTR: return "ACH_EINTR";
    }
    return "UNKNOWN";

}

/* returns 0 if channel name is bad */
static int channel_name_ok( const char *name ) {
    size_t len;
    /* check size */
    if( (len = strlen( name )) >= ACH_CHAN_NAME_MAX )
        return 0;
    /* check hidden file */
    if( name[0] == '.' ) return 0;
    /* check bad characters */
    size_t i;
    for( i = 0; i < len; i ++ ) {
        if( ! ( isalnum( name[i] )
                || (name[i] == '-' )
                || (name[i] == '_' )
                || (name[i] == '.' ) ) )
            return 0;
    }
    return 1;
}

static struct timespec
ts_mk( time_t s, long ns )
{
    long b = (long)1e9;
    /* actual modulus, not remaninder ala % */
    long ns1 = ((ns % b) + b) % b;
    struct timespec ts = { s + (ns-ns1)/b,
                           ns1 };
    return ts;
}

static struct timespec
ts_sub(struct timespec t1, struct timespec t0)
{
    struct timespec delta = {0,0};
    /* bound at zero */
    if (t0.tv_sec > t1.tv_sec)
        return delta;
    if (t0.tv_sec == t0.tv_sec &&
        t0.tv_nsec > t1.tv_nsec)
        return delta;

    return ts_mk( t1.tv_sec - t0.tv_sec,
                  t1.tv_nsec - t0.tv_nsec );
}

static struct timespec
ts_add(struct timespec a, struct timespec b)
{
    return ts_mk( a.tv_sec + b.tv_sec,
                  a.tv_nsec + b.tv_nsec );
}

static struct timespec
abs_time(struct timespec delta)
{
    // TODO: support alternate clocks
    struct timespec now;

    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    return ts_add( now, delta );
}


void ach_create_attr_init( ach_create_attr_t *attr ) {
    memset( attr, 0, sizeof( ach_create_attr_t ) );
}

ach_create_attr_t default_create_attr = {
    .map = ACH_MAP_USER,
    .clock = ACH_DEFAULT_CLOCK
};

enum ach_status
ach_create( const char *channel_name,
            size_t frame_cnt, size_t frame_size,
            ach_create_attr_t *attr)
{
    if( NULL == attr ) attr = &default_create_attr;

    switch( attr->map ) {
    case ACH_MAP_KERNEL:
        return ach_create_klinux(channel_name, frame_cnt, frame_size);
    case ACH_MAP_ANON:
    case ACH_MAP_USER:
    case ACH_MAP_DEFAULT:
        return ach_create_posix( channel_name, frame_cnt, frame_size, attr );
    default:
        return ACH_EINVAL;
    }
}

static ach_attr_t default_ach_attr = {
    .map = ACH_MAP_DEFAULT
};

enum ach_status
ach_open( ach_channel_t *chan, const char *channel_name,
          ach_attr_t *attr )
{
    if( NULL == attr ) attr = &default_ach_attr;
    enum ach_map map = attr->map;

    if( ACH_MAP_DEFAULT == map ) {
        map = (ACH_OK == channel_exists_as_kernel_device(channel_name)) ?
            ACH_MAP_KERNEL : ACH_MAP_USER;
    }

    switch( map ) {
    case ACH_MAP_ANON:
    case ACH_MAP_USER:
        return ach_open_posix( chan, channel_name, attr );
    case ACH_MAP_KERNEL:
        return ach_open_klinux( chan, channel_name, attr );
    default:
        return ACH_EINVAL;
    }
}

enum ach_status
ach_get( ach_channel_t *chan, void *buf, size_t size,
         size_t *frame_size,
         const struct timespec *ACH_RESTRICT timeout,
         int options )
{
    switch( chan->attr.map ) {
    case ACH_MAP_KERNEL:
        return ach_get_klinux(chan, &size, (char**)&buf, frame_size, timeout, options);
    case ACH_MAP_ANON:
    case ACH_MAP_USER:
        return ach_get_posix( chan, buf, size, frame_size, timeout, options );
    default:
        return ACH_EINVAL;
    }
}


enum ach_status
ach_flush( ach_channel_t *chan )
{
    switch( chan->attr.map ) {
    case ACH_MAP_KERNEL:
        return ach_flush_klinux(chan);
    case ACH_MAP_ANON:
    case ACH_MAP_USER:
        return ach_flush_impl(chan);
    default:
        return ACH_EINVAL;
    }
}

enum ach_status
ach_put( ach_channel_t *chan, const void *buf, size_t len )
{
    switch( chan->attr.map ) {
    case ACH_MAP_KERNEL:
        return ach_put_klinux( chan, buf, len);
    case ACH_MAP_ANON:
    case ACH_MAP_USER:
        return ach_xput_posix( chan, put_fun_posix, &len, buf, len );
    default:
        return ACH_EINVAL;
    }
}

enum ach_status
ach_close( ach_channel_t *chan ) {

    switch( chan->attr.map )  {
    case ACH_MAP_ANON: return ACH_OK;  /* FIXME: What to do here? */
    case ACH_MAP_USER:
    {
        enum ach_status r;
        if( ACH_OK != (r = check_guards(chan->shm)) ) return r;
        if( munmap(chan->shm, chan->len) ) {
            ACH_ERRF("Failed to munmap channel\n");
            return check_errno();
        }
        chan->shm = NULL;
    }
    /* fall through to close file */
    case ACH_MAP_KERNEL:
    {   /* close file */
        int i;
        SYSCALL_RETRY( i = close(chan->fd),
                       i < 0 );
        if( i < 0 ) {
            ACH_ERRF("Failed to close() channel fd\n");
            return check_errno();
        }
        return ACH_OK;
    }
    default:
        return ACH_EINVAL;
    }
    return ACH_OK;
}

void ach_dump( ach_header_t *shm ) {
    fprintf(stderr, "Magic: %x\n", shm->magic );
    fprintf(stderr, "len: %"PRIuPTR"\n", shm->len );
    fprintf(stderr, "data_size: %"PRIuPTR"\n", shm->data_size );
    fprintf(stderr, "data_head: %"PRIuPTR"\n", shm->data_head );
    fprintf(stderr, "data_free: %"PRIuPTR"\n", shm->data_free );
    fprintf(stderr, "index_head: %"PRIuPTR"\n", shm->index_head );
    fprintf(stderr, "index_free: %"PRIuPTR"\n", shm->index_free );
    fprintf(stderr, "last_seq: %"PRIu64"\n", shm->last_seq );
    fprintf(stderr, "head guard:  %"PRIx64"\n", * ACH_SHM_GUARD_HEADER(shm) );
    fprintf(stderr, "index guard: %"PRIx64"\n", * ACH_SHM_GUARD_INDEX(shm) );
    fprintf(stderr, "data guard:  %"PRIx64"\n", * ACH_SHM_GUARD_DATA(shm) );

    fprintf(stderr, "head seq:  %"PRIu64"\n",
            (ACH_SHM_INDEX(shm) +
             ((shm->index_head - 1 + shm->index_cnt) % shm->index_cnt)) -> seq_num );
    fprintf(stderr, "head size:  %"PRIuPTR"\n",
            (ACH_SHM_INDEX(shm) +
             ((shm->index_head - 1 + shm->index_cnt) % shm->index_cnt)) -> size );

}

void ach_attr_init( ach_attr_t *attr ) {
    memset( attr, 0, sizeof(ach_attr_t) );
}

enum ach_status
ach_chmod( ach_channel_t *chan, mode_t mode ) {
    return (0 == fchmod(chan->fd, mode)) ? ACH_OK : check_errno();;
}

enum ach_status
ach_unlink( const char *name ) {
    _Bool k_exists = (ACH_OK == channel_exists_as_kernel_device(name));
    _Bool s_exists = (ACH_OK == channel_exists_as_shm_device(name));

    if( ! (k_exists||s_exists) ) return ACH_ENOENT;

    if (k_exists) {
        enum ach_status r = ach_unlink_klinux(name);
        if( ACH_OK != r ) return r;
    }

    if (s_exists) {
        enum ach_status r = ach_unlink_posix(name);
        if( ACH_OK != r ) return r;
    }

    return ACH_OK;
}


void
ach_cancel_attr_init( ach_cancel_attr_t *attr ) {
    memset(attr, 0, sizeof(*attr));
}

static ach_cancel_attr_t default_cancel_attr = {.async_unsafe = 0};

enum ach_status
ach_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr ) {
    if( NULL == attr ) attr = &default_cancel_attr;
    switch( chan->attr.map ) {
    case ACH_MAP_KERNEL:
        return ach_cancel_klinux(chan, attr);
    case ACH_MAP_USER:
    case ACH_MAP_ANON:
        return ach_cancel_posix(chan, attr);
    default:
        return ACH_EINVAL;
    }
}

#ifdef HAVE_THREADS_H
// First, try the C11 standard
static _Thread_local const char *ach_errstr_var;
#elseif defined TLS
// Next, try some Autoconf magic
static TLS const char *ach_errstr_var;
#else
// Otherwise, pray this works...
static __thread const char *ach_errstr_var;
#endif

void ach_set_errstr( const char *str )
{
    ach_errstr_var = str;
}

const char *ach_errstr( void )
{
    return ach_errstr_var;
}

enum ach_status
ach_channel_fd( const struct ach_channel *channel, int *file_descriptor )
{
    *file_descriptor = channel->fd;
    return ACH_OK;
}

enum ach_status
ach_channel_mapping( const struct ach_channel *channel, enum ach_map *mapping )
{
    *mapping = channel->attr.map;
    return ACH_OK;
}

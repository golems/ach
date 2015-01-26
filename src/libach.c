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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>



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
static enum ach_status
channel_name_ok( const char *name ) {
    size_t len;
    /* check size */
    if( (len = strlen( name )) >= ACH_CHAN_NAME_MAX )
        return ACH_INVALID_NAME;
    /* check hidden file */
    if( name[0] == '.' ) return ACH_INVALID_NAME;
    /* check bad characters */
    size_t i;
    for( i = 0; i < len; i ++ ) {
        if( ! ( isalnum( name[i] )
                || (name[i] == '-' )
                || (name[i] == '_' )
                || (name[i] == '.' ) ) )
            return ACH_INVALID_NAME;
    }
    return ACH_OK;
}



ach_create_attr_t default_create_attr =
{{{
    .map = ACH_MAP_USER,
    .clock = ACH_DEFAULT_CLOCK,
    .truncate = 0,
    .set_clock = 0
}}};

void ach_create_attr_init( ach_create_attr_t *attr ) {
    *attr = default_create_attr;
}


enum ach_status
ach_create( const char *channel_name,
            size_t frame_cnt, size_t frame_size,
            ach_create_attr_t *attr)
{
    const struct ach_channel_vtab *vtab;
    enum ach_status r;

    if( NULL == channel_name ) return ACH_EINVAL;
    if( NULL == attr ) attr = &default_create_attr;
    if( 0 == frame_cnt) frame_cnt = ACH_DEFAULT_FRAME_COUNT;
    if( 0 == frame_size) frame_cnt = ACH_DEFAULT_FRAME_SIZE;

    switch( attr->map ) {
    case ACH_MAP_KERNEL:
        vtab = &ach_vtab_klinux;
        goto check_name;
    case ACH_MAP_ANON:
        vtab = &ach_vtab_anon;
        goto create;
    case ACH_MAP_USER:
    case ACH_MAP_DEFAULT:
        vtab = &ach_vtab_user;
        goto check_name;
    default:
        return ACH_EINVAL;
    }
check_name:
    r = channel_name_ok( channel_name );
    if( ACH_OK != r ) return r;
create:
    return vtab->create( channel_name, frame_cnt, frame_size, attr );
}

static ach_attr_t default_ach_attr = {
    .map = ACH_MAP_DEFAULT
};

enum ach_status
ach_open( ach_channel_t *chan, const char *channel_name,
          ach_attr_t *attr )
{
    enum ach_status r = channel_name_ok( channel_name );
    if( ACH_OK != r ) return r;

    if( NULL == attr ) attr = &default_ach_attr;
    enum ach_map map = attr->map;

    if( ACH_MAP_DEFAULT == map ) {
        map = (ACH_OK == ach_vtab_klinux.exists(channel_name)) ?
            ACH_MAP_KERNEL : ACH_MAP_USER;
    }

    switch( map ) {
    case ACH_MAP_ANON:
        chan->vtab = &ach_vtab_anon;
        break;
    case ACH_MAP_USER:
        chan->vtab = &ach_vtab_user;
        break;
    case ACH_MAP_KERNEL:
        chan->vtab = &ach_vtab_klinux;
        break;
    default:
        return ACH_EINVAL;
    }

    return chan->vtab->open( chan, channel_name, attr );
}

enum ach_status
ach_get( ach_channel_t *chan, void *buf, size_t size,
         size_t *frame_size,
         const struct timespec *ACH_RESTRICT timeout,
         int options )
{
    return chan->vtab->get( chan, buf, size, frame_size, timeout, options );
}


enum ach_status
ach_flush( ach_channel_t *chan )
{
    return chan->vtab->flush( chan );
}

enum ach_status
ach_put( ach_channel_t *chan, const void *buf, size_t len )
{
    return chan->vtab->put( chan, buf, len );
}

enum ach_status
ach_close( ach_channel_t *chan )
{
    return chan->vtab->close(chan);
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

    enum ach_status r;

    r = channel_name_ok( name );
    if( ACH_OK != r ) return r;

    _Bool k_exists = (ACH_OK == ach_vtab_klinux.exists(name));
    _Bool s_exists = (ACH_OK == ach_vtab_user.exists(name));

    if( ! (k_exists||s_exists) ) return ACH_ENOENT;

    if (k_exists) {
        r = ach_vtab_klinux.unlink(name);
        if( ACH_OK != r ) return r;
    }

    if (s_exists) {
        r = ach_vtab_user.unlink(name);
        if( ACH_OK != r ) return r;
    }

    return ACH_OK;
}


void
ach_cancel_attr_init( ach_cancel_attr_t *attr ) {
    memset(attr, 0, sizeof(*attr));
}

enum ach_status
ach_cancel_attr_set_async_unsafe( ach_cancel_attr_t *attr, int asyn_unsafe )
{
    switch( asyn_unsafe ) {
    case 0: attr->async_unsafe = 0; return ACH_OK;
    case 1: attr->async_unsafe = 1; return ACH_OK;
    default: return ACH_EINVAL;
    }
}

static ach_cancel_attr_t default_cancel_attr = {.async_unsafe = 0};

enum ach_status
ach_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr ) {
    if( NULL == attr ) attr = &default_cancel_attr;
    return chan->vtab->cancel( chan, attr );
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
ach_channel_map( const struct ach_channel *channel, enum ach_map *mapping )
{
    *mapping = channel->map;
    return ACH_OK;
}

enum ach_status
ach_channel_clock( const struct ach_channel *channel, clockid_t *clock )
{
    *clock = channel->clock;
    return ACH_OK;
}

enum ach_status
ach_create_attr_set_clock( ach_create_attr_t *attr, clockid_t clock )
{
    attr->clock = clock;
    attr->set_clock = 1;
    return ACH_OK;
}

enum ach_status
ach_create_attr_set_map( ach_create_attr_t *attr, enum ach_map map )
{
    switch(map) {
        case ACH_MAP_DEFAULT:
        case ACH_MAP_USER:
        case ACH_MAP_ANON:
        case ACH_MAP_KERNEL:
            attr->map = map;
            return ACH_OK;
    }
    return ACH_EINVAL;
}

enum ach_status
ach_create_attr_get_shm( ach_create_attr_t *attr, struct ach_header **shm )
{
    if( ACH_MAP_ANON != attr->map ) return ACH_EINVAL;
    *shm = attr->shm;
    return ACH_OK;
}

enum ach_status
ach_attr_set_shm( ach_attr_t *attr, struct ach_header *shm )
{
    attr->map = ACH_MAP_ANON;
    attr->shm = shm;
    return ACH_OK;
}

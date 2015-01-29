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
#include <sys/stat.h>

#include <string.h>
#include <inttypes.h>

#include "ach.h"
#include "ach/private_posix.h"
#include "libach/vtab.h"

#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

/* Ensure order matches the enum */
static const struct ach_channel_vtab *libach_vtabs[] = {
    &libach_vtab_user, /* default mapping is posix shm channels */
    &libach_vtab_anon,
    &libach_vtab_user,
    &libach_vtab_klinux
};

static int bad_map( enum ach_map m )
{
    return (m < 0) ||
        (m >= (sizeof(libach_vtabs) / sizeof(libach_vtabs[0])));
}

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
    case ACH_ENOTSUP: return "ACH_ENOTSUP";
    }
    return "UNKNOWN";

}

enum ach_status
libach_name_ok( const char *name )
{
    size_t len;
    if( NULL == name ) return ACH_EINVAL;
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
    const struct ach_channel_vtab *vtab, *other_vtab;
    enum ach_status r;

    /* default */
    if( NULL == attr ) attr = &default_create_attr;
    if( 0 == frame_cnt) frame_cnt = ACH_DEFAULT_FRAME_COUNT;
    if( 0 == frame_size) frame_size = ACH_DEFAULT_FRAME_SIZE;

    /* validate */
    if( bad_map(attr->map) ) return ACH_EINVAL;

    vtab = libach_vtabs[attr->map];

    r = vtab->name_ok( channel_name );
    if( ACH_OK != r ) return r;


    /* Check for existence in sister namespace.  This is theoretically
     * racy since someone else could create a channel in the sister
     * namespace after we check but before we create ours; however,
     * this is unlikely to be a practical problem. Either don't create
     * the colliding channel, or explicitly specify the namespace.
     * Still a creation lock is possible using lock files, e.g. in
     * /tmp.
     */
    if( ACH_MAP_DEFAULT == attr->map ) {
        switch( vtab->map ) {
        case ACH_MAP_USER:
            other_vtab = &libach_vtab_klinux;
            break;
        case ACH_MAP_KERNEL:
            other_vtab = &libach_vtab_user;
            break;
        default:
            other_vtab = NULL;
        }
        if( other_vtab && ACH_OK == other_vtab->exists(channel_name) ) {
            return ACH_EEXIST;
        }
    }

    /* now disptach */
    return vtab->create( channel_name, frame_cnt, frame_size, attr );
}

static ach_attr_t default_ach_attr = {
    .map = ACH_MAP_DEFAULT
};

enum ach_status
ach_open( ach_channel_t *chan, const char *channel_name,
          ach_attr_t *attr )
{
    enum ach_status r;

    if( NULL == attr ) attr = &default_ach_attr;
    if( bad_map(attr->map) ) return ACH_EINVAL;

    r = libach_vtabs[attr->map]->name_ok( channel_name );
    if( ACH_OK != r ) return r;

    if( ACH_MAP_DEFAULT == attr->map ) {
        /* Default behavior: open kernel device if it exists */
        chan->vtab = &libach_vtab_klinux;
        r = chan->vtab->open( chan, channel_name, attr );
        switch(r) {
        case ACH_ENOENT:
        case ACH_EACCES: /* expected "failures", try userspace */
            break;
        default: /* success or unexpected failure */
            return r;
        }
    }

    /* specific mapping requested, or no kernel channel */
    chan->vtab = libach_vtabs[attr->map];
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
ach_unlink( const char *name )
{
    enum ach_status r_name, r_s, r_k;

    r_name = libach_name_ok( name );
    if( ACH_OK != r_name ) return r_name;

    r_s = libach_vtab_user.unlink(name);
    if( !ach_status_match(r_s, ACH_MASK_OK | ACH_MASK_ENOENT) )
        return r_s;

    r_k = libach_vtab_klinux.unlink(name);
    if( !ach_status_match(r_k, ACH_MASK_OK | ACH_MASK_ENOENT | ACH_MASK_EACCES) )
        return r_k;

    if( ach_status_match(ACH_OK, ach_status_mask(r_s) | ach_status_mask(r_k)) ) {
        /* something was unlinked */
        return ACH_OK;
    } else {
        /* either ENOENT or EACCESS */
        return r_k;
    }
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
    return channel->vtab->fd( channel, file_descriptor );
}


enum ach_status
libach_channel_fd_ok( const struct ach_channel *channel, int *file_descriptor )
{
    *file_descriptor = channel->fd;
    return ACH_OK;
}
enum ach_status
libach_channel_fd_notsup( const struct ach_channel *channel, int *file_descriptor )
{
    (void)channel;
    *file_descriptor = -1;
    return ACH_ENOTSUP;
}

enum ach_status
ach_channel_map( const struct ach_channel *channel, enum ach_map *mapping )
{
    *mapping = channel->vtab->map;
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
ach_create_attr_set_truncate( ach_create_attr_t *attr, int truncate )
{
    attr->truncate = truncate ? 1 : 0;
    return ACH_OK;
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

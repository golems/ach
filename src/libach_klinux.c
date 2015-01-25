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


/** \file libach_posix.c
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
#include "libach_private.h"

#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>



static enum ach_status
ach_filename_klinux( const char *name, char *buf, size_t n ) {
    if( n < ACH_CHAN_NAME_MAX + 16 ) return ACH_BUG;
    strcpy( buf, ACH_CHAR_CHAN_NAME_PREFIX_PATH);
    strcat( buf, ACH_CHAR_CHAN_NAME_PREFIX_NAME);
    strncat( buf, name, ACH_CHAN_NAME_MAX );
    return ACH_OK;
}


static enum ach_status
ach_exists_klinux(const char* name)
{
    char ach_name[ACH_CHAN_NAME_MAX + 16];
    int r = ach_filename_klinux(name, ach_name, sizeof(ach_name));
    if (ACH_OK != r) return ACH_BUG;

    struct stat buf;
    if (stat (ach_name, &buf)) {
        return ACH_ENOENT;
    } else {
        return ACH_OK;
    }
}

static int
ctrl_open(void)
{
    int fd;
    SYSCALL_RETRY( fd = open(ACH_CHAR_CHAN_CTRL_NAME, O_WRONLY|O_APPEND),
                   fd < 0 );
    return fd;
}

static enum ach_status
char_close(int fd)
{
    int r;

    SYSCALL_RETRY( r = close(fd),
                   r < 0 );
    if( r ) return check_errno();
    else return ACH_OK;
}

static enum ach_status
ach_unlink_klinux(const char* channel_name)
{
    int ioctl_ret;
    int fd = ctrl_open();

    if (fd < 0) {
        ACH_ERRF ("Failed opening kernel device controller\n");
        return check_errno();;
    }

    struct ach_ctrl_unlink_ch arg;
    strncpy( arg.name, channel_name, ACH_CHAN_NAME_MAX );

    SYSCALL_RETRY( ioctl_ret = ioctl(fd, ACH_CTRL_UNLINK_CH, &arg),
                   ioctl_ret < 0 )

    enum ach_status ach_ret;
    if (ioctl_ret < 0) {
        ach_ret = check_errno();
        ACH_ERRF("Failed removing device %s\n", channel_name);
    } else {
        ach_ret = ACH_OK;
    }

    enum ach_status cr = char_close(fd);
    if( ach_ret ) return ach_ret;
    else return cr;
}


static enum ach_status
ach_create_klinux( const char *channel_name,
                   size_t frame_cnt, size_t frame_size,
                   ach_create_attr_t *attr )
{
    if (ACH_OK == ach_vtab_user.exists(channel_name))
        return ACH_EEXIST;

    int fd = ctrl_open();

    if (fd < 0) {
        ACH_ERRF("Failed opening kernel device controller\n");
        return ACH_FAILED_SYSCALL;
    }
    struct ach_ctrl_create_ch arg;
    arg.frame_cnt = frame_cnt;
    arg.frame_size = frame_size;
    arg.clock = attr->set_clock ? attr->clock : ACH_DEFAULT_CLOCK;
    strcpy(arg.name, channel_name);

    enum ach_status ach_stat = ACH_OK;
    int ioctl_ret;
    SYSCALL_RETRY( ioctl_ret = ioctl(fd, ACH_CTRL_CREATE_CH, &arg),
                   ioctl_ret < 0 );
    if (ioctl_ret < 0) {
        ACH_ERRF("Failed creating device %s\n", channel_name);
        ach_stat = check_errno();
    }

    enum ach_status cr = char_close(fd);

    if( ACH_OK == ach_stat && ACH_OK == cr ) {
        int retry = 0;
        /* Wait for device to become ready */
        enum ach_status r = ach_exists_klinux(channel_name);
        /* TODO: is there any point to waiting? */
        while ( (ACH_OK != r) && (retry++ < ACH_INTR_RETRY*10)) {
            usleep(1000);
            r = ach_exists_klinux(channel_name);
        }
    }

    if( ach_stat ) return ach_stat;
    else return cr;
}

static enum ach_status
ach_get_klinux( ach_channel_t *chan,
                void * buf, size_t size, size_t *frame_size,
                const struct timespec *ACH_RESTRICT timeout,
                int options )
{

    achk_opt_t opts;
    _Bool o_rel = options & ACH_O_RELTIME;
    struct timespec t_end;

    opts.options = options;

    if( timeout ) {
        struct timespec t_begin;
        clock_gettime( chan->clock, &t_begin );
        if( o_rel ) {
            t_end = ts_add( t_begin, *timeout );
            opts.reltime = *timeout;
        } else { /* abstime */
            t_end = *timeout;
            opts.reltime = ts_sub( t_end, t_begin );
        }
    } else {
        opts.reltime.tv_sec = 0;
        opts.reltime.tv_nsec = 0;
    }

    int cnt = 0;
    /* Retry loop to handle interrupts */
    for(;;) {
        /* set mode */
        if (memcmp(&opts, &chan->k_opts, sizeof(achk_opt_t))) {
            struct achk_opt mode;
            int ioctl_ret;
            memset(&mode, 0, sizeof(mode));
            mode.options = options;
            mode.reltime = opts.reltime;

            SYSCALL_RETRY( ioctl_ret = ioctl(chan->fd, ACH_CH_SET_MODE, &mode),
                           ioctl_ret < 0 );
            if ( ioctl_ret < 0 ) return check_errno();
            chan->k_opts = opts;
        }

        ssize_t ret = read(chan->fd, buf, size);

        if( ret >= 0 ) { /* Success! */
            *frame_size = (size_t)ret;
            return ACH_OK;
        }

        /* Failure */
        if( errno != EINTR || cnt++ > ACH_INTR_RETRY ) {
            return check_errno();
        }

        /* We were interrupted, lets go again */
        if(timeout) { /* recompute the relative timeout */
            struct timespec now;
            clock_gettime( chan->clock, &now );
            opts.reltime = ts_sub(t_end, now);
        }
    }
}

static enum ach_status
ach_put_klinux( ach_channel_t *chan, const void *obj, size_t len )
{
    ssize_t size;
    SYSCALL_RETRY( size = write(chan->fd, obj, len),
                   size < 0 );
    if (size < 0) return check_errno();
    else if ((size_t)size != len) return ACH_BUG;
    else return ACH_OK;
}

static enum ach_status
ach_flush_klinux( ach_channel_t * chan)
{
    unsigned int arg = 0;
    int r;

    SYSCALL_RETRY( r = ioctl(chan->fd, ACH_CH_FLUSH, arg),
                   r < 0 );

    return check_ret_errno(r);
}

static enum ach_status
ach_cancel_klinux( ach_channel_t *chan, const ach_cancel_attr_t *attr )
{
    unsigned int arg;
    int r;

    if (!attr)
        arg = 0;
    else
        arg = attr->async_unsafe ? ACH_CH_CANCEL_UNSAFE : 0;

    chan->cancel = 1;

    SYSCALL_RETRY( r = ioctl(chan->fd, ACH_CH_CANCEL, arg),
                   r < 0 );
    return check_ret_errno(r);
}

static int fd_for_kernel_device_channel_name(const char *name,int oflag)
{
    char dev_name[ACH_CHAN_NAME_MAX+16];
    int r = ach_filename_klinux(name, dev_name, sizeof(dev_name));
    if (ACH_OK != r) return -ACH_BUG;

    int fd;
    SYSCALL_RETRY( fd = open( dev_name, O_RDWR | oflag, 0666 ),
                   fd < 0 );

    return fd;
}

static enum ach_status
ach_open_klinux( ach_channel_t *chan, const char *channel_name,
                 ach_attr_t *attr )
{
    (void)attr;
    int fd;
    if ( (fd = fd_for_kernel_device_channel_name(channel_name,0)) < 0 ) {
        return check_errno();
    }

    /* initialize struct */
    chan->fd = fd;
    chan->map = ACH_MAP_KERNEL; /* Indicates kernel device */

    /* get other fields */
    {
        struct ach_ch_options arg;
        int ioctl_ret;
        SYSCALL_RETRY( ioctl_ret = ioctl(fd, ACH_CH_GET_OPTIONS, &arg),
                       ioctl_ret < 0 );
        chan->k_opts = arg.mode;
    }

    return ACH_OK;
}


enum ach_status
ach_close_klinux( ach_channel_t *chan ) {
    int i;
    SYSCALL_RETRY( i = close(chan->fd),
                   i < 0 );
    if( i < 0 ) {
        ACH_ERRF("Failed to close() channel fd\n");
        return check_errno();
    }
    return ACH_OK;
}

const struct ach_channel_vtab
ach_vtab_klinux = {
    .create = ach_create_klinux,
    .open = ach_open_klinux,
    .flush = ach_flush_klinux,
    .put = ach_put_klinux,
    .get = ach_get_klinux,
    .cancel = ach_cancel_klinux,
    .close = ach_close_klinux,
    .unlink = ach_unlink_klinux,
    .exists = ach_exists_klinux,
    .filename = ach_filename_klinux
};

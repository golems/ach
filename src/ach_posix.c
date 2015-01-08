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
#include "ach/impl_generic.h"

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
#define DEBUGF( ... ) fprintf(stderr, __VA_ARGS__ )
#define DEBUG_PERROR(a) perror(a)

#endif /* NDEBUG */

#define IS_KERNEL_DEVICE(chan)  (NULL == chan->shm)

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
    }
    return "UNKNOWN";

}

static enum ach_status
check_errno() {
    switch(errno) {
    case EEXIST: return ACH_EEXIST;
    case ENOENT: return ACH_ENOENT;
    case EACCES: return ACH_EACCES;
    default: return ACH_FAILED_SYSCALL;
    }
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

static enum ach_status
charfile_for_channel_name( const char *name, char *buf, size_t n ) {
    if( n < ACH_CHAN_NAME_MAX + 16 ) return ACH_BUG;
    if( !channel_name_ok(name)   ) return ACH_INVALID_NAME;
    strcpy( buf, ACH_CHAR_CHAN_NAME_PREFIX_PATH);
    strcat( buf, ACH_CHAR_CHAN_NAME_PREFIX_NAME);
    strncat( buf, name, ACH_CHAN_NAME_MAX );
    return ACH_OK;
}

static enum ach_status
shmfile_for_channel_name( const char *name, char *buf, size_t n ) {
    if( n < ACH_CHAN_NAME_MAX + 16 ) return ACH_BUG;
    if( !channel_name_ok(name)   ) return ACH_INVALID_NAME;
    strcpy( buf, ACH_SHM_CHAN_NAME_PREFIX_NAME );
    strncat( buf, name, ACH_CHAN_NAME_MAX );
    return ACH_OK;
}

static enum ach_status
channame_to_full_shm_path(const char* channame, char *buf, size_t n) {
    if (n < ACH_CHAN_NAME_MAX + 32) return ACH_BUG;
    if (!channel_name_ok(channame) ) return ACH_INVALID_NAME;

    strcpy(buf, ACH_SHM_CHAN_NAME_PREFIX_PATH);
    strcat(buf, ACH_SHM_CHAN_NAME_PREFIX_NAME );
    strncat(buf, channame, ACH_CHAN_NAME_MAX);
    return ACH_OK;
}

static enum ach_status
channel_exists_as_shm_device(const char* name)
{
    char ach_name[ACH_CHAN_NAME_MAX + 32];
    int r = channame_to_full_shm_path(name, ach_name, sizeof(ach_name));
    if (ACH_OK != r) return ACH_BUG;

    struct stat buf;
    if (stat(ach_name, &buf)) {
        return ACH_ENOENT;
    } else {
        return ACH_OK;
    }
}


static enum ach_status
channel_exists_as_kernel_device(const char* name)
{
    char ach_name[ACH_CHAN_NAME_MAX + 16];
    int r = charfile_for_channel_name(name, ach_name, sizeof(ach_name));
    if (ACH_OK != r) return ACH_BUG;

    struct stat buf;
    if (stat (ach_name, &buf)) {
        return ACH_ENOENT;
    } else {
        return ACH_OK;
    }
}

static int fd_for_kernel_device_channel_name(const char *name,int oflag)
{
    char dev_name[ACH_CHAN_NAME_MAX+16];
    int r = charfile_for_channel_name(name, dev_name, sizeof(dev_name));
    if (ACH_OK != r) return -ACH_BUG;
    int fd;
    int i = 0;

    do {
        fd = open( dev_name, O_RDWR | oflag, 0666 );
    } while( -1 == fd && EINTR == errno && i++ < ACH_INTR_RETRY);

    return fd;
}

/** Opens shm file descriptor for a channel.
    \pre name is a valid channel name
*/
static int fd_for_channel_name( const char *name, int oflag ) {
    char shm_name[ACH_CHAN_NAME_MAX + 16];
    int r = shmfile_for_channel_name( name, shm_name, sizeof(shm_name) );
    if( 0 != r ) return ACH_BUG;
    int fd;
    int i = 0;

    do {
        fd = shm_open( shm_name, O_RDWR | oflag, 0666 );
    }while( -1 == fd && EINTR == errno && i++ < ACH_INTR_RETRY);
    return fd;
}

/*****************************************************************
  START charfile / kernel module helpers
*****************************************************************/
int charfile_unlink(const char* channel_name)
{
    int fd = open(ACH_CHAR_CHAN_CTRL_NAME, O_WRONLY|O_APPEND);
    if (fd < 0) {
        DEBUGF ("Failed opening kernel device controller\n");
        return errno;
    }
    struct ach_ctrl_unlink_ch arg;
    strcpy(arg.name, channel_name);

    int ret = ioctl(fd, ACH_CTRL_UNLINK_CH, &arg);
    if (ret < 0) {
        ret = errno;
        DEBUGF("Failed removing device %s\n", channel_name);
    }

    close(fd);
    return ret;
}

enum ach_status
achk_create( const char *channel_name,
             size_t frame_cnt, size_t frame_size)
{
    if (ACH_OK == channel_exists_as_shm_device(channel_name))
        return ACH_EEXIST;

    int fd = open(ACH_CHAR_CHAN_CTRL_NAME, O_WRONLY|O_APPEND);
    if (fd < 0) {
        DEBUGF("Failed opening kernel device controller\n");
        return ACH_FAILED_SYSCALL;
    }
    struct ach_ctrl_create_ch arg;
    arg.frame_cnt = frame_cnt;
    arg.frame_size = frame_size;
    strcpy(arg.name, channel_name);

    enum ach_status ach_stat = ACH_OK;
    int ret = ioctl(fd, ACH_CTRL_CREATE_CH, &arg);
    if (ret < 0) {
        DEBUGF("Failed creating device %s\n", channel_name);
        ach_stat = check_errno();
    }

    close(fd);
    return ach_stat;
}

static struct timespec relative_time(struct timespec then)
{
    struct timespec delta;
    struct timespec now;

    clock_gettime( ACH_DEFAULT_CLOCK, &now );

    delta.tv_sec = 0;
    delta.tv_nsec = 0;

    if (now.tv_sec > then.tv_sec)
        return delta;
    if (now.tv_sec == then.tv_sec &&
        now.tv_nsec > then.tv_nsec)
        return delta;
    delta.tv_sec = then.tv_sec - now.tv_sec;
    if (now.tv_nsec > then.tv_nsec) {
        delta.tv_sec--;
        then.tv_nsec += 1e9;
    }
    delta.tv_nsec = then.tv_nsec - now.tv_sec;
    return delta;
}

static enum ach_status
achk_get( ach_channel_t *chan,
          void *cx, char **pobj,
          size_t *frame_size,
          const struct timespec *ACH_RESTRICT abstime,
          int options )
{
    size_t size = *(size_t*)cx;
    achk_opt_t opts;
    opts.options = options;
    if(abstime)
        opts.reltime =*abstime;
    else {
        opts.reltime.tv_sec = 0;
        opts.reltime.tv_nsec = 0;
    }

    if (opts.reltime.tv_sec > 10000) {
        /* Regard time as abstime - convert to relative time */
        opts.reltime = relative_time(opts.reltime);
    }
    if (memcmp(&opts, &chan->k_opts, sizeof(achk_opt_t))) {
        struct ach_ch_mode mode;
        memset(&mode, 0, sizeof(mode));

        mode.mode = options;

        mode.reltime = opts.reltime;

        if (ioctl(chan->fd, ACH_CH_SET_MODE, &mode)) {
            return ACH_FAILED_SYSCALL;
        }
        chan->k_opts = opts;
    }

    ssize_t ret = read(chan->fd, *pobj, size);
    if ( ret < 0) {
        switch (errno) {
        case EAGAIN: return ACH_STALE_FRAMES; break;
        case ETIME: return ACH_TIMEOUT; break;
        case ECANCELED: return ACH_CANCELED; break;
        default: return ACH_FAILED_SYSCALL; break;
        }
    }
    *frame_size = ret;
    return ACH_OK;
}

static enum ach_status
achk_put( ach_channel_t *chan,
          void *cx, const char *obj, size_t len )
{
    ssize_t size = write(chan->fd, obj, len);
    if (size < 0)
        return check_errno();
    return ACH_OK;
}

static enum ach_status
achk_flush( ach_channel_t * chan)
{
    unsigned int arg = 0;
    if (ioctl(chan->fd, ACH_CH_FLUSH, arg)) {
        return ACH_FAILED_SYSCALL;
    }
    return ACH_OK;
}

enum ach_status
achk_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr )
{
    unsigned int arg;

    if (!attr)
        arg = 0;
    else
        arg = attr->async_unsafe ? ACH_CH_CANCEL_UNSAFE : 0;

    chan->cancel = 1;
    if (ioctl(chan->fd, ACH_CH_CANCEL, arg)) {
        return ACH_FAILED_SYSCALL;
    }
    return ACH_OK;
}

/*****************************************************************
  END charfile / kernel module helpers
*****************************************************************/



/*! \page synchronization Synchronization
 *
 * Synchronization currently uses a simple mutex+condition variable
 * around the whole shared memory block
 *
 * Some idea for more complicated synchronization:
 *
 * Our synchronization for shared memory could work roughly like a
 * read-write lock with one one additional feature.  A reader may
 * choose to block until the next write is performed.
 *
 * This behavior could be implemented with a a state variable, a
 * mutex, two condition variables, and three counters.  One condition
 * variable is for writers, and the other for readers.  We count
 * active readers, waiting readers, and waiting writers.  If a writer
 * is waiting, readers will block until it finishes.
 *
 * It may be possible to make these locks run faster by doing some
 * CASs on the state word to handle the uncontended case.  Of course,
 * figuring out how to make this all lock free would really be
 * ideal...
 *
 *  \bug synchronization should be robust against processes terminating
 *
 * Mostly Lock Free Synchronization:
 * - Have a single word atomic sync variable
 * - High order bits are counts of writers, lower bits are counts of readers
 * - Fast path twiddles the counts.  Slow path deals with a mutex and cond-var.
 * - downside: maybe no way for priority inheritance to happen...
 *
 * Other Fancy things:
 * - Use futexes for waiting readers/writers
 * - Use eventfd to signal new data
 */

static enum ach_status
check_lock( int lock_result, ach_channel_t *chan, int is_cond_check ) {
    switch( lock_result ) {
    case ETIMEDOUT:
        if( is_cond_check ) {
            /* release mutex if cond_wait times out */
            pthread_mutex_unlock( &chan->shm->sync.mutex );
        }
        return ACH_TIMEOUT;
#ifdef HAVE_MUTEX_ROBUST
    case ENOTRECOVERABLE:
        /* Shouldn't actually get this because we always mark the
         * mutex as consistent. */
        /* We do not hold the mutex at this point */
        return ACH_CORRUPT;
    case EOWNERDEAD: /* mutex holder died */
        /* We use the dirty bit to detect unrecoverable (for now)
         * errors.  Unconditionally mark the mutex as consistent.
         * Others will detect corruption based on the dirty bit.
         */
        pthread_mutex_consistent( &chan->shm->sync.mutex );
        /* continue to check the dirty bit */
#endif /* HAVE_MUTEX_ROBUST */
    case 0: /* ok */
        if( chan->shm->sync.dirty ) {
            pthread_mutex_unlock( &chan->shm->sync.mutex );
            return ACH_CORRUPT;
        } else return ACH_OK; /* it's ok, channel is consistent */
    default:
        if( is_cond_check ) {
            /* release mutex if cond_wait fails */
            pthread_mutex_unlock( &chan->shm->sync.mutex );
        }
        return ACH_FAILED_SYSCALL;
    }

    return ACH_BUG;
}

static enum ach_status
chan_lock( ach_channel_t *chan ) {
    int i = pthread_mutex_lock( & chan->shm->sync.mutex );
    return check_lock( i, chan, 0 );
}

static enum ach_status
rdlock( ach_channel_t *chan, int wait, const struct timespec *abstime ) {

    ach_header_t *shm = chan->shm;
    {
        enum ach_status r = chan_lock(chan);
        if( ACH_OK != r ) return r;
    }
    enum ach_status r = ACH_BUG;

    while(ACH_BUG == r) {
        if( chan->cancel ) {  /* check operation cancelled */
            pthread_mutex_unlock( &shm->sync.mutex );
            r = ACH_CANCELED;
        } else if( !wait ) r = ACH_OK;                          /* check no wait */
        else if ( chan->seq_num != shm->last_seq ) r = ACH_OK;  /* check if got a frame */
        /* else condition wait */
        else {
            int i = abstime ?
                pthread_cond_timedwait( &shm->sync.cond,  &shm->sync.mutex, abstime ) :
                pthread_cond_wait( &shm->sync.cond,  &shm->sync.mutex );
            enum ach_status c = check_lock(i, chan, 1);
            if( ACH_OK != c ) r = c;
            /* check r and condition next iteration */
        }
    }

    return r;
}

static enum ach_status unrdlock( ach_header_t *shm ) {
    assert( 0 == shm->sync.dirty );
    if ( pthread_mutex_unlock( & shm->sync.mutex ) )
        return ACH_FAILED_SYSCALL;
    else return ACH_OK;
}

static enum ach_status wrlock( ach_channel_t *chan ) {

    enum ach_status r = chan_lock(chan);
    if( ACH_OK != r ) return r;

    assert( 0 == chan->shm->sync.dirty );

    chan->shm->sync.dirty = 1;

    return r;
}

static ach_status_t unwrlock( ach_header_t *shm ) {
    /* mark clean */
    assert( 1 == shm->sync.dirty );
    shm->sync.dirty = 0;

    /* unlock */
    if( pthread_mutex_unlock( & shm->sync.mutex ) )
        return ACH_FAILED_SYSCALL;

    /* broadcast to wake up waiting readers */
    if( pthread_cond_broadcast( & shm->sync.cond ) )
        return ACH_FAILED_SYSCALL;

    return ACH_OK;
}


void ach_create_attr_init( ach_create_attr_t *attr ) {
    memset( attr, 0, sizeof( ach_create_attr_t ) );
}

enum ach_status
ach_create( const char *channel_name,
            size_t frame_cnt, size_t frame_size,
            ach_create_attr_t *attr) {
    ach_header_t *shm;
    int fd;
    size_t len;

    if (attr && ACH_MAP_KERNEL == attr->map) {
        enum ach_status r =  achk_create(channel_name, frame_cnt, frame_size);
        if (ACH_OK == r) {
            int retry = 0;
            /* Wait for device to become ready */
            r = channel_exists_as_kernel_device(channel_name);
            while ( (ACH_OK != r) && (retry++ < ACH_INTR_RETRY*10)) {
                    usleep(1000);
                    r = channel_exists_as_kernel_device(channel_name);
            }
        }
        return r;
    }

    /* fixme: truncate */
    /* open shm */
    {
        len = ach_create_len( frame_cnt, frame_size );

        if( attr && attr->map_anon ) {
            /* anonymous (heap) */
            shm = (ach_header_t *) malloc( len );
            fd = -1;
        }else {
            int oflag = O_EXCL | O_CREAT;
            /* shm */
            if( ! channel_name_ok( channel_name ) )
                return ACH_INVALID_NAME;
            if( attr ) {
                if( attr->truncate ) oflag &= ~O_EXCL;
            }
            if( (fd = fd_for_channel_name( channel_name, oflag )) < 0 ) {
                return check_errno();;
            }

            { /* make file proper size */
                /* FreeBSD needs ftruncate before mmap, Linux can do either order */
                int r;
                int i = 0;
                do {
                    r = ftruncate( fd, (off_t) len );
                }while(-1 == r && EINTR == errno && i++ < ACH_INTR_RETRY);
                if( -1 == r ) {
                    DEBUG_PERROR( "ftruncate");
                    return ACH_FAILED_SYSCALL;
                }
            }

            /* mmap */
            if( (shm = (ach_header_t *)mmap( NULL, len, PROT_READ|PROT_WRITE,
                                             MAP_SHARED, fd, 0) )
                == MAP_FAILED ) {
                DEBUG_PERROR("mmap");
                DEBUGF("mmap failed %s, len: %"PRIuPTR", fd: %d\n", strerror(errno), len, fd);
                return ACH_FAILED_SYSCALL;
            }

        }

        memset( shm, 0, len );
        shm->len = len;
    }

    { /* initialize synchronization */
        { /* initialize condition variables */
            int r;
            pthread_condattr_t cond_attr;
            if( (r = pthread_condattr_init(&cond_attr)) ) {
                DEBUG_PERROR("pthread_condattr_init");
                return ACH_FAILED_SYSCALL;
            }
            /* Process Shared */
            if( ! (attr && attr->map_anon) ) {
                /* Set shared if not anonymous mapping
                   Default will be private. */
                if( (r = pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED)) ) {
                    DEBUG_PERROR("pthread_condattr_setpshared");
                    return ACH_FAILED_SYSCALL;
                }
            }
            /* Clock */
            if( attr && attr->set_clock ) {
                if( (r = pthread_condattr_setclock(&cond_attr, attr->clock)) ) {
                    DEBUG_PERROR("pthread_condattr_setclock");
                    return ACH_FAILED_SYSCALL;
                }
            } else {
                if( (r = pthread_condattr_setclock(&cond_attr, ACH_DEFAULT_CLOCK)) ) {
                    DEBUG_PERROR("pthread_condattr_setclock");
                    return ACH_FAILED_SYSCALL;
                }
            }

            if( (r = pthread_cond_init(&shm->sync.cond, &cond_attr)) ) {
                DEBUG_PERROR("pthread_cond_init");
                return ACH_FAILED_SYSCALL;
            }

            if( (r = pthread_condattr_destroy(&cond_attr)) ) {
                DEBUG_PERROR("pthread_condattr_destroy");
                return ACH_FAILED_SYSCALL;
            }
        }
        { /* initialize mutex */
            int r;
            pthread_mutexattr_t mutex_attr;
            if( (r = pthread_mutexattr_init(&mutex_attr)) ) {
                DEBUG_PERROR("pthread_mutexattr_init");
                return ACH_FAILED_SYSCALL;
            }
            if( (r = pthread_mutexattr_setpshared(&mutex_attr,
                                                  PTHREAD_PROCESS_SHARED)) ) {
                DEBUG_PERROR("pthread_mutexattr_setpshared");
                return ACH_FAILED_SYSCALL;
            }
#ifdef HAVE_MUTEX_PRIORITY_INHERIT
            /* Priority Inheritance Mutex */
            if( (r = pthread_mutexattr_setprotocol(&mutex_attr,
                                                   PTHREAD_PRIO_INHERIT)) ) {
                DEBUG_PERROR("pthread_mutexattr_setprotocol");
                return ACH_FAILED_SYSCALL;
            }
#endif
#ifdef HAVE_MUTEX_ROBUST
            /* Robust Mutex */
            if( (r = pthread_mutexattr_setrobust(&mutex_attr,
                                                 PTHREAD_MUTEX_ROBUST)) ) {
                DEBUG_PERROR("pthread_mutexattr_setrobust");
                return ACH_FAILED_SYSCALL;
            }
#endif
#ifndef NDEBUG
#ifdef HAVE_MUTEX_ERROR_CHECK
            /* Error Checking Mutex */
            if( (r = pthread_mutexattr_settype(&mutex_attr,
                                               PTHREAD_MUTEX_ERRORCHECK)) ) {
                DEBUG_PERROR("pthread_mutexattr_settype");
                return ACH_FAILED_SYSCALL;
            }
#endif /* HAVE_MUTEX_ERROR_CHECK */
#endif /* NDEBUG */
            if( (r = pthread_mutex_init(&shm->sync.mutex, &mutex_attr)) ) {
                DEBUG_PERROR("pthread_mutexattr_init");
                return ACH_FAILED_SYSCALL;
            }

            if( (r = pthread_mutexattr_destroy(&mutex_attr)) ) {
                DEBUG_PERROR("pthread_mutexattr_destroy");
                return ACH_FAILED_SYSCALL;
            }
        }
    }
    /* initialize name */
    strncpy( shm->name, channel_name, ACH_CHAN_NAME_MAX );
    /* initialize counts */
    ach_create_counts( shm, frame_cnt, frame_size );
    assert( sizeof( ach_header_t ) +
            shm->index_free * sizeof( ach_index_t ) +
            shm->data_free + 3*sizeof(uint64_t) ==  len );

    if( attr && attr->map_anon ) {
        attr->shm = shm;
    } else {
        int r;
        /* remove mapping */
        r = munmap(shm, len);
        if( 0 != r ){
            DEBUG_PERROR("munmap");
            return ACH_FAILED_SYSCALL;
        }
        /* close file */
        int i = 0;
        do {
            IFDEBUG( i, DEBUGF("Retrying close()\n") )
            r = close(fd);
        }while( -1 == r && EINTR == errno && i++ < ACH_INTR_RETRY );
        if( -1 == r ){
            DEBUG_PERROR("close");
            return ACH_FAILED_SYSCALL;
        }
    }
    return ACH_OK;
}

enum ach_status
ach_open( ach_channel_t *chan, const char *channel_name,
          ach_attr_t *attr ) {
    ach_header_t * shm;
    size_t len;
    int fd = -1;

    if ((attr && ACH_MAP_KERNEL == attr->map) ||
        ACH_OK == channel_exists_as_kernel_device(channel_name)) {

        if ( (fd = fd_for_kernel_device_channel_name(channel_name,0)) < 0 ) {
            return check_errno();
        }

        /* initialize struct */
        chan->fd = fd;
        chan->len = 0;        /* We don't care */
        chan->shm = NULL;     /* Indicates kernel device */
        chan->seq_num = 0;    /* Not used */
        chan->next_index = 0; /* Not used */
        chan->cancel = 0;

        return ACH_OK;
    }

    if( attr ) memcpy( &chan->attr, attr, sizeof(chan->attr) );
    else memset( &chan->attr, 0, sizeof(chan->attr) );

    if( attr && attr->map_anon ) {
        shm = attr->shm;
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;
    }else {
        if( ! channel_name_ok( channel_name ) )
            return ACH_INVALID_NAME;
        /* open shm */
        if( (fd = fd_for_channel_name( channel_name, 0 )) < 0 ) {
            return check_errno();
        }
        if( (shm = (ach_header_t*) mmap (NULL, sizeof(ach_header_t),
                                         PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0ul) )
            == MAP_FAILED )
            return ACH_FAILED_SYSCALL;
        if( ACH_SHM_MAGIC_NUM != shm->magic )
            return ACH_BAD_SHM_FILE;

        /* calculate mmaping size */
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;

        /* remap */
        if( -1 ==  munmap( shm, sizeof(ach_header_t) ) )
            return check_errno();

        if( (shm = (ach_header_t*) mmap( NULL, len, PROT_READ|PROT_WRITE,
                                         MAP_SHARED, fd, 0ul) )
            == MAP_FAILED )
            return check_errno();
    }

    /* Check guard bytes */
    {
        enum ach_status r = check_guards(shm);
        if( ACH_OK != r ) return r;
    }

    /* initialize struct */
    chan->fd = fd;
    chan->len = len;
    chan->shm = shm;
    chan->seq_num = 0;
    chan->next_index = 1;
    chan->cancel = 0;

    return ACH_OK;
}

static enum ach_status
get_fun(void *cx, void **obj_dst, const void *chan_src, size_t frame_size )
{
    size_t size = *(size_t*)cx;

    if( size < frame_size )
        return ACH_OVERFLOW;

    if( NULL == *obj_dst && 0 != frame_size )
        return ACH_EINVAL;

    memcpy( *obj_dst, chan_src, frame_size );
    return ACH_OK;
}

enum ach_status
ach_get( ach_channel_t *chan, void *buf, size_t size,
         size_t *frame_size,
         const struct timespec *ACH_RESTRICT abstime,
         int options )
{
    if (IS_KERNEL_DEVICE(chan)) {
        return achk_get( chan, &size, (char**)&buf, frame_size, abstime, options);
    }
    return ach_xget( chan,
                     get_fun, &size, &buf,
                     frame_size, abstime, options );
}


enum ach_status
ach_flush( ach_channel_t *chan ) {

    if (IS_KERNEL_DEVICE(chan)) {
        return achk_flush(chan);
    }

    return ach_flush_impl(chan);
}


static enum ach_status
put_fun(void *cx, void *chan_dst, const void *obj)
{
    size_t len = *(size_t*)cx;

    if( NULL == obj && 0 != len ) return ACH_EINVAL;

    memcpy( chan_dst, obj,  len );
    return ACH_OK;
}

enum ach_status
ach_put( ach_channel_t *chan, const void *buf, size_t len )
{
    if (IS_KERNEL_DEVICE(chan)) {
        return achk_put( chan, &len, buf, len);
    }
    return ach_xput( chan, put_fun, &len, buf, len );
}

enum ach_status
ach_close( ach_channel_t *chan ) {

    /* Check guard bytes */
    {
        enum ach_status r = check_guards(chan->shm);
        if( ACH_OK != r ) return r;
    }

    /* fprintf(stderr, "Closing\n"); */
    /* note the close in the channel */
    if( chan->attr.map_anon ) {
        /* FIXME: what to do here?? */
        ;
    } else {
        /* remove mapping */
        int r = munmap(chan->shm, chan->len);
        if( 0 != r ){
            DEBUGF("Failed to munmap channel\n");
            return ACH_FAILED_SYSCALL;
        }
        chan->shm = NULL;

        /* close file */
        int i = 0;
        do {
            IFDEBUG( i, DEBUGF("Retrying close()\n") )
            r = close(chan->fd);
        }while( -1 == r && EINTR == errno && i++ < ACH_INTR_RETRY );
        if( -1 == r ){
            DEBUGF("Failed to close() channel fd\n");
            return ACH_FAILED_SYSCALL;
        }
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
    char ach_name[ACH_CHAN_NAME_MAX + 16];
    enum ach_status kstat = channel_exists_as_kernel_device(name);

    if (ACH_ENOENT == channel_exists_as_shm_device(name) &&
        ACH_ENOENT == kstat) {
        return ACH_ENOENT;
    }
    if (ACH_OK == kstat) {
        enum ach_status r = charfile_unlink(name);
        return r;
    }

    enum ach_status r = shmfile_for_channel_name(name, ach_name, sizeof(ach_name));
    if( ACH_OK == r ) {
        int i = shm_unlink(ach_name);
        if( 0 == i ) {
            return  ACH_OK;
        } else {
            return check_errno();
        }
    } else {
        return r;
    }
}


void
ach_cancel_attr_init( ach_cancel_attr_t *attr ) {
    memset(attr, 0, sizeof(*attr));
}

static ach_cancel_attr_t default_cancel_attr = {.async_unsafe = 0};

enum ach_status
ach_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr ) {
    if( NULL == attr ) attr = &default_cancel_attr;

    if (IS_KERNEL_DEVICE(chan)) {
        return achk_cancel(chan, attr);
    }

    if( attr->async_unsafe ) {
        /* Don't be async safe, i.e., called from another thread */
        enum ach_status r = chan_lock(chan);
        if( ACH_OK != r ) return r;
        chan->cancel = 1;
        if( pthread_mutex_unlock( &chan->shm->sync.mutex ) ) return ACH_FAILED_SYSCALL;
        if( pthread_cond_broadcast( &chan->shm->sync.cond ) )  {
            return ACH_FAILED_SYSCALL;
        }
        return ACH_OK;
    } else {
        /* Async safe, i.e., called from from a signal handler */
        chan->cancel = 1; /* Set cancel from the parent */
        pid_t pid = fork();
        if( 0 == pid ) { /* child */
            /* Now we can touch the synchronization variables in
             * shared memory without deadlock/races */

            /* Take and release the mutex.  This ensures that the
             * previously set cancel field will be seen by the caller
             * in ach_get() before it waits on the condition variable.
             * Otherwise, there is a race between checking the cancel
             * (during a spurious wakeup) and then re-waiting on the
             * condition variable vs. setting the cancel and
             * broadcasting on the condition variable (check-cancel,
             * set-cancel, cond-broadcast, cond-wait loses the race).
             */
            /* Lock mutex */
            if( ACH_OK != chan_lock(chan) ) {
                /* TODO: pipe to pass error to parent? except, can't
                 * wait in the parent or we risk deadlock */
                DEBUG_PERROR("ach_cancel chan_lock()");
                exit(EXIT_FAILURE);
            }
            /* At this point, chan->cancel is TRUE and any waiters
             * inside ach_get() must be waiting on the condition
             * variable */
            /* Unlock Mutex */
            if( pthread_mutex_unlock( &chan->shm->sync.mutex ) ) {
                DEBUG_PERROR("ach_cancel pthread_mutex_unlock()");
                exit(EXIT_FAILURE);
            }
            /* Condition Broadcast */
            if( pthread_cond_broadcast( &chan->shm->sync.cond ) )  {
                DEBUG_PERROR("ach_cancel pthread_cond_broadcast()");
                exit(EXIT_FAILURE);
            }
            exit(EXIT_SUCCESS);
        } else if (0 < pid ) { /* parent */
            /* Can't wait around since we could be holding the mutex
             * here */
            return ACH_OK;
        } else { /* fork error */
            DEBUG_PERROR("ach_cancel fork()");
            return ACH_FAILED_SYSCALL;
        }
    }
    return ACH_BUG;
}

enum ach_status
ach_xput( ach_channel_t *chan,
          ach_put_fun transfer, void *cx, const void *obj, size_t len )
{
    if( 0 == len || NULL == transfer || NULL == chan->shm ) {
        return ACH_EINVAL;
    }

    ach_header_t *shm = chan->shm;

    /* Check guard bytes */
    {
        enum ach_status r = check_guards(shm);
        if( ACH_OK != r ) return r;
    }

    if( shm->data_size < len ) {
        return ACH_OVERFLOW;
    }

    ach_index_t *index_ar = ACH_SHM_INDEX(shm);
    uint8_t *data_ar = ACH_SHM_DATA(shm);

    if( len > shm->data_size ) return ACH_OVERFLOW;

    /* take write lock */
    wrlock( chan );

    /* find next index entry */
    ach_index_t *idx = index_ar + shm->index_head;

    /* clear entry used by index */
    if( 0 == shm->index_free ) { free_index(shm,shm->index_head); }
    else { assert(0== index_ar[shm->index_head].seq_num);}

    assert( shm->index_free > 0 );

    /* Avoid wraparound */
    if( shm->data_size - shm->data_head < len ) {
        size_t i;
        /* clear to end of array */
        assert( 0 == index_ar[shm->index_head].offset );
        for(i = (shm->index_head + shm->index_free) % shm->index_cnt;
            index_ar[i].offset > shm->data_head;
            i = (i + 1) % shm->index_cnt)
        {
            assert( i != shm->index_head );
            free_index(shm,i);
        }
        /* Set counts to beginning of array */
        if( i == shm->index_head ) {
            shm->data_free = shm->data_size;
        } else {
            shm->data_free = index_ar[oldest_index_i(shm)].offset;
        }
        shm->data_head = 0;
    }

    /* clear overlapping entries */
    size_t i;
    for(i = (shm->index_head + shm->index_free) % shm->index_cnt;
        shm->data_free < len;
        i = (i + 1) % shm->index_cnt)
    {
        if( i == shm->index_head ) {
            shm->data_free = shm->data_size;
        } else {
            free_index(shm,i);
        }
    }

    assert( shm->data_free >= len );

    if( shm->data_size - shm->data_head < len ) {
        unwrlock( shm );
        assert(0);
        return ACH_BUG;
    }

    /* transfer */
    enum ach_status r = transfer(cx, data_ar + shm->data_head, obj);
    if( ACH_OK != r ) {
        unwrlock(shm);
        return r;
    }

    /* modify counts */
    shm->last_seq++;
    idx->seq_num = shm->last_seq;
    idx->size = len;
    idx->offset = shm->data_head;

    shm->data_head = (shm->data_head + len) % shm->data_size;
    shm->data_free -= len;
    shm->index_head = (shm->index_head + 1) % shm->index_cnt;
    shm->index_free --;

    assert( shm->index_free <= shm->index_cnt );
    assert( shm->data_free <= shm->data_size );
    assert( shm->last_seq > 0 );

    /* release write lock */
    return unwrlock( shm );
}

/** Copies frame pointed to by index entry at index_offset.
 *
 *   \pre hold read lock on the channel
 *
 *   \post on success, transfer is called. seq_num and next_index fields
 *   are incremented. The variable pointed to by frame_size holds the
 *   frame size.
*/
static enum ach_status
ach_xget_from_offset( ach_channel_t *chan, size_t index_offset,
                      ach_get_fun transfer, void *cx, void **pobj,
                      size_t *frame_size ) {
    ach_header_t *shm = chan->shm;
    assert( index_offset < shm->index_cnt );
    ach_index_t *idx = ACH_SHM_INDEX(shm) + index_offset;
    /* assert( idx->size ); */
    assert( idx->seq_num );
    assert( idx->offset < shm->data_size );
    /* check idx */
    if( chan->seq_num > idx->seq_num ) {
        fprintf(stderr,
                "ach bug: chan->seq_num (%"PRIu64") > idx->seq_num (%"PRIu64")\n"
                "ach bug: index offset: %"PRIuPTR"\n",
                chan->seq_num, idx->seq_num,
                index_offset );
        return ACH_BUG;
    }

    if( idx->offset + idx->size > shm->data_size ) {
        return ACH_CORRUPT;
    }

    /* good to copy */
    uint8_t *data_buf = ACH_SHM_DATA(shm);
    *frame_size = idx->size;
    enum ach_status r = transfer(cx, pobj, data_buf + idx->offset, idx->size);
    if( ACH_OK == r ) {
        chan->seq_num = idx->seq_num;
        chan->next_index = (index_offset + 1) % shm->index_cnt;
    }
    return r;
}

enum ach_status
ach_xget( ach_channel_t *chan,
          ach_get_fun transfer, void *cx, void **pobj,
          size_t *frame_size,
          const struct timespec *ACH_RESTRICT abstime,
          int options )
{
    ach_header_t *shm = chan->shm;
    ach_index_t *index_ar = ACH_SHM_INDEX(shm);

    /* Check guard bytes */
    {
        enum ach_status r = check_guards(shm);
        if( ACH_OK != r ) return r;
    }

    const bool o_wait = options & ACH_O_WAIT;
    const bool o_last = options & ACH_O_LAST;
    const bool o_copy = options & ACH_O_COPY;

    /* take read lock */
    {
        enum ach_status r = rdlock( chan, o_wait, abstime );
        if( ACH_OK != r ) return r;
    }
    assert( chan->seq_num <= shm->last_seq );

    enum ach_status retval = ACH_BUG;
    bool missed_frame = 0;

    /* get the data */
    if( (chan->seq_num == shm->last_seq && !o_copy) || 0 == shm->last_seq ) {
        /* no entries */
        assert(!o_wait);
        retval = ACH_STALE_FRAMES;
    } else {
        /* Compute the index to read */
        size_t read_index;
        if( o_last ) {
            /* normal case, get last */
            read_index = last_index_i(shm);
        } else if (!o_last &&
                   index_ar[chan->next_index].seq_num == chan->seq_num + 1) {
            /* normal case, get next */
            read_index = chan->next_index;
        } else {
            /* exception case, figure out which frame */
            if (chan->seq_num == shm->last_seq) {
                /* copy last */
                assert(o_copy);
                read_index = last_index_i(shm);
            } else {
                /* copy oldest */
                read_index = oldest_index_i(shm);
            }
        }

        if( index_ar[read_index].seq_num > chan->seq_num + 1 ) { missed_frame = 1; }

        /* read from the index */
        retval = ach_xget_from_offset( chan, read_index,
                                       transfer, cx, pobj,
                                       frame_size );

        assert( index_ar[read_index].seq_num > 0 );
    }

    /* release read lock */
    ach_status_t r = unrdlock( shm );
    if( ACH_OK != r ) return r;

    return (ACH_OK == retval && missed_frame) ? ACH_MISSED_FRAME : retval;
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

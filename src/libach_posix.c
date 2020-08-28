/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2014, Georgia Tech Research Corporation
 * Copyright (C) 2015, Rice University
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
#include "libach_private.h"
#include "libach/vtab.h"


#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>



#include "ach/impl_generic.h"


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
        ACH_ERRF("ach corrupt: mutex not recoverable\n");
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
            ACH_ERRF("ach corrupt: channel is dirty\n");
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


static enum ach_status
libach_filename_user(const char* channame, char *buf, size_t n) {
    if (n < ACH_CHAN_NAME_MAX + 32) return ACH_BUG;

    strcpy(buf, ACH_SHM_CHAN_NAME_PREFIX_PATH);
    strcat(buf, ACH_SHM_CHAN_NAME_PREFIX_NAME );
    strncat(buf, channame, ACH_CHAN_NAME_MAX);
    return ACH_OK;
}

static enum ach_status
libach_exists_user(const char* name)
{
    char ach_name[ACH_CHAN_NAME_MAX + 32];
    int r = libach_filename_user(name, ach_name, sizeof(ach_name));
    if (ACH_OK != r) return ACH_BUG;

    struct stat buf;
    if (stat(ach_name, &buf)) {
        return ACH_ENOENT;
    } else {
        return ACH_OK;
    }
}

static enum ach_status
get_fun_posix(void *cx, void **obj_dst, const void *chan_src, size_t frame_size )
{
    size_t size = *(size_t*)cx;

    if( size < frame_size )
        return ACH_OVERFLOW;

    if( NULL == *obj_dst && 0 != frame_size )
        return ACH_EINVAL;

    memcpy( *obj_dst, chan_src, frame_size );
    return ACH_OK;
}

static enum ach_status
put_fun_posix(void *cx, void *chan_dst, const void *obj)
{
    size_t len = *(size_t*)cx;

    if( NULL == obj && 0 != len ) return ACH_EINVAL;

    memcpy( chan_dst, obj,  len );
    return ACH_OK;
}

static enum ach_status
shmfile_for_channel_name( const char *name, char *buf, size_t n ) {
    if( n < ACH_CHAN_NAME_MAX + 16 ) return ACH_BUG;
    strcpy( buf, ACH_SHM_CHAN_NAME_PREFIX_NAME );
    strncat( buf, name, ACH_CHAN_NAME_MAX );
    return ACH_OK;
}

/** Opens shm file descriptor for a channel.
    \pre name is a valid channel name
*/
static int fd_for_channel_name( const char *name, int oflag ) {
    int fd;
    char shm_name[ACH_CHAN_NAME_MAX + 16];
    enum ach_status r = shmfile_for_channel_name( name, shm_name, sizeof(shm_name) );
    if( ACH_OK != r ) return r;

    SYSCALL_RETRY( fd = shm_open( shm_name, O_RDWR | oflag, 0666 ),
                   fd < 0 )
    return fd;
}



static enum ach_status
libach_create_posix( const char *channel_name,
            size_t frame_cnt, size_t frame_size,
            ach_create_attr_t *attr)
{
    ach_header_t *shm;
    int fd;
    size_t len;

    /* fixme: truncate */
    /* open shm */
    {
        len = ach_create_len( frame_cnt, frame_size );

        if( ACH_MAP_ANON == attr->map ) {
            /* anonymous (heap) */
            /* TODO: free this if something fails later */
            shm = (ach_header_t *) malloc( len );
            fd = -1;
        }else {
            int oflag = O_EXCL | O_CREAT;
            /* shm */
            if( attr->truncate ) oflag &= ~O_EXCL;
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
                ACH_ERRF("mmap failed %s, len: %"PRIuPTR", fd: %d\n", strerror(errno), len, fd);
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
            if( ! (ACH_MAP_ANON == attr->map) ) {
                /* Set shared if not anonymous mapping
                   Default will be private. */
                if( (r = pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED)) ) {
                    DEBUG_PERROR("pthread_condattr_setpshared");
                    return ACH_FAILED_SYSCALL;
                }
            }
            /* Clock */
            shm->clock = attr->set_clock ? attr->clock : ACH_DEFAULT_CLOCK;
            if( (r = pthread_condattr_setclock(&cond_attr, shm->clock)) ) {
                DEBUG_PERROR("pthread_condattr_setclock");
                return ACH_FAILED_SYSCALL;
            }
            /* Cond init */
            if( (r = pthread_cond_init(&shm->sync.cond, &cond_attr)) ) {
                DEBUG_PERROR("pthread_cond_init");
                return ACH_FAILED_SYSCALL;
            }

            /* Attr destroy */
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
    /* initialize counts */
    ach_create_counts( shm, channel_name, frame_cnt, frame_size );
    assert( sizeof( ach_header_t ) +
            shm->index_free * sizeof( ach_index_t ) +
            shm->data_free + 3*sizeof(uint64_t) ==  len );

    if( ACH_MAP_ANON == attr->map ) {
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
            IFDEBUG( i, ACH_ERRF("Retrying close()\n") )
            r = close(fd);
        }while( -1 == r && EINTR == errno && i++ < ACH_INTR_RETRY );
        if( -1 == r ){
            DEBUG_PERROR("close");
            return ACH_FAILED_SYSCALL;
        }
    }
    return ACH_OK;
}

static enum ach_status
libach_get_posix( ach_channel_t *chan, void *buf, size_t size,
                  size_t *frame_size,
                  const struct timespec *ACH_RESTRICT timeout,
                  int options )
{
    const struct timespec *ptime;
    struct timespec ltime;
    bool o_rel = options & ACH_O_RELTIME;

    if (timeout && o_rel) {
        /* timeout given as relative time */
        ltime = abs_time(chan->clock, *timeout);
        ptime = &ltime;
    } else {
        /* timeout is absolute or NULL */
        ptime = timeout;
    }
    return ach_xget( chan,
                     get_fun_posix, &size, &buf,
                     frame_size, ptime, options );
}

static enum ach_status
libach_cancel_posix( ach_channel_t *chan, const ach_cancel_attr_t *attr )
{
    /* The user mode ach_cancel() is really a fancy way of
     * interrupting a pthread condition wait.  If called from another
     * thread (async_unsafe=true), one can just take the mutex, set a
     * flag, and broadcast the condition to wakeup the other threads.
     * However, if called from signal handler (async_unsafe=false), we
     * cannot issue any pthread_* calls because they are not in the
     * list of async safe functions.  Specifically, we cannot take the
     * mutex because if also held outside the signal handler, this
     * will deadlock.  Updating the condition without the mutex held
     * is racy.  The workaround is to call fork() (which is async
     * safe) from the signal handler and then take the mutex_lock in
     * the child process.
     */
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

static enum ach_status
libach_open_posix( ach_channel_t *chan, const char *channel_name,
                   ach_attr_t *attr )
{
    ach_header_t * shm;
    size_t len;
    int fd = -1;
    clockid_t clock;

    if( ACH_MAP_ANON == attr->map ) {
        shm = attr->shm;
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;
        clock = ACH_DEFAULT_CLOCK;
    } else {
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
            == MAP_FAILED ) {
            return check_errno();
        }

        clock = shm->clock;
    }

    enum ach_status r = check_guards(shm);
    if( ACH_OK == r ) {
        chan->fd = fd;
        chan->len = len;
        chan->shm = shm;
        chan->seq_num = 0;
        chan->next_index = 0;
        chan->cancel = 0;
        chan->clock = clock;
    }

    return r;
}

static enum ach_status
libach_unlink_user( const char *name )
{
    char ach_name[ACH_CHAN_NAME_MAX + 16];
    enum ach_status r = shmfile_for_channel_name(name, ach_name, sizeof(ach_name));
    if( ACH_OK == r ) {
        if( shm_unlink(ach_name) ) return check_errno();
        else return ACH_OK;
    } else {
        return r;
    }
}

static enum ach_status
libach_put_posix( ach_channel_t *chan, const void *buf, size_t len )
{
    return ach_xput( chan, put_fun_posix, &len, buf, len );
}


static enum ach_status
libach_flush_posix( ach_channel_t *chan )
{
    return ach_flush_impl(chan);
}


static enum ach_status
libach_close_anon( ach_channel_t *chan )
{
    (void) chan;
    /* TODO: anything here? */
    return ACH_OK;
}


static enum ach_status
libach_close_user( ach_channel_t *chan ) {

    enum ach_status r;
    int i;

    if( ACH_OK != (r = check_guards(chan->shm)) ) return r;
    if( munmap(chan->shm, chan->len) ) {
        ACH_ERRF("Failed to munmap channel\n");
        return check_errno();
    }
    chan->shm = NULL;

    SYSCALL_RETRY( i = close(chan->fd),
                   i < 0 );
    if( i < 0 ) {
        ACH_ERRF("Failed to close() channel fd\n");
        return check_errno();
    }
    return ACH_OK;
}

static enum ach_status
libach_exists_anon( const char *name )
{
    (void) name;
    return ACH_EINVAL;
}

static enum ach_status
libach_filename_anon( const char *name, char *buf, size_t n )
{
    (void) name; (void) buf; (void) n;
    return ACH_EINVAL;
}

static enum ach_status
libach_unlink_anon( const char *name )
{
    (void) name;
    return ACH_EINVAL;
}

static enum ach_status
name_ok_always( const char *name )
{
    (void) name;
    return ACH_OK;
}

const struct ach_channel_vtab
libach_vtab_user = {
    .map = ACH_MAP_USER,
    .create = libach_create_posix,
    .open = libach_open_posix,
    .flush = libach_flush_posix,
    .put = libach_put_posix,
    .get = libach_get_posix,
    .cancel = libach_cancel_posix,
    .close = libach_close_user,
    .unlink = libach_unlink_user,
    .exists = libach_exists_user,
    .filename = libach_filename_user,
    .fd = libach_channel_fd_notsup,
    .name_ok = libach_name_ok
};

const struct ach_channel_vtab
libach_vtab_anon = {
    .map = ACH_MAP_ANON,
    .create = libach_create_posix,
    .open = libach_open_posix,
    .flush = libach_flush_posix,
    .put = libach_put_posix,
    .get = libach_get_posix,
    .cancel = libach_cancel_posix,
    .close = libach_close_anon,
    .unlink = libach_unlink_anon,
    .exists = libach_exists_anon,
    .filename = libach_filename_anon,
    .fd = libach_channel_fd_notsup,
    .name_ok = name_ok_always
};

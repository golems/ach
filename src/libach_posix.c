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
    if( !channel_name_ok(name)   ) return ACH_INVALID_NAME;
    strcpy( buf, ACH_SHM_CHAN_NAME_PREFIX_NAME );
    strncat( buf, name, ACH_CHAN_NAME_MAX );
    return ACH_OK;
}

/** Opens shm file descriptor for a channel.
    \pre name is a valid channel name
*/
static int fd_for_channel_name( const char *name, int oflag ) {
    char shm_name[ACH_CHAN_NAME_MAX + 16];
    int r = shmfile_for_channel_name( name, shm_name, sizeof(shm_name) );
    if( 0 != r ) return ACH_BUG;

    int fd;
    SYSCALL_RETRY( fd = shm_open( shm_name, O_RDWR | oflag, 0666 ),
                   fd < 0 )
    return fd;
}



static enum ach_status
ach_create_posix( const char *channel_name,
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
            shm = (ach_header_t *) malloc( len );
            fd = -1;
        }else {
            int oflag = O_EXCL | O_CREAT;
            /* shm */
            if( ! channel_name_ok( channel_name ) )
                return ACH_INVALID_NAME;
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
            if( attr->set_clock ) {
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

    if( ACH_MAP_ANON == attr->map_anon ) {
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

enum ach_status
ach_get_posix( ach_channel_t *chan, void *buf, size_t size,
               size_t *frame_size,
               const struct timespec *ACH_RESTRICT timeout,
               int options )
{
    const struct timespec *ptime;
    struct timespec ltime;
    _Bool o_rel = options & ACH_O_RELTIME;

    if (timeout && o_rel) {
        /* timeout given as relative time */
        ltime = abs_time(*timeout);
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
ach_xput_posix( ach_channel_t *chan,
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

static enum ach_status
ach_cancel_posix( ach_channel_t *chan, const ach_cancel_attr_t *attr )
{
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
ach_open_posix( ach_channel_t *chan, const char *channel_name,
                ach_attr_t *attr )
{
    ach_header_t * shm;
    size_t len;
    int fd = -1;
    enum ach_map map;

    if( ACH_MAP_ANON == attr->map ) {
        map = ACH_MAP_ANON;
        shm = attr->shm;
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;
    } else {
        map = ACH_MAP_USER;
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
            == MAP_FAILED ) {
            return check_errno();
        }

    }

    enum ach_status r = check_guards(shm);
    if( ACH_OK == r ) {
        chan->fd = fd;
        chan->len = len;
        chan->shm = shm;
        chan->seq_num = 0;
        chan->next_index = 1;
        chan->cancel = 0;
        chan->attr.map = map;
    }

    return r;
}

static enum ach_status
ach_unlink_posix( const char *name )
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

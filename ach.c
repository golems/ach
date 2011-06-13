/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \file ach.c
 *  \author Neil T. Dantam
 */

/// Use handy GNU extensions
#define _GNU_SOURCE

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>

#include <string.h>

#include "ach.h"

// verbosity output levels
/*
//#define WARN 0  ///< verbosity level for warnings
//#define INFO 1  ///< verbosity level for info messages
//#define DEBUG 2 ///< verbosity level for debug messages

// print a debug messages at some level
//#define DEBUGF(level, fmt, a... )\
//(((args.verbosity) >= level )?fprintf( stderr, (fmt), ## a ) : 0);
*/

/// macro to print debug messages
#define DEBUGF(fmt, a... )                      \
    fprintf(stderr, (fmt), ## a )

/// macro to do things when debugging
#define IFDEBUG( x ) (x)


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
    }
    return "UNKNOWN";

}

static int check_errno() {
    switch(errno) {
    case EEXIST: return ACH_EEXIST;
    case ENOENT: return ACH_ENOENT;
    default: return ACH_FAILED_SYSCALL;
    }
}


// returns 0 if channel name is bad
static int channel_name_ok( const char *name ) {
    size_t len;
    // check size
#ifdef HAVE_STRNLEN
    if( (len = strnlen( name, ACH_CHAN_NAME_MAX + 1 )) >= ACH_CHAN_NAME_MAX )
#else
        if( (len = strlen( name )) >= ACH_CHAN_NAME_MAX )
#endif
            return 0;
    // check hidden file
    if( name[0] == '.' ) return 0;
    // check bad characters
    for( size_t i = 0; i < len; i ++ ) {
        if( ! ( isalnum( name[i] )
                || (name[i] == '-' )
                || (name[i] == '_' )
                || (name[i] == '.' ) ) )
            return 0;
    }
    return 1;
}

/** Opens shm file descriptor for a channel.
    \pre name is a valid channel name
*/
static int fd_for_channel_name( const char *name, int oflag ) {
    char shm_name[ACH_CHAN_NAME_MAX + 16];
    strncpy( shm_name, "/achshm-", 9 );
    strncat( shm_name, name, ACH_CHAN_NAME_MAX );
    int fd;
    int i = 0;
    do {
        fd = shm_open( shm_name, O_RDWR | oflag, 0666 );
    }while( -1 == fd && EINTR == errno && i++ < ACH_INTR_RETRY);
    return fd;
}



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
 */


static int rdlock_wait( ach_header_t *shm, ach_channel_t *chan,
                        const struct timespec *abstime ) {

    int r;
    r = pthread_mutex_lock( & shm->sync.mutex );
    assert( 0 == r );
    assert( 0 == shm->sync.dirty );
    // if chan is passed, we wait for new data
    // otherwise just return holding the lock
    while( chan &&
           chan->seq_num == shm->last_seq ) {

        if( ACH_CHAN_STATE_CLOSED == shm->state ) {
            pthread_mutex_unlock( &shm->sync.mutex );
            return ACH_CLOSED;
        }
        if( abstime ) { // timed wait
            r = pthread_cond_timedwait( &shm->sync.cond,  &shm->sync.mutex, abstime );
            // check for timeout
            if( ETIMEDOUT == r ){
                pthread_mutex_unlock( &shm->sync.mutex );
                return ACH_TIMEOUT;
            }
        } else { // wait forever
            r = pthread_cond_wait( &shm->sync.cond,  &shm->sync.mutex );
        }
    }
    return ACH_OK;
}

static void rdlock( ach_header_t *shm ) {
    rdlock_wait( shm, NULL, NULL );
}

static void unrdlock( ach_header_t *shm ) {
    int r;
    assert( 0 == shm->sync.dirty );
    r = pthread_mutex_unlock( & shm->sync.mutex );
    assert( 0 == r );
}

static void wrlock( ach_header_t *shm ) {
    int r = pthread_mutex_lock( & shm->sync.mutex );
    assert( 0 == shm->sync.dirty );
    shm->sync.dirty = 1;
    assert( 0 == r );
}

static void unwrlock( ach_header_t *shm ) {
    int r;

    // mark clean
    assert( 1 == shm->sync.dirty );
    shm->sync.dirty = 0;

    // broadcast to wake up waiting readers
    r = pthread_cond_broadcast( & shm->sync.cond );
    assert( 0 == r );

    // unlock
    r = pthread_mutex_unlock( & shm->sync.mutex );
    assert( 0 == r );

}


void ach_create_attr_init( ach_create_attr_t *attr ) {
    memset( attr, 0, sizeof( ach_create_attr_t ) );
}

int ach_create( char *channel_name,
                size_t frame_cnt, size_t frame_size,
                ach_create_attr_t *attr) {
    ach_header_t *shm;
    int fd;
    size_t len;
    //fixme: truncate
    // open shm
    {
        len = sizeof( ach_header_t) +
            frame_cnt*sizeof( ach_index_t ) +
            frame_cnt*frame_size +
            3*sizeof(uint64_t);

        if( attr && attr->map_anon ) {
            // anonymous (heap)
            shm = (ach_header_t *) malloc( len );
            fd = -1;
        }else {
            int oflag = O_EXCL | O_CREAT;
            // shm
            if( ! channel_name_ok( channel_name ) )
                return ACH_INVALID_NAME;
            if( attr ) {
                if( attr->truncate ) oflag &= ~O_EXCL;
            }
            if( (fd = fd_for_channel_name( channel_name, oflag )) < 0 )
                return check_errno();;

            if( (shm = (ach_header_t *)mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0) )
                == MAP_FAILED )
                return ACH_FAILED_SYSCALL;

            // initialize shm
            { //make file proper size
                int r;
                int i = 0;
                do {
                    r = ftruncate( fd, (off_t) len );
                }while(-1 == r && EINTR == errno && i++ < ACH_INTR_RETRY);
                if( -1 == r )
                    return ACH_FAILED_SYSCALL;
            }
        }

        memset( shm, 0, len );
        shm->len = len;
        shm->state = ACH_CHAN_STATE_INIT;
    }

    { //initialize synchronization
        { //initialize condition variables
            int r;
            pthread_condattr_t cond_attr;
            r = pthread_condattr_init( &cond_attr );
            assert( 0 == r );
            r = pthread_condattr_setpshared( &cond_attr, PTHREAD_PROCESS_SHARED );
            assert( 0 == r );

            r = pthread_cond_init( & shm->sync.cond, &cond_attr );
            assert( 0 == r );

            r = pthread_condattr_destroy( &cond_attr );
            assert( 0 == r );
        }
        { //initialize mutex
            int r;
            pthread_mutexattr_t mutex_attr;
            r = pthread_mutexattr_init( &mutex_attr );
            assert( 0 == r );
            r = pthread_mutexattr_setpshared( &mutex_attr, PTHREAD_PROCESS_SHARED );
            assert( 0 == r );
#ifdef PTHREAD_MUTEX_ERRORCHECK_NP
            r = pthread_mutexattr_settype( &mutex_attr, PTHREAD_MUTEX_ERRORCHECK_NP );
            assert( 0 == r );
#endif

            pthread_mutex_init( &shm->sync.mutex, &mutex_attr );
            assert( 0 == r );

            r = pthread_mutexattr_destroy( &mutex_attr );
            assert( 0 == r );
        }
    }
    // initialize name
    strncpy( shm->name, channel_name, ACH_CHAN_NAME_MAX );
    //initialize counts
    shm->index_cnt = frame_cnt;
    shm->index_head = 0;
    shm->index_free = frame_cnt;
    shm->data_head = 0;
    shm->data_free = frame_cnt * frame_size;
    shm->data_size = frame_cnt * frame_size;
    assert( sizeof( ach_header_t ) +
            shm->index_free * sizeof( ach_index_t ) +
            shm->data_free + 3*sizeof(uint64_t) ==  len );

    *ACH_SHM_GUARD_HEADER(shm) = ACH_SHM_GUARD_HEADER_NUM;
    *ACH_SHM_GUARD_INDEX(shm) = ACH_SHM_GUARD_INDEX_NUM;
    *ACH_SHM_GUARD_DATA(shm) = ACH_SHM_GUARD_DATA_NUM;
    shm->magic = ACH_SHM_MAGIC_NUM;
    shm->state = ACH_CHAN_STATE_RUN;

    if( attr && attr->map_anon ) {
        attr->shm = shm;
    } else {
        int r;
        // remove mapping
        r = munmap(shm, len);
        if( 0 != r ){
            DEBUGF("Failed to munmap channel\n");
            return ACH_FAILED_SYSCALL;
        }
        // close file
        int i = 0;
        do {
            IFDEBUG( i ? DEBUGF("Retrying close()\n"):0 );
            r = close(fd);
        }while( -1 == r && EINTR == errno && i++ < ACH_INTR_RETRY );
        if( -1 == r ){
            DEBUGF("Failed to close() channel fd\n");
            return ACH_FAILED_SYSCALL;
        }
    }
    return ACH_OK;
}

int ach_open(ach_channel_t *chan, const char *channel_name,
             ach_attr_t *attr ) {
    ach_header_t * shm;
    size_t len;
    int fd = -1;

    if( attr ) memcpy( &chan->attr, attr, sizeof(chan->attr) );
    else memset( &chan->attr, 0, sizeof(chan->attr) );

    if( attr && attr->map_anon ) {
        shm = attr->shm;
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;
    }else {
        if( ! channel_name_ok( channel_name ) )
            return ACH_INVALID_NAME;
        // open shm
        if( ! channel_name_ok( channel_name ) ) return ACH_INVALID_NAME;
        if( (fd = fd_for_channel_name( channel_name, 0 )) < 0 )
            return ACH_FAILED_SYSCALL;
        if( (shm = (ach_header_t*) mmap(NULL, sizeof(ach_header_t),
                                        PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0) )
            == MAP_FAILED )
            return ACH_FAILED_SYSCALL;
        if( ACH_SHM_MAGIC_NUM != shm->magic )
            return ACH_BAD_SHM_FILE;

        // calculate mmaping size
        len = sizeof(ach_header_t) + sizeof(ach_index_t)*shm->index_cnt + shm->data_size;

        // remap
        if( -1 ==  munmap( shm, sizeof(ach_header_t) ) )
            return ACH_FAILED_SYSCALL;

        if( (shm = (ach_header_t*) mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0) )
            == MAP_FAILED )
            return ACH_FAILED_SYSCALL;
    }
    assert( ACH_SHM_MAGIC_NUM == shm->magic );
    assert( ACH_SHM_GUARD_HEADER_NUM == *ACH_SHM_GUARD_HEADER(shm) );
    assert( ACH_SHM_GUARD_INDEX_NUM == *ACH_SHM_GUARD_INDEX(shm) );
    assert( ACH_SHM_GUARD_DATA_NUM == *ACH_SHM_GUARD_DATA(shm) );


    // initialize struct
    chan->fd = fd;
    chan->len = len;
    chan->shm = shm;
    chan->seq_num = 0;
    chan->next_index = 1;

    return ACH_OK;
}


/** Copies frame pointed to by index entry at index_offset.

    \pre hold read lock on the channel

    \pre on success, buf holds the frame seq_num and next_index fields
    are incremented. The variable pointed to by size_written holds the
    number of bytes written to buf (0 on failure).
*/
static int ach_get_from_offset( ach_channel_t *chan, size_t index_offset,
                                char *buf, size_t size, size_t *frame_size, int copy ) {
    ach_header_t *shm = chan->shm;
    assert( index_offset < shm->index_cnt );
    ach_index_t *idx = ACH_SHM_INDEX(shm) + index_offset;
    assert( idx->size );
    assert( idx->seq_num );
    assert( idx->offset < shm->data_size );
    if( 0 == size || idx->size > size ) {
        // buffer overflow
        *frame_size = idx->size;
        return ACH_OVERFLOW;
    }else if(!copy && chan->seq_num >= idx->seq_num ) {
        // no new data
        assert( chan->seq_num == idx->seq_num );
        *frame_size = 0;
        if( ACH_CHAN_STATE_CLOSED == shm->state )
            return ACH_CLOSED;
        else return ACH_STALE_FRAMES;
    }else {
        //good to copy
        uint8_t *data_buf = ACH_SHM_DATA(shm);
        if( idx->offset + idx->size < shm->data_size ) {
            //simple memcpy
            memcpy( (uint8_t*)buf, data_buf + idx->offset, idx->size );
        }else {
            // wraparound memcpy
            size_t end_cnt = shm->data_size - idx->offset;
            memcpy( (uint8_t*)buf, data_buf + idx->offset, end_cnt );
            memcpy( (uint8_t*)buf + end_cnt, data_buf, idx->size - end_cnt );
        }
        *frame_size = idx->size;
        chan->seq_num = idx->seq_num;
        chan->next_index = (index_offset + 1) % shm->index_cnt;
        return ACH_OK;
    }
}


int ach_flush( ach_channel_t *chan ) {
    //int r;
    ach_header_t *shm = chan->shm;
    rdlock(shm);
    chan->seq_num = shm->last_seq;
    chan->next_index = shm->index_head;
    unrdlock(shm);
    return ACH_OK;
}

static int ach_get( ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                    const struct timespec *ACH_RESTRICT abstime,
                    int last, int wait, int copy ) {
    //FIXME: somehow gives missed frame on first get...
    ach_header_t *shm = chan->shm;
    ach_index_t *index_ar = ACH_SHM_INDEX(shm);
    assert( ACH_SHM_MAGIC_NUM == shm->magic );
    assert( ACH_SHM_GUARD_HEADER_NUM == *ACH_SHM_GUARD_HEADER(shm) );
    assert( ACH_SHM_GUARD_INDEX_NUM == *ACH_SHM_GUARD_INDEX(shm) );
    assert( ACH_SHM_GUARD_DATA_NUM == *ACH_SHM_GUARD_DATA(shm) );
    // not doing this mode stuff anymore
    //assert( ACH_MODE_SUBSCRIBE == chan->mode );

    // take read lock
    int r;
    if( wait ) {
        if( ACH_OK != (r = rdlock_wait( shm, chan, abstime ) ) )
            return r;
    } else
        rdlock( shm );

    // get frame
    int retval;
    size_t next_index;
    int missed_frame = 0;
    if(!copy && 0 == shm->last_seq ) { // no entries
        retval = ACH_STALE_FRAMES;
    } else {
        if ( last ) {
            next_index = (shm->index_head - 1 + shm->index_cnt) % shm->index_cnt;
        } else {
            next_index = chan->next_index;
            if( 0 == index_ar[next_index].size ||
                index_ar[next_index].seq_num != chan->seq_num + 1 ) {
                // we've missed a frame, find the oldest
                missed_frame = 1;
                next_index = (shm->index_head + shm->index_free) % shm->index_cnt;
            }
        }
        retval = ach_get_from_offset( chan, next_index, (char*)buf, size, frame_size, copy );
    }

    // release read lock
    unrdlock( shm );

    return (ACH_OK == retval && missed_frame) ? ACH_MISSED_FRAME : retval;

}

/* The next few functions are variations on reading from an ach channel.
 */

int ach_get_next(ach_channel_t *chan, void *buf, size_t size,
                 size_t *frame_size) {
    return ach_get( chan, buf, size, frame_size, NULL, 0, 0, 0 );
}

int ach_get_last(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size ) {
    return ach_get( chan, buf, size, frame_size, NULL, 1, 0, 0 );
}

int ach_copy_last(ach_channel_t *chan, void *buf,
                  size_t size, size_t *frame_size) {
    return ach_get( chan, buf, size, frame_size, NULL, 1, 0, 1 );

}

int ach_wait_last(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                  const struct timespec *ACH_RESTRICT abstime) {
    return ach_get( chan, buf, size, frame_size, abstime, 1, 1, 0 );
}



int ach_wait_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                  const struct timespec *ACH_RESTRICT abstime) {
    return ach_get( chan, buf, size, frame_size, abstime, 0, 1, 0 );
}


int ach_put(ach_channel_t *chan, void *buf, size_t len) {


    assert( ACH_SHM_MAGIC_NUM == chan->shm->magic );
    assert( ACH_SHM_GUARD_HEADER_NUM == *ACH_SHM_GUARD_HEADER(chan->shm) );
    assert( ACH_SHM_GUARD_INDEX_NUM == *ACH_SHM_GUARD_INDEX(chan->shm) );
    assert( ACH_SHM_GUARD_DATA_NUM == *ACH_SHM_GUARD_DATA(chan->shm) );

    // not doing this mode stuff
    //assert( ACH_MODE_PUBLISH == chan->mode );

    ach_header_t *shm = chan->shm;

    ach_index_t *index_ar = ACH_SHM_INDEX(shm);
    uint8_t *data_ar = ACH_SHM_DATA(shm);

    if( len > shm->data_size ) return ACH_OVERFLOW;

    // take write lock
    wrlock( shm );

    // find next index entry
    ach_index_t *idx = index_ar + shm->index_head;

    // clear entry used by index
    if( 0 == shm->index_free ) {
        shm->data_free += idx->size;
        shm->index_free ++;
        memset( idx, 0, sizeof( ach_index_t ) );
    }

    // clear overlapping entries
    size_t i;
    for(i = (shm->index_head + shm->index_free) % shm->index_cnt;
        shm->data_free < len;
        i = (i + 1) % shm->index_cnt) {
        assert( i != shm->index_head );
        assert( 0 != index_ar[i].size );

        shm->data_free += index_ar[i].size;
        shm->index_free ++;
        memset( index_ar + i, 0, sizeof( ach_index_t ) );
    }

    // copy buffer
    if( shm->data_size - shm->data_head >= len ) {
        //simply copy
        memcpy( data_ar + shm->data_head, buf, len );
    } else {
        //wraparound copy
        size_t end_cnt = shm->data_size - shm->data_head;
        memcpy( data_ar + shm->data_head, buf, end_cnt);
        memcpy( data_ar, (uint8_t*)buf + end_cnt, len - end_cnt );
    }

    // modify counts
    idx->seq_num = ++shm->last_seq;
    idx->size = len;
    idx->offset = shm->data_head;

    shm->data_head = (shm->data_head + len) % shm->data_size;
    shm->data_free -= len;
    shm->index_head = (shm->index_head + 1) % shm->index_cnt;
    shm->index_free --;

    assert( shm->index_free <= shm->index_cnt );
    assert( shm->data_free <= shm->data_size );

    // release write lock
    unwrlock( shm );
    return ACH_OK;
}

int ach_close(ach_channel_t *chan) {


    assert( ACH_SHM_MAGIC_NUM == chan->shm->magic );
    assert( ACH_SHM_GUARD_HEADER_NUM == *ACH_SHM_GUARD_HEADER(chan->shm) );
    assert( ACH_SHM_GUARD_INDEX_NUM == *ACH_SHM_GUARD_INDEX(chan->shm) );
    assert( ACH_SHM_GUARD_DATA_NUM == *ACH_SHM_GUARD_DATA(chan->shm) );

    int r;
    //fprintf(stderr, "Closing\n");
    // note the close in the channel
    if( chan->attr.map_anon ) {
        //FIXME: what to do here??
        ;
    } else {
        // remove mapping
        r = munmap(chan->shm, chan->len);
        if( 0 != r ){
            DEBUGF("Failed to munmap channel\n");
            return ACH_FAILED_SYSCALL;
        }
        chan->shm = NULL;

        // close file
        int i = 0;
        do {
            IFDEBUG( i ? DEBUGF("Retrying close()\n"):0 );
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
    fprintf(stderr, "len: %d\n", shm->len );
    fprintf(stderr, "data_size: %d\n", shm->data_size );
    fprintf(stderr, "data_head: %d\n", shm->data_head );
    fprintf(stderr, "data_free: %d\n", shm->data_free );
    fprintf(stderr, "index_head: %d\n", shm->index_head );
    fprintf(stderr, "index_free: %d\n", shm->index_free );
    fprintf(stderr, "last_seq: %lld\n", shm->last_seq );
    fprintf(stderr, "head guard:  %llX\n", * ACH_SHM_GUARD_HEADER(shm) );
    fprintf(stderr, "index guard: %llX\n", * ACH_SHM_GUARD_INDEX(shm) );
    fprintf(stderr, "data guard:  %llX\n", * ACH_SHM_GUARD_DATA(shm) );
}

void ach_attr_init( ach_attr_t *attr ) {
    attr->shm = NULL;
    attr->map_anon = 0;
}

int ach_chmod( ach_channel_t *chan, mode_t mode ) {
    return (fchmod( chan->fd, mode ));
}

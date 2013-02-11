/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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

/** \file ach.h
 *  \author Neil T. Dantam
 */


/** \mainpage ACH IPC Library
 *
 * Ach is a library that provides a publish-subscribe or message-bus
 * form of IPC.
 *
 * -----------------------------------------------------
 *
 * A tutorial-style manual is available at
 * http://golems.github.com/ach/manual
 *
 * -----------------------------------------------------
 *
 * Ach differs from other message passing transports with regard to
 * head of line blocking.  In ach, newer messages always supersede old
 * messsages, regardless of whether or not a subscriber has seen the
 * old message.  Old messages will never block new messages.  This
 * behavior is suited to real-time systems where a subscriber is
 * generally interested only in the latest version of a message.
 *
 * Clients may be publishers and or subscribers. Publishers they push
 * data to channels, and subscribers can then poll or wait on the
 * channels for data.
 *
 * Ach deals only with byte arrays.  Any higher level data
 * organization (records, dictionaries, etc) must be handled in the
 * client application.
 *
 * \author Neil T. Dantam
 * \author Developed at the Georgia Tech Humanoid Robotics Lab
 * \author Under Direction of Professor Mike Stilman
 *
 * Copyright (c) 2008-2013, Georgia Tech Research Corporation.
 * All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   - Redistributions in binary form must reproduce the above
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



/*
 * Shared Memory Layout:
 *
 *    ________
 *   | Header |
 *   |--------|
 *   | GUARDH |
 *   |--------|
 *   | Index  |
 *   |        |
 *   |        |
 *   |--------|
 *   | GUARDI |
 *   |--------|
 *   |  Data  |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |--------|
 *   | GUARDD |
 *   |________|
 */

#ifndef ACH_H
#define ACH_H

/* restict only in C99 */
#ifdef __cplusplus
/** Alias restrict keyword */
# define ACH_RESTRICT
#elif  __STDC_VERSION__ < 199901L
/** Alias restrict keyword */
#  define ACH_RESTRICT
#else
/** Alias restrict keyword */
# define ACH_RESTRICT restrict
#endif

#if (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1))
/** Deprecated old symbol */
#define ACH_DEPRECATED  __attribute__((__deprecated__))
#else
/** Deprecated old symbol */
#define ACH_DEPRECATED
#endif /* __GNUC__ */

/* Determine a default clock */
#ifdef CLOCK_MONOTONIC
/** Default condition variable clock */
#define ACH_DEFAULT_CLOCK CLOCK_MONOTONIC
#elif defined CLOCK_HIGHRES /* Old Solaris lacks CLOCK_MONOTONIC,
                               CLOCK_HIGHRES looks the same */
/** Default condition variable clock */
#define ACH_DEFAULT_CLOCK CLOCK_HIGHRES
#elif defined CLOCK_REALTIME /* Try fallback to CLOCK_REALTIME */
/** Default condition variable clock */
#define ACH_DEFAULT_CLOCK CLOCK_REALTIME
#else
#error No valid CLOCKS defined.  Expecting CLOCK_MONOTONIC.
#endif /* CLOCK_MONOTONIC */


#ifdef __cplusplus
extern "C" {
#endif

/**  maximum size of a channel name */
#define ACH_CHAN_NAME_MAX 64ul

/** prefix to apply to channel names to get the shared memory file name */
#define ACH_CHAN_NAME_PREFIX "/achshm-"

/** Number of times to retry a syscall on EINTR before giving up */
#define ACH_INTR_RETRY 8

    /** magic number that appears the the beginning of our mmaped files.

        This is just to be used as a check.
    */
#define ACH_SHM_MAGIC_NUM 0xb07511f3


    /** A separator between different shm sections.

        This one comes after the header.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_HEADER_NUM ((uint64_t)0x1A2A3A4A5A6A7A8ALLU)
    /** A separator between different shm sections.

        This ones comes after the index array.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_INDEX_NUM ((uint64_t)0x1B2B3B4B5B6B7B8BLLU)

    /** A separator between different shm sections.

        This one comes after the data section (at the very end of the
        file).  Should aid debugging by showing we don't overstep and
        bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_DATA_NUM ((uint64_t)0x1C2C3C4C5C6C7C8CLLU)

    /** return status codes for ach functions */
    typedef enum ach_status {
        ACH_OK = 0,             /**< Call successful */
        ACH_OVERFLOW = 1,       /**< buffer to small to hold frame */
        ACH_INVALID_NAME = 2,   /**< invalid channel name */
        ACH_BAD_SHM_FILE = 3,   /**< channel file didn't look right */
        ACH_FAILED_SYSCALL = 4, /**< a system call failed */
        ACH_STALE_FRAMES = 5,   /**< no new data in the channel */
        ACH_MISSED_FRAME = 6,   /**< we missed the next frame */
        ACH_TIMEOUT = 7,        /**< timeout before frame received */
        ACH_EEXIST = 8,         /**< channel file already exists */
        ACH_ENOENT = 9,         /**< channel file doesn't exist */
        ACH_CLOSED = 10,        /**< unused */
        ACH_BUG = 11,           /**< internal ach error */
        ACH_EINVAL = 12,        /**< invalid channel */
        ACH_CORRUPT = 13,       /**< channel memory has been corrupted */
        ACH_BAD_HEADER = 14,    /**< an invalid header was given */
        ACH_EACCES = 15         /**< permission denied */
    } ach_status_t;


    /** Option flags for ach_get().
     *
     * Default behavior is to retrieve the oldest unseen frame without
     * waiting.*/
    typedef enum {
        /** Blocks until an unseen message arrives
         *  or timeout.  If the channel already has data that this subscriber
         *  has not seen, ach_get() immediately copies the new data.
         *  Otherwise, it waits for some other process or thread to put data
         *  into the channel.
         */
        ACH_O_WAIT = 0x01,
        /** Reads the newest message out of the channel.  If the channel
         * contains multiple messages that this subscriber has not seen,
         * ach_get() will return the newest of these messages.  The subscriber
         * will skip past all older messages.
         */
        ACH_O_LAST = 0x02,
        /** Copy the message out of the channel, even if already seen.
         *  Return code of ach_get() for successful copy will be ACH_OK.
         */
        ACH_O_COPY = 0x04
    } ach_get_opts_t;

    /** Header for shared memory area.
     *
     * There is no tail pointer here.  Every subscriber that opens the
     * channel must maintain its own tail pointer.
     */
    typedef struct {
        uint32_t magic;          /**< magic number of ach shm files */
        size_t len;              /**< length of mmap'ed file */
        char name[1+ACH_CHAN_NAME_MAX]; /**< Name of this channel */
        union {
            struct {
                size_t index_cnt;        /**< number of entries in index */
                size_t data_size;        /**< size of data bytes */
                size_t data_head;        /**< offset to first open byte of data */
                size_t data_free;        /**< number of free data bytes */
                size_t index_head;       /**< index into index array of first unused index entry */
                size_t index_free;       /**< number of unused index entries */
                int anon;                /**< is channel in the heap? */
            };
            uint64_t reserved[16];  /**< Reserve to compatibly add future variables */
        };
        struct /* anonymous structure */ {
            pthread_mutex_t mutex;         /**< mutex for condition variables */
            pthread_cond_t cond;           /**< condition variable */
            int dirty;
        } sync;                   /**< variables for synchronization */
        /* should force our alignment to 8-bytes... */
        uint64_t last_seq;        /**< last sequence number written */
    } ach_header_t;

    /** Entry in shared memory index array
     */
    typedef struct {
        size_t size;      /**< size of frame */
        size_t offset;    /**< byte offset of entry from beginning of data array */
        uint64_t seq_num; /**< number of frame */
    } ach_index_t ;


    /** Attributes to pass to ach_open */
    typedef struct {
        union {
            struct{
                int map_anon;        /**< anonymous channel (put it in process heap, not shm) */
                ach_header_t *shm;   /**< the memory buffer used by anonymous channels */
            };
            uint64_t reserved_size[8]; /**< Reserve space to compatibly add future options */
        };
    } ach_attr_t;

    /** Attributes to pass to ach_create  */
    typedef struct {
        union {
            struct{
                int map_anon;      /**< allocate channel in heap, rather than shm */
                ach_header_t *shm; /**< pointer to channel, set on output of create iff map_anon */
                int truncate;      /**< remove and recreate an existing shm file */
                int set_clock;     /**< if true, set the clock of the condition variable */
                clockid_t clock;   /**< Which clock to use if set_clock is true.
                                    *   The default is defined by ACH_DEFAULT_CLOCK. */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    } ach_create_attr_t;

    /** Descriptor for shared memory area
     */
    typedef struct {
        union {
            struct {
                ach_header_t *shm;   /**< pointer to mmap'ed block */
                size_t len;          /**< length of memory mapping */
                int fd;              /**< file descriptor of mmap'ed file */
                uint64_t seq_num;    /**< last sequence number read */
                size_t next_index;   /**< next index entry to try get from */
                ach_attr_t attr;     /**< attributes used to create this channel */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    } ach_channel_t;

    /** Size of ach_channel_t */
    extern size_t ach_channel_size;
    /** Size of ach_attr_t */
    extern size_t ach_attr_size;

/** Gets pointer to guard uint64 following the header */
#define ACH_SHM_GUARD_HEADER( shm ) ((uint64_t*)((ach_header_t*)(shm) + 1))

/** Gets the pointer to the index array in the shm block */
#define ACH_SHM_INDEX( shm ) ((ach_index_t*)(ACH_SHM_GUARD_HEADER(shm) + 1))

/**  gets pointer to the guard following the index section */
#define ACH_SHM_GUARD_INDEX( shm )                                      \
    ((uint64_t*)(ACH_SHM_INDEX(shm) + ((ach_header_t*)(shm))->index_cnt))

/** Gets the pointer to the data buffer in the shm block */
#define ACH_SHM_DATA( shm ) ( (uint8_t*)(ACH_SHM_GUARD_INDEX(shm) + 1) )

/** Gets the pointer to the guard following data buffer in the shm block */
#define ACH_SHM_GUARD_DATA( shm )                                       \
    ((uint64_t*)(ACH_SHM_DATA(shm) + ((ach_header_t*)(shm))->data_size))


    /** Initialize attributes for opening channels. */
    void ach_attr_init( ach_attr_t *attr );

    /** Initialize attributes for creating channels. */
    void ach_create_attr_init( ach_create_attr_t *attr );

    /** Creates a new channel.
        \param channel_name Name of the channel
        \param frame_cnt number of frames to hold in circular buffer
        \param frame_size nominal size of each frame
        \param attr options
    */
    enum ach_status
    ach_create( const char *channel_name,
                size_t frame_cnt, size_t frame_size,
                ach_create_attr_t *attr );

/** Default number of index entries in a channel */
#define ACH_DEFAULT_FRAME_COUNT 16

/** Default nominal frame size for a channel */
#define ACH_DEFAULT_FRAME_SIZE 512

    /** Opens a handle to channel.
     */
    enum ach_status
    ach_open( ach_channel_t *chan, const char *channel_name,
              ach_attr_t *attr );

    /** Pulls a message from the channel.
        \pre chan has been opened with ach_open()

        \post If buf is big enough to hold the next frame, buf contains
        the data for the last frame and chan.seq_num is set to the last
        frame.  If buf is too small to hold the next frame, no side
        effects occur.  The seq_num field of chan will be set to the
        latest sequence number (that of the gotten frame).

        \param chan The previously opened channel handle
        \param buf Buffer to store data
        \param size Length of buffer in bytes
        \param frame_size The number of bytes copied to buf, or the
        size of the desired frame if buf is too small.
        \param abstime An absolute timeout if ACH_O_WAIT is specified.
        Take care that abstime is given in the correct clock.  The
        default is defined by ACH_DEFAULT_CLOCK.
        \param options Option flags
    */
    enum ach_status
    ach_get( ach_channel_t *chan, void *buf, size_t size,
             size_t *frame_size,
             const struct timespec *ACH_RESTRICT abstime,
             int options );

    /** Writes a new message in the channel.

        \pre chan has been opened with ach_open()

        \post The contents of buf are copied into the channel and the
        sequence number of the channel is incremented.

        \param chan (action) The channel to write to
        \param buf The buffer containing the data to copy into the channel
        \param len number of bytes in buf to copy, len > 0
        \return ACH_OK on success.
    */
    enum ach_status
    ach_put( ach_channel_t *chan, void *buf, size_t len );


    /** Discards all previously received messages for this handle.  Does
        not change the actual channel, just resets the sequence number in
        the handle.*/
    enum ach_status
    ach_flush( ach_channel_t *chan );

    /** Closes the shared memory block.

        \pre chan is an initialized ach channel with open shared memory area

        \post the shared memory file for chan is closed
    */
    enum ach_status
    ach_close( ach_channel_t *chan );


    /** Converts return code from ach call to a human readable string;
     */
    const char *ach_result_to_string(ach_status_t result);


    /** Prints information about the channel shm to stderr

        This function is mostly for internal debugging.
    */
    void ach_dump( ach_header_t *shm);

    /** Sets permissions of chan to specified mode */
    enum ach_status
    ach_chmod( ach_channel_t *chan, mode_t mode );

    /** Delete an ach channel */
    enum ach_status
    ach_unlink( const char *name );

    /** Format for ach frames sent over pipes or stored on disk */
    typedef struct {
        char magic[8];         /**< magic number: "achpipe", null terminated */
        uint8_t size_bytes[8]; /**< size, stored little endian for disk and network transmission */
        uint8_t data[1];       /**< flexible array */
    } ach_pipe_frame_t;

    /** Malloc an ach_pipe_frame_t with room for `size' data bytes.
     *
     * \return a newly allocated ach_pipe_frame with its magic and
     * size fields properly filled.
     */
    ach_pipe_frame_t *ach_pipe_alloc(size_t size);

    /** Set size field in ach frame, always stored little endian.
     * \param frame The frame struct
     * \param size The size in native byte order
     */
    void ach_pipe_set_size(ach_pipe_frame_t *frame, uint64_t size);

    /** Set size field in ach frame, always stored little endian.
     * \param frame The frame struct
     * \returns The size in native byte order
     */
    uint64_t ach_pipe_get_size(const ach_pipe_frame_t *frame );

#ifdef __cplusplus
}
#endif

#endif /* ACH_H */

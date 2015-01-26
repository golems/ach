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

/** \file ach.h
 *
 *  \brief This header file defines the public interface for Ach.
 *
 *  \author Neil T. Dantam
 *
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

#include <signal.h>

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

#include "ach/generic.h"

#ifdef __cplusplus
extern "C" {
#endif


     /** Options to specify the mapping for a channels backing memory
     *   buffer.
     */
    enum ach_map {
        ACH_MAP_DEFAULT = 0,  /**< Use the default mapping for channels */
        ACH_MAP_ANON = 1,     /**< anonymous channel - use heap memory */
        ACH_MAP_USER = 2,     /**< Use shared memory for channels */
        ACH_MAP_KERNEL = 3,   /**< Use kernel memory for channels - require ach kernel module being loaded */
    };


    /** Type for header in shared memory */
    struct ach_header;

    /** Attributes to pass to ach_open.
     *
     *  Library users should access this struct through the provided
     *  functions rather than directly manipulating it.
     */
    struct ach_attr {
        union {
            struct{
                union {
                    int map_anon ACH_DEPRECATED;  /**< anonymous channel (put it in process heap, not shm) */
                    enum ach_map map;            /**< Where to put channel backing memory.
                                                  *   Replaces map_anon */
                };
                struct ach_header *shm;   /**< the memory buffer used by anonymous channels */
            };
            uint64_t reserved_size[8]; /**< Reserve space to compatibly add future options */
        };
    };

    typedef struct ach_attr ach_attr_t;

    /** Initialize attributes for opening channels. */
    void ach_attr_init( ach_attr_t *attr );

    /** Set shared memory area for anonymous channels */
    enum ach_status
    ach_attr_set_shm( ach_attr_t *attr, struct ach_header *shm );

    typedef struct ach_attr ach_attr_t;

    /** Attributes to pass to ach_create.
     *
     *  Library users should access this struct through the provided
     *  functions rather than directly manipulating it.
     */
    struct ach_create_attr {
        union {
            struct{
                union {
                    int map_anon;            /**< allocate channel in heap, rather than shm */
                    enum ach_map map;        /**< replaces map_anon */
                };
                struct ach_header *shm;      /**< pointer to channel, set on output of create iff map_anon */
                clockid_t clock;             /**< Which clock to use if set_clock is true.
                                              *   The default is defined by ACH_DEFAULT_CLOCK. */
                unsigned int truncate  : 1;  /**< remove and recreate an existing shm file */
                unsigned int set_clock : 1;  /**< if true, set the clock of the condition variable */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    };

    typedef struct ach_create_attr ach_create_attr_t;

    /** Initialize attributes for creating channels. */
    void ach_create_attr_init( ach_create_attr_t *attr );

    /** Set the clockid */
    enum ach_status
    ach_create_attr_set_clock( ach_create_attr_t *attr, clockid_t clock );

    /** Set the mapping */
    enum ach_status
    ach_create_attr_set_map( ach_create_attr_t *attr, enum ach_map map );

    /** Set to truncate */
    enum ach_status
    ach_create_attr_set_truncate( ach_create_attr_t *attr, int truncate );

    /** Get backing memory for anonymous channel */
    enum ach_status
    ach_create_attr_get_shm( ach_create_attr_t *attr, struct ach_header **shm );

    /** Virtual Method Table for handling different channel mappings */
    struct ach_channel_vtab;

    /** Handle for an Ach channel.
     *
     *  Library users are strongly discourged from directly accessing
     *  members of this structure; its fields may change in future
     *  library versions.  All access should be through the provided
     *  library functions.
     */
    typedef struct ach_channel {
        union {
            struct {
                struct ach_header *shm;                /**< pointer to mmap'ed block */
                size_t len;                            /**< length of memory mapping */
                int fd;                                /**< file descriptor of mmap'ed file */
                union {
                    uint64_t seq_num;                  /**< last sequence number read */
                    achk_opt_t k_opts;                 /**< Used by kernel devices */
                };
                size_t next_index;                     /**< next index entry to try get from */
                enum ach_map map;                      /**< attributes used to create this channel */
                clockid_t clock;                       /**< attributes used to create this channel */
                volatile sig_atomic_t cancel;          /**< cancel a waiting ach_get */
                const struct ach_channel_vtab *vtab;   /**< virtual method table */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    } ach_channel_t;

    /** Return the file descriptor associated with this channel */
    enum ach_status
    ach_channel_fd( const struct ach_channel *channel, int *file_descriptor );

    /** Return the mapping of the channel. */
    enum ach_status
    ach_channel_map( const struct ach_channel *channel, enum ach_map *map );

    /** Return the clock used by the channel. */
    enum ach_status
    ach_channel_clock( const struct ach_channel *channel, clockid_t *clock );

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

    /** Opens a handle to channel.

        \post A file descriptor for the named channel is opened, and
        chan is initialized.

        \return ACH_OK on success.  Otherwise, return an error code
        indicating the particular error.
     */
    enum ach_status
    ach_open( ach_channel_t *chan, const char *channel_name,
              ach_attr_t *attr );

    /** Pulls a message from the channel.
     *
     *  If a signal is delivered to a thread executing or waiting in
     *  ach_get, the thread will resume ach_get after the signal
     *  handler completes.
     *
     *  To interrupt a call to ach_get before a new message is
     *  received, use ach_cancel.
     *
     *  \pre chan has been opened with ach_open()
     *
     *  \post If buf is big enough to hold the next frame, buf
     *  contains the data for the last frame and chan.seq_num is set
     *  to the last frame.  If buf is too small to hold the next
     *  frame, no side effects occur and ACH_OVERFLOW is returned.
     *  The seq_num field of chan will be set to the latest sequence
     *  number (that of the gotten frame).
     *
     *  \bug Linux kernel channels will return ACH_OK when they should
     *  return ACH_MISSED_FRAME.  The data is still retrieved, but
     *  library clients are not notified of the skipped messages.
     *
     *  \param[in,out] chan The previously opened channel handle
     *
     *  \param[out] buf Buffer to store data
     *
     *  \param[in] size Length of buffer in bytes
     *
     *  \param[out] frame_size The number of bytes copied to buf, or
     *                         the size of the desired frame if buf is
     *                         too small.
     *
     *  \param[in] abstime An absolute timeout if ::ACH_O_WAIT is
     *                     specified.  Take care that abstime is given
     *                     in the correct clock.  The default is
     *                     defined by ACH_DEFAULT_CLOCK.
     *
     *  \param[in] options Option flags
     *
     *  \return On success, returns ACH_OK.  Otherwise, returns an
     *          error code indication the failure:
     *
     *  - ::ACH_OK: On success, returns ::ACH_OK and the location pointed
     *            to by frame_size contains the number of bytes copied
     *            to buf.
     *
     *  - ::ACH_MISSED_FRAME: If any frames were skipped in the buffer,
     *                      the ::ACH_MISSED_FRAME may be returned.  The
     *                      data is still copied and frame_size set as
     *                      on ::ACH_OK.
     *
     *  - ::ACH_STALE_FRAMES
     *  - ::ACH_TIMEOUT
     *  - ::ACH_CANCELED
     *  - ::ACH_OVERFLOW
     *  - ::ACH_CORRUPT
     *  - ::ACH_EINVAL
     *  - ::ACH_EFAULT
     *
     * \sa ach_get_opts_t, ach_status
     *
     */
    enum ach_status
    ach_get( ach_channel_t *chan, void *buf, size_t size,
             size_t *frame_size,
             const struct timespec *ACH_RESTRICT abstime,
             int options );

    /** Writes a new message in the channel.

        \pre chan has been opened with ach_open() and is large enough
        to hold the message.

        \post The contents of buf are copied into the channel and the
        sequence number of the channel is incremented.  If the channel
        is too small to hold the message, the message is not copied
        into the channel as will not be seen by receivers.

        \param chan (action) The channel to write to
        \param buf The buffer containing the data to copy into the channel
        \param len number of bytes in buf to copy, len > 0
        \return ACH_OK on success. If the channel is too small to hold
        the frame, returns ACH_OVERFLOW.
    */
    enum ach_status
    ach_put( ach_channel_t *chan, const void *buf, size_t len );


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


    /** Return a string describing the most recent ach error in detail.
     */
    const char *ach_errstr();

    /** Prints information about the channel shm to stderr

        This function is mostly for internal debugging.
    */
    void ach_dump( struct ach_header *shm);

    /** Sets permissions of chan to specified mode */
    enum ach_status
    ach_chmod( ach_channel_t *chan, mode_t mode );

    /** Delete an ach channel */
    enum ach_status
    ach_unlink( const char *name );

    /** Attributes parameter for ach_cancel
     *
     *  Library users should access this struct through the provided
     *  functions rather than directly manipulating it.
     */
    typedef struct ach_cancel_attr {
        union {
            struct {
                /** If true, permit calls that are unsafe in a signal
                 * handler. */
                unsigned int async_unsafe : 1;
            };
            int64_t reserved[8];
        };
    } ach_cancel_attr_t;

    /** Initialize attributes */
    void
    ach_cancel_attr_init( ach_cancel_attr_t *attr );

    /** Set async unsafe field. */
    enum ach_status
    ach_cancel_attr_set_async_unsafe( ach_cancel_attr_t *attr, int asyn_unsafe );

    /** Cancel a pending ach_get() on channel */
    enum ach_status
    ach_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr );

    /** Control structure for event handling loop
     */
    struct ach_evhandler {
        /** Channel to get messages from
         */
        struct ach_channel *channel;

        /** Context argument for handler
         */
        void *context;

        /** Handler function.
         *
         *  Called whenever there is new data in the channel.
         *
         *  Handler should return ACH_OK when new frames are read, or
         *  ACH_STALE_FRAMES if no new frames are read.  Any other
         *  return value will terminate the event loop.
         */
        enum ach_status (*handler)
        ( void *context, struct ach_channel *channel );
    };

    /** Lookup the host providing specified channel via DNS/mDNS SRV
     *  records.
     *
     *
     *
     *  \param[in] channel Name of the channel
     *
     *  \param[in] domain  DNS domain, e.g., "local", "golems.org".
     *                     The domain "local" is treated specially and
     *                     searched via mDNS.  All other domains are
     *                     searched via the convential hierarchical
     *                     DNS.
     *
     * \param[out] host    the name of the host providing the channel.
     *
     * \param[out] hostlen length of the host parameter
     *
     * \param[out] port    the port on host from which channel is
     *                     provided
     */
    enum ach_status
    ach_srv_search( const char *channel, const char *domain,
                    char *host, size_t hostlen,
                    int *port );


/* Options for event handler */

/** Execute the periodic function after the period expires.
 */
#define ACH_EV_O_PERIODIC_TIMEOUT 0x01

/** Execute the periodic function every time new messages are received.
 */
#define ACH_EV_O_PERIODIC_INPUT 0x02


    /** Event loop for handling multiple channels.
    *
    *  \param[in,out]              handlers array of handler descriptors
    *
    *  \param[in] n                size of handlers array
    *
    *  \param[in] period           timeout to wait between execution of periodic
    *                              function when requested in flags.
    *
    *  \param[in] periodic_handler function to executre periodicly,
    *                              i.e., when timeout occurs or when
    *                              new messages are received.
    *
    *  \param[in] periodic_context context argument to the periodic_handler
    *
    * \param[in] options           bit flags, may include
    *                              ACH_EV_O_PERIODIC_INPUT and
    *                              ACH_EV_O_PERIODIC_TIMEOUT
    */
    enum ach_status
    ach_evhandle( struct ach_evhandler *handlers,
                  size_t n,
                  const struct timespec *period,
                  enum ach_status (*periodic_handler)(void *context),
                  void *periodic_context,
                  int options );

#ifdef __cplusplus
}
#endif

#endif /* ACH_H */

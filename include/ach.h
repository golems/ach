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


    typedef enum ach_map {
        ACH_MAP_USER = 0,     /**< Use shared memory for channels */
        ACH_MAP_ANON = 1,     /**< anonymous channel - use heap memory */
        ACH_MAP_KERNEL = 2,   /**< Use kernel memory for channels - require ach kernel module being loaded */
    } ach_map_t;


    /** Type for header in shared memory */
    struct ach_header;

    /** Attributes to pass to ach_open */
    typedef struct {
        union {
            struct{
                union {
                    int map_anon;        /**< anonymous channel (put it in process heap, not shm) */
                    ach_map_t map;       /** < replaces map_anon */
                };
                struct ach_header *shm;   /**< the memory buffer used by anonymous channels */
            };
            uint64_t reserved_size[8]; /**< Reserve space to compatibly add future options */
        };
    } ach_attr_t;

    /** Attributes to pass to ach_create  */
    typedef struct {
        union {
            struct{
                union {
                    int map_anon;  /**< allocate channel in heap, rather than shm */
                    ach_map_t map; /**< replaces map_anon */
                };
                struct ach_header *shm; /**< pointer to channel, set on output of create iff map_anon */
                int truncate;      /**< remove and recreate an existing shm file */
                int set_clock;     /**< if true, set the clock of the condition variable */
                clockid_t clock;   /**< Which clock to use if set_clock is true.
                                    *   The default is defined by ACH_DEFAULT_CLOCK. */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    } ach_create_attr_t;

    /* Struct containing 'cache' of kernel module data to avoid updating when no changes exist */
    typedef struct {
        int options;
        struct timespec reltime;   /**< kernel use relative time */
    } achk_opt_t;

    /** Handle for an Ach channel.
     *
     * Direct access of this structure by library clients is
     * discouraged; its fields may change in future library version.
     */
    typedef struct ach_channel {
        union {
            struct {
                struct ach_header *shm;   /**< pointer to mmap'ed block */
                size_t len;          /**< length of memory mapping */
                int fd;              /**< file descriptor of mmap'ed file */
                union {
                    uint64_t seq_num;    /**< last sequence number read */
                    achk_opt_t k_opts;   /**< Used by kernel devices */
                };
                size_t next_index;   /**< next index entry to try get from */
                ach_attr_t attr;     /**< attributes used to create this channel */
                volatile sig_atomic_t cancel; /**< cancel a waiting ach_get */
            };
            uint64_t reserved[16]; /**< Reserve space to compatibly add future options */
        };
    } ach_channel_t;

    /** Return the file descriptor associated with this channel */
    enum ach_status
    ach_channel_fd( const struct ach_channel *channel, int *file_descriptor );

    /** Return the mapping of the channel. */
    enum ach_status
    ach_channel_mapping( const struct ach_channel *channel, enum ach_map *mapping );

    /** Size of ach_channel_t */
    extern size_t ach_channel_size;
    /** Size of ach_attr_t */
    extern size_t ach_attr_size;

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
        \pre chan has been opened with ach_open()

        \post If buf is big enough to hold the next frame, buf
        contains the data for the last frame and chan.seq_num is set
        to the last frame.  If buf is too small to hold the next
        frame, no side effects occur and ACH_OVERFLOW is returned.
        The seq_num field of chan will be set to the latest sequence
        number (that of the gotten frame).

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

    /** Attributes parameter for ach_cancel */
    typedef struct ach_cancel_attr {
        union {
            struct {
                int async_unsafe; /**< If true, permit calls that are
                                   * unsafe in a signal handler */
            };
            int64_t reserved[8];
        };
    } ach_cancel_attr_t;

    /** Initialize attributes */
    void
    ach_cancel_attr_init( ach_cancel_attr_t *attr );

    /** Cancel a pending ach_get() on channel */
    enum ach_status
    ach_cancel( ach_channel_t *chan, const ach_cancel_attr_t *attr );

    /** Function type to transfer data into the channel.
     *
     * This function could, for example, perform tasks such as serialization.
     *
     * \returns 0 on success, nonzero on failure
     */
    typedef enum ach_status
    ach_put_fun(void *cx, void *chan_dst, const void *obj_src);

    /** Function type to transfer data out of the channel.
     *
     * This function could, for example, perform tasks such as
     * de-serialization and memory allocation.
     *
     * \returns 0 on success, nonzero on failure
     */
    typedef enum ach_status
    ach_get_fun(void *cx, void **obj_dst, const void *chan_src, size_t frame_size );

    /** Writes a new message in the channel.
     *
     *  \pre chan has been opened with ach_open() and is large enough
     *  to hold the message.
     *
     *  Note that transfer() is called while holding the channel lock.
     *  Expensive computation should thus be avoided during this call.
     *
     *  \param [in,out] chan The channel to write to
     *  \param [in] transfer Function to transfer data into the channel
     *  \param [in,out] cx Context argument to transfer
     *  \param [in] obj Source object passed to transfer()
     *  \param [in] dst_size Number of bytes needed in the channel to hold obj
     *
     *  \return ACH_OK on success. If the channel is too small to hold
     *  the frame, returns ACH_OVERFLOW.
    */
    enum ach_status
    ach_xput( ach_channel_t *chan,
              ach_put_fun transfer, void *cx, const void *obj, size_t dst_size );

    /** Pull a message from the channel.
     *
     *  \pre chan has been opened with ach_open()
     *
     *  Note that transfer() is called while holding the channel lock.
     *  Expensive computation should thus be avoided during this call.
     *
     *  \param [in,out] chan The previously opened channel handle
     *  \param [in] transfer Function to transfer data out of the channel
     *  \param [in,out] cx Context argument to transfer
     *  \param [in,out] pobj Pointer to object pointer
     *  \param [out] frame_size The number of bytes occupied by the frame in the channel
     *  \param [in] abstime An absolute timeout if ACH_O_WAIT is specified.
     *  Take care that abstime is given in the correct clock.  The
     *  default is defined by ACH_DEFAULT_CLOCK.
     *  \param[in] options Option flags
     *
     *  \return ACH_OK on success.
     */
    enum ach_status
    ach_xget( ach_channel_t *chan,
              ach_get_fun transfer, void *cx, void **pobj,
              size_t *frame_size,
              const struct timespec *ACH_RESTRICT abstime,
              int options );


    enum ach_status
    ach_srv_search( const char *channel, const char *domain,
                    char *host, size_t host_len,
                    int *port );

#ifdef __cplusplus
}
#endif

#endif /* ACH_H */

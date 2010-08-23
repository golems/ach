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

/** \file ach.h
 *  \author Neil T. Dantam
 *
 *  Moral Support:
 *    Jon Scholz,
 *    Pushkar Kolhe,
 *    Jon Olson,
 *    Venkata Subramania Mahalingam
 */


/** \mainpage
 *
 * \brief Ach is a library that provides a publish-subscribe form of
 * IPC based on POSIX shared memory.
 *
 * Clients may be publishers and or subscribers. Publishers they push
 * data to channels, and subscribers can then poll or wait on the
 * channels for data.
 *
 * Ach deals only with byte arrays.  Any higher level data
 * organization (records, dictionaries, etc) must be handled in the
 * client application.
 *
 * \author Neil T. Dantam, MacOSX porting: Jon Olson
 *
 * \todo better networking
 *
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

#ifdef __cplusplus
/// abstract restrict keyword to play nice with C++
#define ACH_RESTRICT
extern "C" {
#else
/// abstract restrict keyword to play nice with C++
#define ACH_RESTRICT restrict
#endif

    /// maximun size of a channel name
#define ACH_CHAN_NAME_MAX 64

    /// Number of times to retry a syscall on EINTR before giving up
#define ACH_INTR_RETRY 8

    /** magic number that appears the the beginning of our mmaped files.

        This is just to be used as a check.
    */
#define ACH_SHM_MAGIC_NUM 0xb07511f3


    /** A separator between different shm sections.

        This one comes after the header.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_HEADER_NUM ((uint64_t)0x1A2A3A4A5A6A7A8A)
    /** A separator between different shm sections.

        This ones comes after the index array.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_INDEX_NUM ((uint64_t)0x1B2B3B4B5B6B7B8B)

    /** A separator between different shm sections.

        This one comes after the data section (at the very end of the
        file).  Should aid debugging by showing we don't overstep and
        bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_DATA_NUM ((uint64_t)0x1C2C3C4C5C6C7C8C)

    /// return status codes for ach functions
    typedef enum {
        ACH_OK = 0,         ///< Call successful
        ACH_OVERFLOW = 1,       ///< buffer to small to hold frame
        ACH_INVALID_NAME = 2,   ///< invalid channel name
        ACH_BAD_SHM_FILE = 3,   ///< shared memory file didn't look right
        ACH_FAILED_SYSCALL = 4, ///< a system call failed
        ACH_STALE_FRAMES = 5,   ///< no new data in the channel
        ACH_MISSED_FRAME = 6,   ///< we missed the next frame
        ACH_TIMEOUT = 7,        ///< timeout before frame received
        ACH_EEXIST = 8,          ///< shm already exists
        ACH_ENOENT = 9,        ///< shm doesn't
        ACH_CLOSED = 10
    } ach_status_t;

    /// Whether a channel is opened to publish or subscribe
    typedef enum {
        ACH_MODE_PUBLISH,  ///< Channel opened to publish
        ACH_MODE_SUBSCRIBE  ///< Channel opened to subscribe
    } ach_mode_t;

    /// synchronization state of a channel
    typedef enum {
        ACH_CHAN_STATE_INIT,    ///< channel is initializing
        ACH_CHAN_STATE_RUN,     ///< channel is uncontended
        //ACH_CHAN_STATE_READING, ///< readers using channel
        //ACH_CHAN_STATE_WRITING, ///< writer modifying channel
        ACH_CHAN_STATE_CLOSED ///< channel has been closed
    } ach_chan_state_t;

    /** Header for shared memory area.
     */
    typedef struct {
        uint32_t magic;          ///< magic number of ach shm files,
        size_t len;              ///< length of mmap'ed file
        size_t index_cnt;        ///< number of entries in index
        size_t data_size;        ///< size of data bytes
        size_t data_head;        ///< offset to first open byte of data
        size_t data_free;        ///< number of free data bytes
        size_t index_head;       ///< index into index array of first unused index entry
        size_t index_free;       ///< number of unused index entries
        int state;  ///< state of the channel (ie open/closed)
        int refcount; ///< number of open handles to channel
        int anon; ///< is channel in the heap?
        char name[1+ACH_CHAN_NAME_MAX]; ///< Name of this channel
        struct /* anonymous structure */ {
            pthread_mutex_t mutex;     ///< mutex for condition variables
            pthread_cond_t cond; ///< condition variable
            int dirty;
        } sync; ///< variables for synchronization
        // should force our alignment to 8-bytes...
        uint64_t last_seq;       ///< last sequence number written
    } ach_header_t;

    /** Entry in shared memory index array
     */
    typedef struct {
        size_t size;      ///< size of frame
        size_t offset;    ///< byte offset of entry from beginning of data array
        uint64_t seq_num; ///< number of frame
    } ach_index_t ;


    /** Attributes to pass to ach_open */
    typedef struct {
        int map_anon; ///< anonymous channel (put it in process heap, not shm)
        ach_header_t *shm;   ///< the memory buffer used by anonymous channels
    } ach_attr_t;



    /** Attributes to pass to ach_create  */
    typedef struct {
        int truncate; ///< remove and recreate an existing shm file
        int map_anon; ///< allocate channel in heap, rather than shm
        ach_header_t *shm; ///< pointer to channel, set out output of create iff map_anon
    } ach_create_attr_t;


    /** Descriptor for shared memory area
     */
    typedef struct {
        ach_header_t *shm; ///< pointer to mmap'ed block
        size_t len;        ///< length of memory mapping
        int fd;            ///< file descriptor of mmap'ed file
        uint64_t seq_num;  ///<last sequence number read or written
        size_t next_index; ///< next index entry to try to use
        ach_attr_t attr; ///< attributes used to create this channel
        int mode; ///< whether channel was opened for publish or subscribe
    } ach_channel_t;


    /// Gets pointer to guard uint64 following the header
#define ACH_SHM_GUARD_HEADER( shm ) ((uint64_t*)((ach_header_t*)(shm) + 1))

    /// Gets the pointer to the index array in the shm block
#define ACH_SHM_INDEX( shm ) ((ach_index_t*)(ACH_SHM_GUARD_HEADER(shm) + 1))

    /// gets pointer to the guard following the index section
#define ACH_SHM_GUARD_INDEX( shm )                                      \
    ((uint64_t*)(ACH_SHM_INDEX(shm) + ((ach_header_t*)(shm))->index_cnt))

    /// Gets the pointer to the data buffer in the shm block
#define ACH_SHM_DATA( shm ) ( (uint8_t*)(ACH_SHM_GUARD_INDEX(shm) + 1) )

    /// Gets the pointer to the guard following data buffer in the shm block
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
    int ach_create( char *channel_name,
                    size_t frame_cnt, size_t frame_size,
                    ach_create_attr_t *attr);

    /** Opens a handle to channel.
     */
    int ach_open(ach_channel_t *chan, const char *channel_name,
                 ach_attr_t *attr);


    /** Pulls the next message from a channel following the most recent
        message this subscriber has read.

        see ach_get_last for parameter descriptions

        \pre chan has been opened with ach_subscribe()
        \post buf contains the data for the next frame and chan.seq_num is incremented
    */
    int ach_get_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size);

    /** like ach_get_next but blocks if no new data is there */
    int ach_wait_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                      const struct timespec *ACH_RESTRICT abstime);

    /** Pulls the most recent message from the channel.
        \pre chan has been opened with ach_open()

        \post If buf is big enough to hold the next frame, buf contains
        the data for the last frame and chan.seq_num is set to the last
        frame.  If buf is too small to hold the next frame, no side
        effects occur.  The seq_num field of chan will be set to the
        latest sequence number (that of the gotten frame).

        \param chan the channel to read from
        \param buf (output) The buffer to write data to
        \param size the maximum number of bytes that may be written to buf
        \param frame_size (output) The size of the frame.  This will be
        the number of bytes that were written on ACH_SUCCESS or the
        necessary size of the buffer on ACH_OVERFLOW.

        \return ACH_OK on success.  ACH_OVERFLOW if buffer is too small
        to hold the frame.  ACH_STALE_FRAMES if a new frame is not yet
        available.
    */
    int ach_get_last(ach_channel_t *chan, void *buf,
                     size_t size, size_t *frame_size);

    /// always copies the last message, even if it is stale
    int ach_copy_last(ach_channel_t *chan, void *buf,
                      size_t size, size_t *frame_size);


    /** Blocks until a new message is available.

        If the latest message is new than the most recently read
        message, copy that message into buf.  Otherwise, sleep until a
        new message is available.

        See ach_get_next for parameters
    */
    int ach_wait_last( ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                       const struct timespec *ACH_RESTRICT abstime);

    /** Writes a new message in the channel.

        \pre chan has been opened with ach_open()

        \post The contents of buf are copied into the channel and the
        sequence number of the channel is incremented.

        \param chan (action) The channel to write to
        \param buf The buffer containing the data to copy into the channel
        \param len number of bytes in buf to copy
        \return ACH_OK on success.
    */
    int ach_put(ach_channel_t *chan, void *buf, size_t len);


    /** Discards all previously received messages for this handle.  Does
        not change the actual channel, just resets the sequence number in
        the handle.*/
    int ach_flush( ach_channel_t *chan );

    /** Closes the shared memory block.

        \pre chan is an initialized ach channel with open shared memory area

        \post the shared memory file for chan is closed
    */
    int ach_close(ach_channel_t *chan);


    /** Converts return code from ach call to a human readable string;
     */
    const char *ach_result_to_string(ach_status_t result);


    /** Prints information about the channel shm to stderr

        This function is mostly for internal debugging.
    */
    void ach_dump( ach_header_t *shm);

    /// size of stream prefix (guard words and size word)
#define ACH_STREAM_PREFIX_SIZE  12

    /** Writes message pointed to by but to stream fd */
    int ach_stream_write_msg( int fd, const char *buf, size_t cnt);

    /** Reads the size of a message from fd and stores in cnt. */
    int ach_stream_read_msg_size( int fd, int *cnt);

    /** Reads msg_size bytes of data from fd and stores in buf. */
    int ach_stream_read_msg_data( int fd, char *buf, size_t msg_size, size_t buf_size);


    /** Reads from fd into buffer completely */
    int ach_stream_read_fill( int fd, char *buf, size_t cnt );

    /** Writes from buffer into fd completely */
    int ach_stream_write_fill( int fd, const char *buf, size_t cnt );

    /** Reads a line from fd */
    int ach_read_line( int fd, char *buf, size_t cnt );

    /** Sets permissions of chan to specified mode */
    int ach_chmod( ach_channel_t *chan, mode_t mode );

#ifdef __cplusplus
}
#endif

#endif // ACH_H

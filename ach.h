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
 *  Moral Support
 *    Jon Scholz
 *    Pushkar Kolhe
 *    Jon Olson
 *    Venkata Subramania Mahalingam
 */


/** \mainpage
 *
 * ach is a library (and eventually daemon) that provides a
 * publish-subscribe form of IPC. Clients may be publishers and or
 * subscribers. Publishers create "channels" which they push data
 * to. Clients can then poll then channels for data.
 *
 * \todo Write network daemon
 * \todo test variable size frames
 * \todo write blocking method calls
 *
 * \bug could do better error checking on existence or non-existence
 * of the shared memory file.
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

#ifdef __cplusplus
extern "C" {
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
        ACH_OVERFLOW,       ///< buffer to small to hold frame
        ACH_INVALID_NAME,   ///< invalid channel name
        ACH_BAD_SHM_FILE,   ///< shared memory file didn't look right
        ACH_FAILED_SYSCALL, ///< a system call failed
        ACH_STALE_FRAMES,          ///< no new data in the channel
        ACH_MISSED_FRAME,   ///< we missed the next frame
        ACH_TIMEOUT         ///< timeout before frame received
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
        ACH_CHAN_STATE_READING, ///< readers using channel
        ACH_CHAN_STATE_WRITING, ///< writer modifying channel
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
        //pthread_rwlock_t rwlock; ///< the lock
        struct /* anonymous structure */ {
            int state;  ///< synchronization state of the channel, type ach_chan_state_t
            unsigned int reader_active_cnt; ///< number of readers currently reading
            /** number of readers waiting to read. includes both blocked by
                lock and blocking while waiting for new data */
            unsigned int reader_wait_cnt;
            unsigned int writer_wait_cnt; ///< number of writers waiting to write
            pthread_mutex_t mutex;     ///< mutex for condition variables
            pthread_cond_t write_cond; ///< condition variable writers wait on
            pthread_cond_t read_cond;  ///< condition variable readers wait on
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


    /** Descriptor for shared memory area
     */
    typedef struct {
        ach_header_t *shm; ///< pointer to mmap'ed block
        size_t len;        ///< length of memory mapping
        int fd;            ///< file descriptor of mmap'ed file
        uint64_t seq_num;  ///<last sequence number read or written
        size_t next_index; ///< next index entry to try to use
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


    /** Establishes a new channel.

        \post A shared memory area is created for the channel and chan is initialized for writing

        The size of the data array reserved in shared memory will be
        frame_cnt*frame_size.  Frames of any size smaller than the size
        of the data array may be sent over the channel.

        \param chan The channel structure to initialize
        \param channel_name Name of the channel
        \param frame_cnt number of frames to hold in circular buffer
        \param frame_size nominal size of each frame

        \return ACH_OK on success.  ACH_INVALID_NAME is the channel name
        is invalid.  ACH_SYSCALL if a syscall fails.  In that case,
        errno ma or may not be set.
    */
    int ach_publish(ach_channel_t *chan, char *channel_name,
                    size_t frame_cnt, size_t frame_size);

    /** Subscribes to a channel.

        \pre The channel has been published

        \post chan is initialized for reading

        \return ACH_OK on success.  ACH_INVALID_NAME is the channel name
        is invalid.  ACH_SYSCALL if a syscall fails.  ACH_BAD_SHM_FILE
        if the shared memory file is broken or otherwise invalid.  In
        that case, errno may or may not be set.
    */
    int ach_subscribe(ach_channel_t *chan, char *channel_name);


    /** Pulls the next message from a channel following the most recent
        message this subscriber has read.

        see ach_get_last for parameter descriptions

        \pre chan has been opened with ach_subscribe()
        \post buf contains the data for the next frame and chan.seq_num is incremented
    */
    int ach_get_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size);

    /** like ach_get_next but blocks if no new data is there */
    int ach_wait_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size,
                      const struct timespec *abstime);

    /** Pulls the most recent message from the channel.
        \pre chan has been opened with ach_subscribe()

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
    int ach_get_last(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size);

    /** Blocks until a new message is available.

        If the latest message is new than the most recently read
        message, copy that message into buf.  Otherwise, sleep until a
        new message is available.

        See ach_get_next for parameters
    */
    int ach_wait_last( ach_channel_t *chan, void *buf, size_t size, size_t *size_written, const struct timespec *restrict abstime);

    /** Writes a new message in the channel.

        \pre chan has been opened with ach_publish()

        \post The contents of buf are copied into the channel and the
        sequence number of the channel is incremented.

        \param chan (action) The channel to write to
        \param buf The buffer containing the data to copy into the channel
        \param len number of bytes in buf to copy
        \return ACH_OK on success.
    */
    int ach_put(ach_channel_t *chan, void *buf, size_t len);



    /** Closes the shared memory block.

        \pre chan is an initialized ach channel with open shared memory area

        \post the shared memory file for chan is closed
    */
    int ach_close(ach_channel_t *chan);


    /** Converts return code from ach call to a human readable string;
     */
    char *ach_result_to_string(ach_status_t result);


    /** Prints information about the channel shm to stderr

        This function is mostly for internal debugging.
    */
    void ach_dump( ach_header_t *shm);


#ifdef __cplusplus
}
#endif

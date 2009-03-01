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
 *  \author Jon Scholz
 */




/*
 * Shared Memory Layout:
 *
 *    ________
 *   | Header |
 *   |--------|
 *   | Index  |
 *   |        |
 *   |--------|
 *   |  Data  |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |________|
 */

#ifdef __cplusplus
//extern "C" {
#endif

typedef enum {
    ACH_OK = 0,
    ACH_OVERFLOW
} ach_status_t;

typedef struct {
    pthread_rwlock_t rwlock;
    // should force our alignment to 8-bytes...
    size_t index_count;     //< number of entries in index
    size_t last_index;      //< offset from index start to last entry written
} ach_header_t;

typedef struct {
    uint64_t seq_num; //< number of frame
    size_t size;      //< size of frame
    size_t offset;    //< byte offset of entry from beginning of data array
} ach_index_t ;


typedef struct {
    uint64_t seq_num;  //<last sequence number read or written
    void *shm_ptr;
} ach_channel_t;


// general idea here...
#define ACH_SHM_INDEX( shm_ptr ) ( (shm_ptr) + sizeof(shm_header) )
#define ACH_SHM_DATA( shm_ptr ) ( (shm_ptr) + sizeof(shm_header) + (shm_header*)(shm_ptr).index_size)


/** Establishes a new channel.
    \post A shared memory area is created for the channel and chan is initialized for writing
*/
ach_status_t ach_publish(ach_channel_t *chan, char *channel_name, int freq_hz);

/** Subscribes to a channel.
    \pre The channel has been published
    \post chan is initialized for reading
*/
ach_status_t ach_subscribe(ach_channel_t *chan, char *channel_name, int freq_hz);

/** Pulls the next message from a channel.
    \pre chan has been opened with ach_subscribe()
    \post buf contains the data for the next frame and chan.seq_num is incremented
*/
ach_status_t ach_get_next(ach_channel_t *chan, char *buf, size_t size, size_t *size_written);

/** Pulls the most recent message from the channel.
    \pre chan has been opened with ach_subscribe()
    \post buf contains the data for the last frame and chan.seq_num is set to the last frame
*/
ach_status_t ach_get_last(ach_channel_t *chan, char *buf, size_t size, size_t *size_written);

/** Writes a new message in the channel.
    \pre chan has been opened with ach_publish()
    \post The contents of buf are copied into the channel and chan.seq_num is incremented.
*/
ach_status_t ach_put(ach_channel_t *chan, char *buf, size_t len);



#ifdef __cplusplus
//}
#endif

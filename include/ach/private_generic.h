/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2014, Georgia Tech Research Corporation
 * Copyright (C) 2014, Prevas A/S
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *            Kim Boendergaard Poulsen <kibo@prevas.dk>
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
 *   * Neither the name of the copyright holder the names of its
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

/** \file ach_private_generic.h
 *  \author Neil T. Dantam, Kim Boendergaard Poulsen
 */

#ifndef ACH_PRIVATE_GENERIC_H
#define ACH_PRIVATE_GENERIC_H

#ifdef __cplusplus
extern "C" {
#endif

/** prefix to apply to channel names got get the chardev file name */
#define ACH_CHAR_CHAN_NAME_PREFIX_PATH "/dev/"
#define ACH_CHAR_CHAN_NAME_PREFIX_NAME "ach-"

/** Name of chardev channel controlling device */
#define ACH_CHAR_CHAN_CTRL_NAME "/dev/achctrl"

/** magic number that appears the the beginning of our mmaped files.
 *
 *  This is just to be used as a check.
 */
#define ACH_SHM_MAGIC_NUM 0xb07511f3

/** A separator between different shm sections.
 *
 *  This one comes after the header.  Should aid debugging by
 *  showing we don't overstep and bounds.  64-bit for alignment.
 */
#define ACH_SHM_GUARD_HEADER_NUM ((uint64_t)0x1A2A3A4A5A6A7A8ALLU)

/** A separator between different shm sections.
 *
 *  This ones comes after the index array.  Should aid debugging by
 *  showing we don't overstep and bounds.  64-bit for alignment.
 */
#define ACH_SHM_GUARD_INDEX_NUM ((uint64_t)0x1B2B3B4B5B6B7B8BLLU)

/** A separator between different shm sections.
 *
 *  This one comes after the data section (at the very end of the
 *  file).  Should aid debugging by showing we don't overstep and
 *  bounds.  64-bit for alignment.
 */
#define ACH_SHM_GUARD_DATA_NUM ((uint64_t)0x1C2C3C4C5C6C7C8CLLU)

/** Header for shared memory area.
 *
 * There is no tail pointer here.  Every subscriber that opens the
 * channel must maintain its own tail pointer.
 */
typedef struct ach_header {
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
            clockid_t clock;         /**< clock used for this channel */
#ifdef ACH_POSIX
            int anon;                /**< is channel in the heap? */
#endif
        };
        uint64_t reserved[16];  /**< Reserve to compatibly add future variables */
    };
    struct /* anonymous structure */ {
#ifdef ACH_POSIX
        pthread_mutex_t mutex;         /**< mutex for condition variables */
        pthread_cond_t cond;           /**< condition variable */
#endif
#ifdef ACH_KLINUX
        struct rt_mutex mutex;
        wait_queue_head_t readq;
#endif
        int dirty;
    } sync;                   /**< variables for synchronization */
#ifdef ACH_KLINUX
    struct kref refcount;
    struct rt_mutex ref_mutex;
    unsigned rd_cnt;
#endif
    /* should force our alignment to 8-bytes... */
    uint64_t last_seq;        /**< last sequence number written */
} ach_header_t;

/** Entry in shared memory index array
 */
typedef struct ach_index {
    size_t size;      /**< size of frame */
    size_t offset;    /**< byte offset of entry from beginning of data array */
    uint64_t seq_num; /**< number of frame */
} ach_index_t;

#ifdef __cplusplus
}
#endif

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

/** Default number of index entries in a channel */
#define ACH_DEFAULT_FRAME_COUNT 16

/** Default nominal frame size for a channel */
#define ACH_DEFAULT_FRAME_SIZE 512

#endif

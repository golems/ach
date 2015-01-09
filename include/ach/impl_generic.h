/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
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

/** \file impl_generic.h
 *
 *  \brief This file contains definitions used by both the POSIX and
 *         Linux kernel implementations.
 *
 *  \author Neil T. Dantam
 */

#ifndef ACH_IMPL_H
#define ACH_IMPL_H



static size_t oldest_index_i( ach_header_t *shm ) {
    return (shm->index_head + shm->index_free)%shm->index_cnt;
}

static size_t last_index_i( ach_header_t *shm ) {
    return (shm->index_head + shm->index_cnt -1)%shm->index_cnt;
}

static enum ach_status
rdlock(ach_channel_t * chan, int wait, const struct timespec *time);

static enum ach_status unrdlock(struct ach_header *shm);

static enum ach_status
ach_flush_impl( ach_channel_t *chan )
{
    ach_header_t *shm = chan->shm;
    enum ach_status r = rdlock(chan, 0,  NULL);
    if( ACH_OK != r ) return r;

    chan->seq_num = shm->last_seq;
    chan->next_index = shm->index_head;
    return unrdlock(shm);
}

static enum ach_status
check_guards( ach_header_t *shm )
{
    if( ACH_SHM_MAGIC_NUM != shm->magic ||
        ACH_SHM_GUARD_HEADER_NUM != *ACH_SHM_GUARD_HEADER(shm) ||
        ACH_SHM_GUARD_INDEX_NUM != *ACH_SHM_GUARD_INDEX(shm) ||
        ACH_SHM_GUARD_DATA_NUM != *ACH_SHM_GUARD_DATA(shm)  )
    {
        return ACH_CORRUPT;
    } else {
        return ACH_OK;
    }
}

static void free_index(ach_header_t *shm, size_t i )
{
    ach_index_t *index_ar = ACH_SHM_INDEX(shm);

#ifdef ACH_POSIX
    assert( index_ar[i].seq_num ); /* only free used indices */
    assert( index_ar[i].size );    /* must have some data */
    assert( shm->index_free < shm->index_cnt ); /* must be some used index */
#endif

    shm->data_free += index_ar[i].size;
    shm->index_free ++;
    memset( &index_ar[i], 0, sizeof( ach_index_t ) );
}


static int ach_create_len( size_t frame_cnt, size_t frame_size )
{
    return sizeof(struct ach_header) +
        frame_cnt * sizeof(ach_index_t) +
        frame_cnt * frame_size + 3 * sizeof(uint64_t);
}

static void ach_create_counts( ach_header_t *shm, size_t frame_cnt, size_t frame_size )
{
    /* initialize counts */
    shm->index_cnt = frame_cnt;
    shm->index_head = 0;
    shm->index_free = frame_cnt;
    shm->data_head = 0;
    shm->data_free = frame_cnt * frame_size;
    shm->data_size = frame_cnt * frame_size;

    *ACH_SHM_GUARD_HEADER(shm) = ACH_SHM_GUARD_HEADER_NUM;
    *ACH_SHM_GUARD_INDEX(shm) = ACH_SHM_GUARD_INDEX_NUM;
    *ACH_SHM_GUARD_DATA(shm) = ACH_SHM_GUARD_DATA_NUM;
    shm->magic = ACH_SHM_MAGIC_NUM;
}


#endif /* ACH_IMPL_H */

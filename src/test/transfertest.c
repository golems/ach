/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/wait.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include "ach.h"

#define OPT_CHAN  "ach-test-transfer"
ach_channel_t channel;

static void test(ach_status_t r, const char *thing) {
    if( r != ACH_OK ) {
        fprintf(stderr, "%s: %s\n",
                thing, ach_result_to_string(r));
        exit(-1);
    }
}

/* --- MEMCPY TEST ---*/
enum ach_status
ach_put_fun_memcpy(void *cx, void *chan_dst, const void *obj)
{
    memcpy( chan_dst, obj, *(size_t*)cx );
    return ACH_OK;
}

enum ach_status
ach_get_fun_memcpy(void *cx, void **obj_dst, const void *chan_src, size_t frame_size )
{
    size_t max_size = *(size_t*)cx;
    if( max_size < frame_size )
        return ACH_OVERFLOW;
    memcpy( *obj_dst, chan_src, frame_size );
    return ACH_OK;
}

static void check_cpy() {
    /* Generate Data */
    enum ach_status r;
    char data[]="foobar123";
    char buf[64] = {0};

    /* Put Data */
    size_t n = strlen(data)+1;
    r = ach_xput( &channel, ach_put_fun_memcpy, &n, data, n );
    test(r, "memcpy put");

    /* Get Data */
    size_t max_n = sizeof(buf);
    size_t frame_size;
    void *pbuf = buf;
    r = ach_xget( &channel,
                  &ach_get_fun_memcpy, &max_n, &pbuf,
                  &frame_size, NULL, 0 );
    test(r, "memcpy get");

    /* Check Data */
    if( strcmp(data, buf) ) exit(-1);
}

/* --- MALLOC TEST ---*/

enum ach_status
ach_get_fun_malloc(void *cx, void **obj_dst, const void *chan_src, size_t frame_size )
{
    (void)cx;
    *obj_dst = malloc(frame_size);
    memcpy( *obj_dst, chan_src, frame_size );
    return ACH_OK;
}

static void check_malloc() {
    /* Generate Data */
    enum ach_status r;
    char data[]="malloc321";

    /* Put Data */
    size_t n = strlen(data)+1;
    r = ach_xput( &channel, ach_put_fun_memcpy, &n, data, n );
    test(r, "malloc put");

    /* Get Data */
    size_t frame_size;
    void *pbuf;
    r = ach_xget( &channel,
                  &ach_get_fun_malloc, NULL, &pbuf,
                  &frame_size, NULL, 0 );
    test(r, "malloc get");

    /* Check Data */
    if( strcmp(data, pbuf) ) exit(-1);
    free(pbuf);
}

/* --- Main ---*/

int main( int argc, char **argv ){

    /* unlink */
    ach_status_t r = ach_unlink(OPT_CHAN);
    if( ! (ACH_OK==r || ACH_ENOENT == r) ) {
        fprintf(stderr, "ach_unlink failed\n: %s",
                ach_result_to_string(r));
        return -1;
    }

    /* create */
    r = ach_create(OPT_CHAN, 32ul, 64ul, NULL );
    test(r, "ach_create");

    /* open */
    r = ach_open(&channel, OPT_CHAN, NULL);
    test(r, "ach_open");

    /* unlink */
    r = ach_unlink(OPT_CHAN);
    test(r, "ach_unlink");


    check_cpy();
    check_malloc();

    return 0;
}

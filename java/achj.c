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

#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include "ach.h"
#include "org_golems_ach_Lib.h"

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_open (JNIEnv *env, jclass cls, jstring name, jlongArray arr) {
    (void)cls;
    /* open */
    ach_channel_t *chan = (ach_channel_t*)malloc(sizeof(ach_channel_t));
    const char *native_name = (*env)->GetStringUTFChars(env, name, 0);
    int r = ach_open( chan, native_name, NULL );
    (*env)->ReleaseStringUTFChars(env, name, native_name);

    /* store */
    if( ACH_OK == r ) {
        jlong *ptr = (*env)->GetLongArrayElements(env,arr,0);
        ptr[0] = (intptr_t)chan;
        (*env)->ReleaseLongArrayElements(env,arr,ptr,0);
    } else {
        free(chan);
    }

    return r;
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_close
(JNIEnv *env, jclass clazz, jlong ptr) {
    (void)env; (void)clazz;
    ach_channel_t *chan = (ach_channel_t*)ptr;
    int r = ach_close( chan );
    free( chan );
    return r;
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_flush
(JNIEnv *env, jclass cls, jlong ptr) {
    (void)env; (void)cls;
    return ach_flush( (ach_channel_t*) ptr );
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_put
(JNIEnv *env, jclass clazz, jlong ptr, jbyteArray arr) {
    (void)clazz;
    jsize len = (*env)->GetArrayLength(env,arr);
    jbyte *buf = (*env)->GetByteArrayElements(env,arr,0);
    int r = ach_put( (ach_channel_t*)ptr, buf, (size_t)len );
    (*env)->ReleaseByteArrayElements(env,arr,buf,0);
    return r;
}

static jint s_get( JNIEnv *env, jlong ptr, jbyteArray arr, jlongArray fs_ar, struct timespec *ts, jint opt ) {
    /* get data */
    jsize len = (*env)->GetArrayLength(env,arr);
    jbyte *buf = (*env)->GetByteArrayElements(env,arr,0);
    size_t fs = 0;
    int r = ach_get( (ach_channel_t*)ptr, buf, (size_t)len,
                     &fs, ts, opt );
    (*env)->ReleaseByteArrayElements(env,arr,buf,0);

    /* store frame size */
    if( NULL != fs_ar &&
        0 < (*env)->GetArrayLength(env,fs_ar) )
    {
        jlong *fsbuf = (*env)->GetLongArrayElements(env,fs_ar,0);
        fsbuf[0] = (jlong)fs;
        (*env)->ReleaseLongArrayElements(env,fs_ar,fsbuf,0);
    }

    return r;
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_get__J_3B_3JI
(JNIEnv *env, jclass cls,
 jlong ptr, jbyteArray arr, jlongArray fs,
 jint opt)
{
    (void)cls;
    return s_get( env, ptr, arr, fs, NULL, opt );
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_get__J_3B_3JJII
(JNIEnv *env, jclass cls,
 jlong ptr, jbyteArray buf, jlongArray fs,
 jlong sec, jint nsec, jint opt)
{
    (void) cls;
    struct timespec ts;
    ts.tv_sec = sec;
    ts.tv_nsec = nsec;
    return s_get( env, ptr, buf, fs, &ts, opt );
}

JNIEXPORT jint JNICALL
Java_org_golems_ach_Lib_time
(JNIEnv *env, jclass cls, jlongArray arr)
{
    (void)env; (void)cls;
    jsize len = (*env)->GetArrayLength(env,arr);
    if( len ) {
        struct timespec ts;
        int r = clock_gettime( ACH_DEFAULT_CLOCK, &ts );
        jlong *ptr = (*env)->GetLongArrayElements(env,arr,0);
        ptr[0] = ts.tv_sec;
        if( 2 <= len ) {
            ptr[1] = ts.tv_nsec;
        }
        (*env)->ReleaseLongArrayElements(env,arr,ptr,0);
        return r;
    } else {
        return -2;
    }
}

/* Define java functions for enum ach_status */

#define DEF_CONST(NAME)                         \
    JNIEXPORT jint JNICALL                      \
    Java_org_golems_ach_Lib_ACH_1##NAME         \
    (JNIEnv *env, jclass cls) {                 \
        (void) env; (void)cls;                  \
        return ACH_##NAME;                      \
    }

DEF_CONST(OK);
DEF_CONST(OVERFLOW);
DEF_CONST(INVALID_NAME);
DEF_CONST(BAD_SHM_FILE);
DEF_CONST(FAILED_SYSCALL);
DEF_CONST(STALE_FRAMES);
DEF_CONST(MISSED_FRAME);
DEF_CONST(TIMEOUT);
DEF_CONST(EEXIST);
DEF_CONST(ENOENT);
DEF_CONST(CLOSED);
DEF_CONST(BUG);
DEF_CONST(EINVAL);
DEF_CONST(CORRUPT);
DEF_CONST(BAD_HEADER);
DEF_CONST(EACCES);
DEF_CONST(CANCELED);

/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
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

#ifndef IPCBENCH_H
#define IPCBENCH_H

#include <stdio.h>
#include <sched.h>
#include <inttypes.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <assert.h>


struct ipcbench_vtab {

    /** Init global structures */
    void (*init)(void);

    /** Initialize sending process data */
    void (*init_send)(void);
    /** Initialize receiving process data */
    void (*init_recv)(void);

    /** Send a timespec */
    void (*send)(const struct timespec *ts);
    /** Receive a timespec */
    void (*recv)(struct timespec *ts );

    /** Destroy sending process data */
    void (*destroy_send)(void);
    /** Destroy receiving process data */
    void (*destroy_recv)(void);


    /** Destroy global structures */
    void (*destroy)(void);
};

extern struct timespec ipcbench_period;

extern struct ipcbench_vtab ipc_bench_vtab_ach;
extern struct ipcbench_vtab ipc_bench_vtab_lcm;
extern struct ipcbench_vtab ipc_bench_vtab_pipe;
extern struct ipcbench_vtab ipc_bench_vtab_mq;
extern struct ipcbench_vtab ipc_bench_vtab_tcp;
extern struct ipcbench_vtab ipc_bench_vtab_local;
extern struct ipcbench_vtab ipc_bench_vtab_udp;
extern struct ipcbench_vtab ipc_bench_vtab_udp_multicast;
extern struct ipcbench_vtab ipc_bench_vtab_local_dgram;
extern struct ipcbench_vtab ipc_bench_vtab_corba;

#endif // IPCBENCH_H

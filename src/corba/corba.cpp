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

#include "ipcbench.h"
#include "ipcbenchC.h"
#include "ace/streams.h"
#include <stdio.h>
#include <time.h>

#define IOR_THING "file:///tmp/ipcbench.ior"

CORBA::ORB_var orb;
CORBA::Object_var factory_object;
ipcbench::Thingy_Factory_var factory;
ipcbench::Thingy_var thing;

static void s_init_recv() {
    static int k = 1;
    static const char *v [] = {"ipcbench", NULL};

    orb = CORBA::ORB_init (k, (char**)v, "my_orb" );
    factory_object = orb->string_to_object (IOR_THING);
    factory = ipcbench::Thingy_Factory::_narrow (factory_object.in ());
    thing = factory->get_thingy ( );
}


//static void s_nop(void) { }

static void s_nop_ts(const struct timespec *ts) { (void)ts; }

static void s_recv( struct timespec *ts ) {
    /* Kludge: do the wait here, instead of the publish */
    clock_nanosleep( CLOCK_MONOTONIC, 0, &ipcbench_period, NULL ); // TODO, handle eintr

    ipcbench::corba_timespec cts = thing->getit();
    ts->tv_sec = cts.sec;
    ts->tv_nsec = cts.nsec;
}

static void s_destroy_recv(void) {
    orb->destroy();
}

struct ipcbench_vtab ipc_bench_vtab_corba = {
    NULL, // init
    NULL, // init_send
    s_init_recv, // init_recv
    s_nop_ts, // send
    s_recv, // recv
    NULL, // destroy_send
    s_destroy_recv, // destroy_recv
    NULL // destroy
};

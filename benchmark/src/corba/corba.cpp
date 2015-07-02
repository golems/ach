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
#include "util.h"
#include <stdio.h>
#include <time.h>
#include "ipcbenchS.h"

class Thingy_i : public POA_ipcbench::Thingy {
public:
   virtual ipcbench::corba_timespec getit ( void );
   virtual void set_timespec ( CORBA::LongLong sec, CORBA::Long nsec );
private:
};

void
Thingy_i::set_timespec ( CORBA::LongLong sec, CORBA::Long nsec )
{
    printf("set_timespec: %ld, %d \n", sec, nsec);
}

ipcbench::corba_timespec
Thingy_i::getit ()
{
    struct timespec ts = get_ticks();

    ipcbench::corba_timespec cts;
    cts.sec = ts.tv_sec;
    cts.nsec = ts.tv_nsec;
    return cts;
}


class Thingy_Factory_i : public POA_ipcbench::Thingy_Factory {
public:
    ipcbench::Thingy_ptr get_thingy ();
private:
    Thingy_i obj;
};


ipcbench::Thingy_ptr
Thingy_Factory_i::get_thingy ()
{
    printf("getit\n");
    return this->obj._this();
}

static int ior_pipe[2];

static void s_init() {
    if( pipe(ior_pipe) ) {
        perror( "could not create IOR pipe" );
        abort();
    }
}

static CORBA::ORB_var orb;
static CORBA::Object_var factory_object;
static ipcbench::Thingy_Factory_var factory;
static ipcbench::Thingy_var thing;
static void s_init_recv() {
    printf("init_recv()\n");
    static int k = 1;
    static const char *v [] = {"ipcbench", NULL};

    orb = CORBA::ORB_init (k, (char**)v, "my_orb" );

    // get ior
    char *buf = NULL;
    {
        FILE *fp = fdopen(ior_pipe[0], "r");
        size_t n;
        ssize_t r = getline( &buf, &n, fp );
        if( r < 0 ) {
            perror( "could not read IOR data on pipe" );
            abort();
        }
        fclose(fp);
    }
    buf[strlen(buf)-1] = '\0'; // null newline
    printf("foo: %s\n", buf);
    usleep(10e3);

    factory_object = orb->string_to_object (buf);
    printf("got factory\n");
    factory = ipcbench::Thingy_Factory::_narrow (factory_object.in ());
    printf("factory narrowed\n");
    thing = factory->get_thingy ( );
    printf("got thingy\n");
    free(buf);
    printf("recv init\n");
}

static CORBA::Object_var poa_object;
static PortableServer::POA_var poa;
static PortableServer::POAManager_var poa_manager;
static Thingy_Factory_i factory_i;
static void s_init_send() {
    // First initialize the ORB, that will remove some arguments...
    int argc=1;
    const char *argv[] = {"foo"};
    orb = CORBA::ORB_init (argc, (char**)argv, "my_orb" );

    poa_object = orb->resolve_initial_references ("RootPOA");
    poa = PortableServer::POA::_narrow (poa_object.in ());

    poa_manager = poa->the_POAManager ();

    poa_manager->activate ();

    // Activate it to obtain the object reference
    factory = factory_i._this ();

    // Put the object reference as an IOR string
    CORBA::String_var ior = orb->object_to_string (factory.in ());

    {
        const char *ior_str = ior;
        char buf[strlen(ior_str)+2];
        strcpy(buf, ior_str);
        strcat(buf, "\n");
        ssize_t r = write( ior_pipe[1], buf, strlen(buf) );

        if( strlen(buf) != r ) {
            perror( "could not send data on pipe" );
            abort();
        }
    }
}

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

static void s_destroy_send(void) {
    poa->destroy (1, 1);
    orb->destroy ();
}

static void s_send_loop(void) {
    s_init_send();
    orb->run ();
    s_destroy_send();
}

struct ipcbench_vtab ipc_bench_vtab_corba = {
    s_init, // init
    NULL, // init_send
    s_init_recv, // init_recv
    s_nop_ts, // send
    s_recv, // recv
    s_destroy_send, // destroy_send
    s_destroy_recv, // destroy_recv
    NULL, // destroy
    s_send_loop, //send_loop
    NULL // recv_loop
};

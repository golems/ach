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
#include "ace/streams.h"
#include "ipcbenchC.h"
#include "ipcbenchS.h"
#include <orbsvcs/orbsvcs/CosEventCommS.h>
#include <orbsvcs/orbsvcs/CosEventChannelAdminC.h>
#include "ace/streams.h"
#include "ace/OS_NS_unistd.h"
#include "orbsvcs/CosNamingC.h"

#include <stdio.h>
#include <time.h>

#include "util.h"

#include <mqueue.h>
mqd_t mq;

/* Thingy */

class CosThingy_i
  : public virtual POA_ipcbench::Thingy
{
public:
    CosThingy_i();
    virtual ipcbench::corba_timespec getit ( void );
    virtual void set_timespec ( CORBA::LongLong sec, CORBA::Long nsec );

    void disconnect_push_supplier (void);
    void connect (CosEventChannelAdmin::SupplierAdmin_ptr supplier_admin);

private:
    ipcbench::corba_timespec data;

    CosEventChannelAdmin::ProxyPushConsumer_var consumer_proxy_;

    POA_CosEventComm::PushSupplier_tie<CosThingy_i> supplier_personality_;
};

CosThingy_i::CosThingy_i()
    :  supplier_personality_ (this)
{}

ipcbench::corba_timespec
CosThingy_i::getit ()
{
    return data;
}

void
CosThingy_i::set_timespec ( CORBA::LongLong sec, CORBA::Long nsec )
{
    data.sec = sec;
    data.nsec = nsec;

    if (CORBA::is_nil (this->consumer_proxy_.in ()))
        return;

    CORBA::Any event;
    event <<= this->data;
    this->consumer_proxy_->push (event);
}

void
CosThingy_i::disconnect_push_supplier (void)
{
  // Forget about the consumer it is not there anymore
  this->consumer_proxy_ =
    CosEventChannelAdmin::ProxyPushConsumer::_nil ();
}

void
CosThingy_i::connect (CosEventChannelAdmin::SupplierAdmin_ptr supplier_admin)
{
  this->consumer_proxy_ =
    supplier_admin->obtain_push_consumer ();
  CosEventComm::PushSupplier_var supplier =
    this->supplier_personality_._this ();
  this->consumer_proxy_->connect_push_supplier (supplier.in ());
}


/* Thingy Factory */

class CosThingy_Factory_i : public POA_ipcbench::Thingy_Factory {
public:
    CosThingy_Factory_i ();
    ipcbench::Thingy_ptr get_thingy ();
    PortableServer::POA_var factory_poa_;
    CosThingy_i obj;
    void load( PortableServer::POA_ptr poa,
               PortableServer::POAManager_ptr poa_manager,
               CosEventChannelAdmin::SupplierAdmin_ptr supplier_admin);
};

CosThingy_Factory_i::CosThingy_Factory_i () {}

ipcbench::Thingy_ptr
CosThingy_Factory_i::get_thingy ( ) {
    return this->obj._this();
}
void
CosThingy_Factory_i::load( PortableServer::POA_ptr poa,
                        PortableServer::POAManager_ptr poa_manager,
                        CosEventChannelAdmin::SupplierAdmin_ptr supplier_admin) {

    if (!CORBA::is_nil (this->factory_poa_.in ()))
        return;

    CORBA::PolicyList policies (2);
    policies.length (2);

    policies[0] =
        poa->create_id_assignment_policy (PortableServer::USER_ID);
    policies[1] =
        poa->create_implicit_activation_policy (PortableServer::NO_IMPLICIT_ACTIVATION);

    this->factory_poa_ =
        poa->create_POA ("Thingy_Factory_POA",
                         poa_manager,
                         policies);

    for (CORBA::ULong i = 0; i != policies.length (); ++i) {
        policies[i]->destroy ();
    }

    PortableServer::ServantBase_var servant = & (this->obj);
    PortableServer::ObjectId_var oid = PortableServer::string_to_ObjectId ("my_time_symbol");

    this->factory_poa_->activate_object_with_id (oid.in (), servant.in ());
    this->obj.connect (supplier_admin);
}

/* Consumer */

class Thingy_Consumer : public POA_CosEventComm::PushConsumer {
public:
    Thingy_Consumer ();
    void connect (CosEventChannelAdmin::EventChannel_ptr event_channel);
    void push (const CORBA::Any& data);
    void disconnect ();
    void disconnect_push_consumer (void);
private:
    CosEventChannelAdmin::ProxyPushSupplier_var supplier_proxy_;
};

Thingy_Consumer::Thingy_Consumer () { }

void
Thingy_Consumer::connect (CosEventChannelAdmin::EventChannel_ptr event_channel)
{
  CosEventChannelAdmin::ConsumerAdmin_var consumer_admin =
    event_channel->for_consumers ();

  this->supplier_proxy_ =
    consumer_admin->obtain_push_supplier ();

  CosEventComm::PushConsumer_var myself = this->_this ();
  this->supplier_proxy_->connect_push_consumer (myself.in ());
}

void
Thingy_Consumer::push (const CORBA::Any& data)
{
    struct timespec now = get_ticks();
    ipcbench::corba_timespec *event;
    if ((data >>= event) == 0)
        return; // Invalid event

    struct timespec ts = {event->sec, event->nsec };

    double us = ticks_delta( ts, now ) * 1e6;
    ssize_t r = mq_send(mq, (char*)&us, sizeof(us), 0);
    if( sizeof(us) == r )  {
        perror("mq send failed \n");
    }
}

void
Thingy_Consumer::disconnect() {
    this->supplier_proxy_->disconnect_push_supplier ();
}

void
Thingy_Consumer::disconnect_push_consumer ()
{
    this->supplier_proxy_ = CosEventChannelAdmin::ProxyPushSupplier::_nil ();
}




static void client() {
    puts("client");
    try {
        // First initialize the ORB, that will remove some arguments...
        int argc = 1;
        const char *argv[] = {"client", NULL};
        CORBA::ORB_var orb =
            CORBA::ORB_init (argc, (char**)argv);
        CORBA::Object_var poa_object =
            orb->resolve_initial_references ("RootPOA");
        PortableServer::POA_var poa =
            PortableServer::POA::_narrow (poa_object.in ());
        PortableServer::POAManager_var poa_manager =
            poa->the_POAManager ();
        poa_manager->activate ();

        CORBA::Object_var naming_context_object =
            orb->resolve_initial_references ("NameService");
        CosNaming::NamingContext_var naming_context =
            CosNaming::NamingContext::_narrow (naming_context_object.in ());

        CosNaming::Name name (1);
        name.length (1);
        name[0].id = CORBA::string_dup ("CosEventService");

        CORBA::Object_var ec_object =
            naming_context->resolve (name);

        // Now downcast the object reference to the appropriate type
        CosEventChannelAdmin::EventChannel_var ec =
            CosEventChannelAdmin::EventChannel::_narrow (ec_object.in ());

        Thingy_Consumer consumer_i;
        consumer_i.connect (ec.in ());

        puts("run client");
        orb->run ();

        consumer_i.disconnect ();

        poa->destroy (1, 1);
        orb->destroy ();
    }
    catch (CORBA::Exception &) {
        cerr << "CORBA exception raised!" << endl;
    }
}

static void server() {
    puts("server\n");
    try {
        // First initialize the ORB, that will remove some arguments...
        int argc = 1;
        const char *argv[] = {"server", NULL};
        CORBA::ORB_var orb =
            CORBA::ORB_init (argc, (char**)argv);
        CORBA::Object_var poa_object =
            orb->resolve_initial_references ("RootPOA");
        PortableServer::POA_var poa =
            PortableServer::POA::_narrow (poa_object.in ());
        PortableServer::POAManager_var poa_manager =
            poa->the_POAManager ();
        poa_manager->activate ();

        // Create the servant
        CosThingy_Factory_i factory_i;

        // Activate it to obtain the object reference
        ipcbench::Thingy_Factory_var factory = factory_i._this ();

        // Get the Naming Context reference
        CORBA::Object_var naming_context_object =
            orb->resolve_initial_references ("NameService");
        CosNaming::NamingContext_var naming_context =
            CosNaming::NamingContext::_narrow (naming_context_object.in ());

        // Create and initialize the name.
        CosNaming::Name name (1);
        name.length (1);
        name[0].id = CORBA::string_dup ("Thingy_Factory");

        // Bind the object
        naming_context->rebind (name, factory.in ());

        // Resolve the Event Service
        name[0].id = CORBA::string_dup ("CosEventService");

        CORBA::Object_var ec_object = naming_context->resolve (name);

        // Now downcast the object reference to the appropriate type
        CosEventChannelAdmin::EventChannel_var ec =
            CosEventChannelAdmin::EventChannel::_narrow (ec_object.in());

        CosEventChannelAdmin::SupplierAdmin_var supplier_admin =
            ec->for_suppliers ();

        factory_i.load( poa.in (),
                        poa_manager.in (),
                        supplier_admin.in ());

        // ****************************************************************
        {
            puts("server running");
            struct timespec ts = {0,0};
            while(1) {
                factory_i.obj.set_timespec( ts.tv_sec++, ts.tv_nsec++ );
                sleep(1);
            }
        }


        // Destroy the POA, waiting until the destruction terminates
        poa->destroy (1, 1);
        orb->destroy ();
    }
    catch (CORBA::Exception &) {
        cerr << "CORBA exception raised!" << endl;
    }
}

static CORBA::ORB_var orb;
static CORBA::Object_var factory_object;
static ipcbench::Thingy_Factory_var factory;
static ipcbench::Thingy_var thing;

static CORBA::Object_var poa_object;
static PortableServer::POA_var poa;
static PortableServer::POAManager_var poa_manager;
static CosThingy_Factory_i factory_i;

static Thingy_Consumer consumer_i;
static void s_init_recv() {
    if( (mq = mq_open(MQ, O_CREAT | O_WRONLY | O_NONBLOCK, 0600, &mq_lat_attr )) < 0 ) {
        perror( "could not open mq" );
        abort();
    }


    int argc = 1;
    const char *argv[] = {"client", NULL};
    orb = CORBA::ORB_init (argc, (char**)argv);
    poa_object = orb->resolve_initial_references ("RootPOA");
    poa = PortableServer::POA::_narrow (poa_object.in ());
    poa_manager = poa->the_POAManager ();
    poa_manager->activate ();

    CORBA::Object_var naming_context_object =
        orb->resolve_initial_references ("NameService");
    CosNaming::NamingContext_var naming_context =
        CosNaming::NamingContext::_narrow (naming_context_object.in ());

    CosNaming::Name name (1);
    name.length (1);
    name[0].id = CORBA::string_dup ("CosEventService");

    CORBA::Object_var ec_object =
        naming_context->resolve (name);

    // Now downcast the object reference to the appropriate type
    CosEventChannelAdmin::EventChannel_var ec =
        CosEventChannelAdmin::EventChannel::_narrow (ec_object.in ());

    consumer_i.connect (ec.in ());

    //puts("run client");

    //consumer_i.disconnect ();


}

static void s_init_send() {
    int argc = 1;
    const char *argv[] = {"server", NULL};
    orb = CORBA::ORB_init (argc, (char**)argv);
    poa_object = orb->resolve_initial_references ("RootPOA");
    poa = PortableServer::POA::_narrow (poa_object.in ());
    poa_manager = poa->the_POAManager ();
    poa_manager->activate ();

    // Activate it to obtain the object reference
    factory = factory_i._this ();

    // Get the Naming Context reference
    CORBA::Object_var naming_context_object =
        orb->resolve_initial_references ("NameService");
    CosNaming::NamingContext_var naming_context =
        CosNaming::NamingContext::_narrow (naming_context_object.in ());

    // Create and initialize the name.
    CosNaming::Name name (1);
    name.length (1);
    name[0].id = CORBA::string_dup ("Thingy_Factory");

    // Bind the object
    naming_context->rebind (name, factory.in ());

    // Resolve the Event Service
    name[0].id = CORBA::string_dup ("CosEventService");

    CORBA::Object_var ec_object = naming_context->resolve (name);

    // Now downcast the object reference to the appropriate type
    CosEventChannelAdmin::EventChannel_var ec =
        CosEventChannelAdmin::EventChannel::_narrow (ec_object.in());

    CosEventChannelAdmin::SupplierAdmin_var supplier_admin =
        ec->for_suppliers ();

    factory_i.load( poa.in (),
                    poa_manager.in (),
                    supplier_admin.in ());
}

static void s_nop_ts(const struct timespec *ts) { (void)ts; }


static void s_destroy_recv(void) {
    poa->destroy (1, 1);
    orb->destroy ();
}

static void s_destroy_send(void) {
    poa->destroy (1, 1);
    orb->destroy ();
}

static void s_send( const struct timespec *ts ) {
    factory_i.obj.set_timespec( ts->tv_sec, ts->tv_nsec );
}

static void s_send_loop(void) {
    s_init_send();
    orb->run ();
    s_destroy_send();
}

static void s_recv_loop(int emit) {
    s_init_recv();
    orb->run ();
    s_destroy_recv;
}

struct ipcbench_vtab ipc_bench_vtab_cos = {
    NULL, // init
    s_init_send, // init_send
    s_init_recv, // init_recv
    s_send, // send
    NULL, // recv
    s_destroy_send, // destroy_send
    s_destroy_recv, // destroy_recv
    NULL, // destroy
    NULL, //send_loop
    s_recv_loop // recv_loop
};

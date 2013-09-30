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

#include "ace/streams.h"
#include "ipcbenchS.h"
#include <stdio.h>
#include <time.h>

#include "util.h"

class Thingy_i : public POA_ipcbench::Thingy {
public:
    virtual ipcbench::corba_timespec getit ( void );
private:
};


//Thingy_i::Thingy_i() {};

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
    return this->obj._this();
}



int main (int argc, char* argv[])
{
  try {
    // First initialize the ORB, that will remove some arguments...
    CORBA::ORB_var orb =
      CORBA::ORB_init (argc, argv,
                       "my_orb" /* the ORB name, it can be anything! */);

    CORBA::Object_var poa_object =
      orb->resolve_initial_references ("RootPOA");
    PortableServer::POA_var poa =
      PortableServer::POA::_narrow (poa_object.in ());

    PortableServer::POAManager_var poa_manager =
        poa->the_POAManager ();

    poa_manager->activate ();


    // Create the servant
    Thingy_Factory_i factory_i;

    // Activate it to obtain the object reference
    ipcbench::Thingy_Factory_var factory =
        factory_i._this ();

    // Put the object reference as an IOR string
    CORBA::String_var ior = orb->object_to_string (factory.in ());
    // Print it out!
    cout << ior.in () << endl;


    make_realtime(30);
    calibrate();

    orb->run ();



    // TAO-specific descructor
    poa->destroy (1, 1);
    orb->destroy ();
  }
  catch (CORBA::Exception &ex) {
      std::cerr << "CORBA exception raised!" << std::endl;
  }
  return 0;
}

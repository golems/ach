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

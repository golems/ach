#include "ipcbenchC.h"
#include "ace/streams.h"
#include <stdio.h>

int main (int argc, char* argv[])
{
  try {
    // First initialize the ORB, that will remove some arguments...
    CORBA::ORB_var orb =
      CORBA::ORB_init (argc, argv,
                       "my_orb" /* the ORB name, it can be anything! */);


    CORBA::Object_var factory_object =
        orb->string_to_object (argv[1]);

    ipcbench::Thingy_Factory_var factory =
        ipcbench::Thingy_Factory::_narrow (factory_object.in ());

    ipcbench::Thingy_var thing = factory->get_thingy ( );

    ipcbench::corba_timespec ts = thing->getit();

    printf("Time: %ul, %ul\n", ts.sec, ts.nsec );

    // TAO-specific descructor
    orb->destroy ();
  }
  catch (CORBA::Exception &ex) {
      std::cerr << "CORBA exception raised!" << std::endl;
  }
  return 0;
}

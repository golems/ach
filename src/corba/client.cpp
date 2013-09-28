#include "ipcbenchC.h"
#include "ace/streams.h"
#include <stdio.h>

#include "util.h"

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

    make_realtime(30);
    calibrate();

    while(1) {
        ipcbench::corba_timespec cts = thing->getit();
        struct timespec ts = {.tv_sec = cts.sec, .tv_nsec = cts.nsec };
        struct timespec now = get_ticks();
        printf("delta: %lf us\n", ticks_delta( ts, now ) * 1e6);
        usleep(1e3);
    }

    // TAO-specific descructor
    orb->destroy ();
  }
  catch (CORBA::Exception &ex) {
      std::cerr << "CORBA exception raised!" << std::endl;
  }
  return 0;
}

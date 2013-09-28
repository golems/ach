

# CHANGE ACE_ROOT FOR YOUR SYSTEM
ACE_ROOT = /home/ntd/src/3rdparty/ACE_wrappers


all: client server

make_idl:
	$(ACE_ROOT)/TAO/TAO_IDL/tao_idl ipcbench.idl

INCLUDES = -I$(ACE_ROOT)/TAO -I$(ACE_ROOT) -I../../include
LIBS = -L$(ACE_ROOT)/lib

client: client.cpp
	g++ $(INCLUDES) $(LIBS) -lACE -lTAO -lTAO_AnyTypeCode client.cpp ipcbenchC.cpp -o client

SLIBS =             \
	-lACE -lTAO \
	-lTAO_AnyTypeCode \
	-lTAO_TypeCodeFactory \
	-lTAO_IDL_BE \
	-lTAO_IDL_FE \
	-lTAO_PortableServer \
	-lTAO_IFR_Client

server: server.cpp
	g++ $(INCLUDES) $(LIBS) $(SLIBS) server.cpp ipcbenchS.cpp ipcbenchC.cpp -o server



run_server:
	LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):$(ACE_ROOT)/lib ./server > /tmp/ior_file

run_client:
	LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):$(ACE_ROOT)/lib ./client file:///tmp/ior_file

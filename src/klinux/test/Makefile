CC=gcc

all: achlog achk-test achbench

ach.o: ../../ach.c
	$(CROSS_COMPILE)$(CC) -c -I../../../include  ../../ach.c 

achk-test: achk-test.c ach.o
	$(CROSS_COMPILE)$(CC) achk-test.c ach.o -L. -lrt -lpthread -o achk-test


achutil.o:  ../../achutil.c
	$(CROSS_COMPILE)$(CC) -c -I../../../include -I../../.. -DHAVE_CONFIG_H  ../../achutil.c 

pipe.o:  ../../pipe.c
	$(CROSS_COMPILE)$(CC) -c -I../../../include ../../pipe.c

achlog: ../../achlog.c achutil.o ach.o pipe.o
	$(CROSS_COMPILE)$(CC) -I../../../include ../../achlog.c ach.o achutil.o pipe.o -lrt -lpthread -o achlog

achbench: ../../ach-bench.c ach.o
	$(CROSS_COMPILE)$(CC) -I../../../include ../../ach-bench.c ach.o achutil.o -lrt -lpthread -o achbench

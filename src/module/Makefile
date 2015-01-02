obj-m += ach.o
CFLAGS_ach.o += -DDEBUG

all:
	make -C $(KERNELDIR) M=$(PWD) modules

clean:
	make -C $(KERNELDIR) M=$(PWD) clean


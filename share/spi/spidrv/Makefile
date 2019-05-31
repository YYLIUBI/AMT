export ARCH = arm
CROSS_COMPILE = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-
CC = $(CROSS_COMPILE)gcc
	obj-m:=spidrv.o
   KDIR:= /home/work/linux-2.6.30
all:
	make -C  $(KDIR) M=`pwd` modules 
clean:
	make -C  $(KDIR) M=`pwd` clean



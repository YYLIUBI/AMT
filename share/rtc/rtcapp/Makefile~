
CC = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -lpthread -lm

all: mfemapp.o  
	$(CC) $(CFLAGS) -o mfemapp mfemapp.o  
mfemapp.o: mfemapp.c 
	$(CC) -c mfemapp.c
clean:
	-rm -rf *.o
	rm -f mfemapp
install:
	chmod a+x mfemapp


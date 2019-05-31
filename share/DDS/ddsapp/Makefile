
CC = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -lpthread -lm

all: ddsapp.o  
	$(CC) $(CFLAGS) -o ddsapp ddsapp.o  
ddsapp.o: ddsapp.c 
	$(CC) -c ddsapp.c
clean:
	-rm -rf *.o
	rm -f ddsapp
install:
	chmod a+x ddsapp


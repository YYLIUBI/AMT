
CC = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -lpthread -lm

all: eepromapp.o  
	$(CC) $(CFLAGS) -o eepromapp eepromapp.o  
eepromapp.o: eepromapp.c 
	$(CC) -c eepromapp.c
clean:
	-rm -rf *.o
	rm -f eepromapp
install:
	chmod a+x eepromapp


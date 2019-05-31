
CC = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -lpthread -lm

all: mfemmt.o  gps_rtc.o  ad.o tbl.o
	$(CC) $(CFLAGS) -o mfemmt mfemmt.o  gps_rtc.o  ad.o tbl.o
#	cp mfemmt /home/work/
mfemmt.o: mfemmt.c  macro_def.h  gps_rtc.h	ad.h
	$(CC) -c mfemmt.c
gps_rtc.o: gps_rtc.c gps_rtc.h macro_def.h
	$(CC)  -c gps_rtc.c
ad.o: ad.c ad.h macro_def.h
	$(CC)  -c ad.c
tbl.o: tbl.c tbl.h macro_def.h 
	$(CC)  -c tbl.c
clean:
	-rm -rf *.o
#	rm -f /home/work/mfemmt
	rm -f mfemmt
install:
	chmod a+x mfemmt

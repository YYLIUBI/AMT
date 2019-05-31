
CC = /usr/local/arm/arm-2007q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -lpthread -lm

all: mfemcal.o  gps_rtc.o  build_prm.o ad.o
	$(CC) $(CFLAGS) -o mfemcal mfemcal.o  gps_rtc.o  build_prm.o ad.o
#	cp mfemcal /home/work/
mtemapp.o: mfemcal.c  macro_def.h  build_prm.h gps_rtc.h	ad.h
	$(CC) -c mfemcal.c
gps_rtc.o: gps_rtc.c gps_rtc.h macro_def.h
	$(CC)  -c gps_rtc.c
build_prm.o: build_prm.c build_prm.h macro_def.h	
	$(CC)  -c build_prm.c
ad.o: ad.c ad.h build_prm.h macro_def.h
	$(CC)  -c ad.c
clean:
	-rm -rf *.o
#	rm -f /home/work/mfemcal
#	rm -f mfemcal
install:
	chmod a+x mfemcal

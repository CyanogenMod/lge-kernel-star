#CC=arm-generic-linux-gnueabi-gcc
#CC=arm-linux-gcc
CC=/home2/kyoungha.moon/working/bssq-20110415-4PM/android/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-gcc

TARGET		= mtv250app

INCLUDES	= -I. -I./.. -I/home2/kyoungha.moon/working/bssq-20110415-4PM/android/prebuilt/ndk/android-ndk-r4/platforms/android-8/arch-arm/usr/include
SUBLIBS		=  -lpthread
CFLAGS		= -Wall -D_REENTRANT 
LDFLAGS		= 

SRCS    = test.c 
OBJS	= $(SRCS:.c=.o)


.SUFFIXES: .c .o

# .PHONY: all clean

.c.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<

$(TARGET):	$(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(SUBLIBS) -lm -static

all:	$(TARGET)

clean:
	rm -f $(TARGET) *.o *.a *~

distclean:
	rm -f $(TARGET) *.o *.a *.bak *~ .depend

dep:    depend

depend:
	$(CC) -MM $(CFLAGS) $(SRCS) 1>.depend


#
# include dependency files if they exist
#
ifneq ($(wildcard .depend),)
include .depend
endif


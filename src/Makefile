TARGET		= libdxl.a
OBJS		= dxl_hal.o dynamixel.o AX12A.o dmath.o serialCommuni.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../include
LIBDIRS		+=
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2

#CC			= gcc
#AR			= ar
CC			= arm-linux-gnueabi-gcc
AR			= arm-linux-gnueabi-ar

$(TARGET): $(OBJS)
	$(AR) rs $@ $^
	cp $(TARGET) ../lib

.c.o:
	$(CC) -c $< $(CFLAGS)

clean:
	rm -f $(OBJS) $(TARGET)
	rm -f *~
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

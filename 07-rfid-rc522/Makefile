TEMPLATEROOT = C:/Users/trinch/repos/STM32F7-Template

# compilation flags for gdb

CFLAGS  = -O1 -g
ASFLAGS = -g

# object files

## all user-defined .c files in current directory
#SRCS=$(wildcard *.c)
#SRCS=tm_stm32f4_disco.c
SRCS=rfid-rc522.c
OBJS_CURR=$(SRCS:.c=.o)

OBJS=  $(STARTUP) $(OBJS_CURR) main.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common

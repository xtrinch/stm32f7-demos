TEMPLATEROOT = C:/Users/trinch/repos/STM32F7-Template

# compilation flags for gdb

CFLAGS  = -O1 -g
ASFLAGS = -g

# object files

## all .c files in current directory
#SRCS=$(wildcard *.c)
#SRCS=tm_stm32f4_disco.c
SRCS=
OBJS_CURR=$(SRCS:.c=.o)

OBJS=  $(STARTUP) $(OBJS_CURR) ethernetif.o main.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common

CC = iccavr
LIB = ilibw
CFLAGS =  -e -D__ICC_VERSION=722 -DATMega128  -l -g -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -g -e:0x20000 -ucrtatmega.o -bfunc_lit:0x8c.0x20000 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:30 -beeprom:0.4096 -fihx_coff -S2
FILES = Robot_Program.o 

ROBOT_PROGRAM:	$(FILES)
	$(CC) -o ROBOT_PROGRAM $(LFLAGS) @ROBOT_PROGRAM.lk   -lcatm128
Robot_Program.o: C:\iccv7avr\include\iom128v.h C:\iccv7avr\include\macros.h C:\iccv7avr\include\AVRdef.h C:\iccv7avr\include\string.h C:\iccv7avr\include\_const.h
Robot_Program.o:	Robot_Program.c
	$(CC) -c $(CFLAGS) Robot_Program.c

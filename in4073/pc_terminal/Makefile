CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc_terminal

all:
	$(CC) $(CFLAGS) ../protocol.c serial.c term.c keyboard.c joystick.c pc_terminal.c -o $(EXEC)

run: all
	$(EXEC)


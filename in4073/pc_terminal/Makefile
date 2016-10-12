CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc_terminal

all:
	$(CC) $(CFLAGS) -pthread ../protocol.c serial.c term.c keyboard.c joystick.c command.c pc_terminal.c -o $(EXEC)

run: all
	$(EXEC)


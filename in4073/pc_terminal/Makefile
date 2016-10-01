CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) ../logging_protocol.c pc_terminal.c -o $(EXEC)

run: all
	$(EXEC)


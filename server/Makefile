INSTALLROOT=$(PWD)
#cc = gcc
CC = arm-linux-gcc

INSTALL = install
BIN = /usr/local/bin

SERVFLAGS = -O2 -DLINUX $(WARNINGS)

SERVFLIBS = -lpthread
WARNINGS = -Wall\

OBJSERVER = server.o tcputils.o serial.o client.o

all: tcp_server

clean:
	rm -f *.a *.o tcp_server     core *~ log errlog

tcp_server:$(OBJSERVER)
	$(CC) $(SERVFLAG) -o tcp_server $(OBJSERVER) $(SERVFLIBS)

serial.o: serial.c serial.h
	$(CC) $(SERVFLAG) -c -o $@ $<
tcputils.o: tcputils.c tcputils.h
	$(CC) $(SERVFLAG) -c -o $@ $<
client.o:client.c client.h
	$(CC) $(SERVFLAG) -c -o $@ $<
server.o:server.c package.h
	$(CC) $(SERVFLAG) -c -o $@ $<



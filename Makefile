CROSS_COMPILE=        ../toolchain/bin/mips-linux-gnu-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld

CFLAGS=-std=c99 -muclibc -O2 -lrt

pswdt: pswdt.c
	$(CC) $(CFLAGS) -o $@ $<


.PHONY: clean
clean:
	rm -f pswdt

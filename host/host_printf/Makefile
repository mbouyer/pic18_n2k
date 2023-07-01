.PATH: ${.CURDIR}/../n2k

CFLAGS+= -I${.CURDIR} -I${.CURDIR}/../n2k -I${.CURDIR}/../..

all: n2k_printf

n2k_printf: n2k_printf.o host_nmea2000.o
	cc -o ${.TARGET} ${.ALLSRC}


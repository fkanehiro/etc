all: udpsend udprecv

udpsend udprecv:
	gcc -o $@ $@.c

clean:
	rm -rf udpsend udprecv *~

udpsend: udpsend.c
udprecv: udprecv.c

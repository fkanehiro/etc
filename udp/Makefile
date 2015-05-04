all: udpsend udprecv stdin2udp udp2stdout

udpsend udprecv:
	gcc -o $@ $@.c

stdin2udp udp2stdout:
	g++ -o $@ $@.cpp

clean:
	rm -rf udpsend udprecv stdin2dup *~

udpsend: udpsend.c
udprecv: udprecv.c
stdin2udp: stdin2udp.cpp
udp2stdout: udp2stdout.cpp

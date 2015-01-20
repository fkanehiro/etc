all: tcpsend tcprecv

tcpsend.o: tcpsend.cpp SocketUtil.h
tcprecv.o: tcprecv.cpp SocketUtil.h
SocketUtil.o: SocketUtil.cpp SocketUtil.h

tcpsend: tcpsend.o SocketUtil.o
tcprecv: tcprecv.o SocketUtil.o

tcpsend tcprecv:
	g++ -o $@ $^

tcpsend.o tcprecv.o SocketUtil.o:
	g++ -c $<

clean:
	-rm -f *.o *~ tcprecv tcpsend

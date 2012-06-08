all: test test2

test: test.o
test2: test2.o

test test2:
	g++ -o $@ $^ `pkg-config --libs sot-dynamic` `pkg-config --libs dynamic-graph-corba`

test.o: test.cpp
test2.o: test2.cpp

test.o test2.o:
	g++ -Wall -g -O2 -c $< `pkg-config --cflags sot-dynamic`

clean:
	-rm *.o *~ test test2
 
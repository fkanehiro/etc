all: test test2 test3 test4

test: test.o
test2: test2.o
test3: test3.o
test4: test4.o
test test2 test3 test4:
	g++ -o $@ $< `pkg-config --libs hmc`

test.o: test.cpp
test2.o: test2.cpp
test3.o: test3.cpp
test4.o: test4.cpp
test.o test2.o test3.o test4.o:
	g++ -c -g -Wall $< -o $@ `pkg-config --cflags hmc`
  
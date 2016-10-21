all: test

test: test.cpp
	g++ test.cpp -O2 `pkg-config --cflags --libs sch-core`

clean:
	-rm -rf test *~

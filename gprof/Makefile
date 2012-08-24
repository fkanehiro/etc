all: run

test: test.cpp
	g++ -pg -O test.cpp -o test

run: test
	./test
	gprof ./test gmon.out | less

clean:
	-rm -f test *~ gmon.out

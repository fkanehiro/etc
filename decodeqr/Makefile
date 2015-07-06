all: sample

sample: sample.cpp
	g++ `pkg-config --cflags opencv` sample.cpp -o sample `pkg-config --libs opencv` -ldecodeqr

clean:
	-rm -rf sample *~


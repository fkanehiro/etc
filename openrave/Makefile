all: iktest

iktest: iktest.o ik.o
	g++ -o iktest iktest.o ik.o `pkg-config --libs hmc2`  

iktest.o: iktest.cpp 
	g++ -c iktest.cpp `pkg-config --cflags hmc2` -I$(HOME)/src/openrave/python 

ik.o: ik.cpp
	g++ -c ik.cpp `pkg-config --cflags hmc2` -I$(HOME)/src/openrave/python -DIKFAST_NO_MAIN

clean: 
	-rm -rf iktest *.o *~

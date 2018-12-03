all: libexample_nodes.so example2.1

CXX_FLAGS=-I${HOME}/openrtp/include -g -std=gnu++11 -fPIE

example2.1: example2.1.o
	g++ -o example2.1 $^ -L${HOME}/openrtp/lib -lbehaviortree_cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -L. -lexample_nodes

example2.1.o: example2.1.cpp
	g++  ${CXX_FLAGS} -c $<

example_nodes.o: example_nodes.cpp example_nodes.h
	g++  ${CXX_FLAGS} -c $<

libexample_nodes.so: example_nodes.o
	g++ -shared -o $@ $^ -L${HOME}/openrtp/lib -lbehaviortree_cpp 

clean:
	-rm -rf *.o *~ *.fbl *.json example2.1 example2.1.dSYM


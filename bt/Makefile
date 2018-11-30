all: example2.1

CXX_FLAGS=-I${HOME}/openrtp/include -g -std=gnu++11 -fPIE

example2.1: example2.1.o example_nodes.o
	g++ -o example2.1 $^ -L${HOME}/openrtp/lib -lbehaviortree_cpp -lopencv_core -lopencv_highgui -lopencv_imgproc

example2.1.o: example2.1.cpp example_nodes.h
	g++  ${CXX_FLAGS} -c $<

example_nodes.o: example_nodes.cpp example_nodes.h
	g++  ${CXX_FLAGS} -c $<

clean:
	-rm -rf *.o *~ *.fbl *.json example2.1 example2.1.dSYM


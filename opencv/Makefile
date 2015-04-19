all: viewer oval circle

viewer: viewer.cpp
	g++ viewer.cpp -o viewer `pkg-config --cflags --libs opencv`

oval: oval.cpp
	g++ oval.cpp -o oval `pkg-config --cflags --libs opencv`

circle: circle.cpp
	g++ circle.cpp -o circle `pkg-config --cflags --libs opencv`

clean : 
	-rm -f viewer *.o *~ core oval circle
 

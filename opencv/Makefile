all: viewer oval circle rectangle shape hough

viewer: viewer.cpp
	g++ viewer.cpp -o viewer `pkg-config --cflags --libs opencv`

hough: hough.cpp
	g++ hough.cpp -o hough `pkg-config --cflags --libs opencv`

oval: oval.cpp
	g++ oval.cpp -o oval `pkg-config --cflags --libs opencv`

circle: circle.cpp
	g++ circle.cpp -o circle `pkg-config --cflags --libs opencv`

rectangle: rectangle.cpp
	g++ rectangle.cpp -o rectangle `pkg-config --cflags --libs opencv`

shape: shape.cpp
	g++ shape.cpp -o shape `pkg-config --cflags --libs opencv`

clean : 
	-rm -f viewer *.o *~ core oval circle rectangle shape hough
 

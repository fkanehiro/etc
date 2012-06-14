all: viewer

viewer: viewer.cpp
	g++ viewer.cpp -lopencv_core -I/usr/include/opencv-2.3.1/opencv -I/usr/include/opencv-2.3.1 -lopencv_highgui

clean : 
	-rm -f viewer *.o *~ core
 
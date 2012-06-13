all: colmap

colmap: colmap.cpp
	g++ -o $@ $< `pkg-config --cflags --libs openhrp3.1`

clean: 
	-rm -rf colmap *.o *~ core

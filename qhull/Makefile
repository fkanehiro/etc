all: qhull

qhull: qhull.cpp
	g++ -o $@ $< `pkg-config --cflags --libs openhrp3.1` -lqhull

clean:
	-rm -rf *~ qhull 


all: qhull bbox obb

qhull: qhull.cpp
	g++ -o $@ $< `pkg-config --cflags --libs openhrp3.1` -lqhull

bbox: bbox.cpp
	g++ -o $@ $< `pkg-config --cflags --libs openhrp3.1`

obb: obb.cpp
	g++ -o $@ $< `pkg-config --cflags --libs openhrp3.1`

clean:
	-rm -rf *~ qhull bbox obb


all: scan_image

scan_image: scan_image.cpp
	g++ -o $@ $< -lzbar `pkg-config --cflags --libs opencv`

clean:
	-rm -rf scan_image *~



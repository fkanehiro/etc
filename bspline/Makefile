all: main

OBJS = main.o bspline.o

main: $(OBJS)
	g++ -o $@ $^
 
$(OBJS): 
	g++ -c $< -o $@ -g `pkg-config --cflags openhrp3.1`

main.o: main.cpp
bspline.o: bspline.cpp

clean:
	-rm -f *.o *~



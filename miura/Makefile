all: extract

extract: extract.o
	g++ -o $@ $^ `pkg-config --libs hmc`

extract.o: extract.cpp
	g++ -c $< -o $@ `pkg-config --cflags hmc`

clean:
	-rm -f *.o *~ extract

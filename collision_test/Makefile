all: collision_test

collision_test: collision_test.cpp
	g++ -o $@ $< `pkg-config --cflags --libs hmc2`

clean: 
	-rm -rf collision_test *.o *~ core

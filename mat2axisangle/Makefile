all: mat2axisangle

mat2axisangle: mat2axisangle.cpp
	g++ -o mat2axisangle $< `pkg-config --cflags --libs openhrp3.1` 

clean:
	-rm -rf *.o mat2axisangle *~

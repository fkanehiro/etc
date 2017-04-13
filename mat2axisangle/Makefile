all: mat2axisangle rpy2axisangle

mat2axisangle: mat2axisangle.cpp
	g++ -o mat2axisangle $< `pkg-config --cflags --libs openhrp3.1` 

rpy2axisangle: rpy2axisangle.cpp
	g++ -o rpy2axisangle $< `pkg-config --cflags --libs openhrp3.1` 

clean:
	-rm -rf *.o mat2axisangle rpy2axisangle *~

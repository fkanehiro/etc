all: mat2axisangle rpy2axisangle axisangle2mat

axisangle2mat: axisangle2mat.cpp
	g++ -o axisangle2mat $< `pkg-config --cflags --libs openhrp3.1` 

mat2axisangle: mat2axisangle.cpp
	g++ -o mat2axisangle $< `pkg-config --cflags --libs openhrp3.1` 

rpy2axisangle: rpy2axisangle.cpp
	g++ -o rpy2axisangle $< `pkg-config --cflags --libs openhrp3.1` 

clean:
	-rm -rf *.o mat2axisangle axisangle2mat rpy2axisangle *~

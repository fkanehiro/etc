all: fk inverse rpy

fk: fk.cpp
inverse: inverse.cpp
rpy: rpy.cpp

fk inverse rpy:
	g++ -o $@ `pkg-config --cflags openhrp3.1` $< `pkg-config --libs openhrp3.1`

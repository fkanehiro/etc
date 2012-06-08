all: genogmap ogmaptest

genogmap: genogmap.o
ogmaptest: ogmaptest.o

genogmap ogmaptest:
	g++ -o $@ $^ `pkg-config --libs openhrp3.1` -loctomap -loctomath

genogmap.o: genogmap.cpp
ogmaptest.o: ogmaptest.cpp

genogmap.o ogmaptest.o:
	g++ -c $< `pkg-config --cflags openhrp3.1` -I/usr/local/include 

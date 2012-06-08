all: libchildA.so libchildB.so main pluginA.so pluginB.so

CXXFLAGS = -Wall -O2 -g

main: main.o
	g++ -o $@ $^ -ldl

libchildA.so libchildB.so:
	g++ -shared -o $@ $^

pluginA.so:
	g++ -shared -o $@ -L. -lchildA $^

pluginB.so:
	g++ -shared -o $@ -L. -lchildA -lchildB $^

main.o childA.o childB.o pluginA.o pluginB.o:
	g++ $(CXXFLAGS) -c $< 

clean:
	-rm -rf *~ *.o main *.so


main.o: main.cpp parent.h port.h
childA.o: childA.cpp childA.h parent.h port.h
childB.o: childB.cpp childB.h parent.h port.h
libchildA.so: childA.o
libchildB.so: childB.o
pluginA.o: pluginA.cpp childA.h parent.h port.h
pluginB.o: pluginB.cpp childB.h parent.h port.h
pluginA.so: pluginA.o
pluginB.so: pluginB.o

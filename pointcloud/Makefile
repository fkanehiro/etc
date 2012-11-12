all: pc2wrl

pc2wrl: pc2wrl.cpp
	g++ -o $@ `pkg-config --cflags openhrp3.1` `pkg-config --libs openhrp3.1` $<

clean:
	-rm -rf a.out pc2wrl *~

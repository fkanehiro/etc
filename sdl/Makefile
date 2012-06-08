all: sdl texture ttf main

sdl: sdl.cpp
	g++ -o $@ $< `sdl-config --cflags --libs`
main: main.cpp
	g++ -o $@ $< `sdl-config --cflags --libs` -lGL -lGLU
texture: texture.cpp
	g++ -o $@ $< `sdl-config --cflags --libs` -lGL
ttf: ttf.cpp
	g++ -o $@ $< `sdl-config --cflags --libs` -lSDL_ttf

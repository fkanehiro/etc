all: collision_map

collision_map: collision_map.cpp
	g++ -o collision_map $< `pkg-config --cflags --libs openhrp3.1`

clean:
	-rm -rf collision_map *~

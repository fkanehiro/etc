all: spot spot2

spot: spot.c
	gcc -o $@ $< -lGL -lglut
spot2: spot2.c
	gcc -o $@ $< -lGL -lglut

clean:
	-rm -rf spot spot2 *~

LDLIBS = -L/usr/X11R6/lib -lglut -lGLU -lGL -lXmu -lXi -lXext -lX11 -lpthread -lm
TARGETS = fbo1
OBJECTS = main1.o scene.o trackball.o glsl.o

$(TARGETS): $(OBJECTS)
	$(CXX) $(CFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

main1.o: scene.h trackball.h glsl.h
scene.o: scene.h
trackball.o: trackball.h
glsl.o: glsl.h

clean:
	-rm -f $(TARGETS) *.o *~

all: clip cylinder cube simple_cube 04_stopper_rotation 09_part_g face_check tridist segdisttest deftest lcptest lcptest-sofa defsim cpoints clip_sparse 04_stopper_rotation_sparse 09_part_g_sparse 01_collar_sparse #cublas lcptest-amelif 

#EIGEN_DIR=/usr/include/eigen2
#OPENCV_DIR=/usr
#OPENCV_LIB=-L$(OPENCV_DIR)/lib -lcv -lhighgui -lsuperlu 
#GL_LIB=-lGLU
EIGEN_DIR=/opt/local/include/eigen3 -I/usr/local/include/superlu
OPENCV_DIR=/opt/local
OPENCV_LIB=-L$(OPENCV_DIR)/lib -lopencv_core -lopencv_highgui -L/usr/local/lib -lsuperlu_4.1 -lblas
GL_LIB=-framework OpenGL -framework Cocoa

defsim: defsim.o FEM.o DeformableLink.o ConstraintForceSolverExt.o LinkExt.o
defsim.o: defsim.cpp FEM.h

cublas: cublas.c
	g++ -Wall -O2 -DNDEBUG -o $@ $^ -I$(EIGEN_DIR) -I/usr/local/cuda/include -L/usr/local/cuda/lib -lcublas -lcudart

tridist: tridist.o
segdisttest: segdisttest.o
cpoints: cpoints.o

cpoints tridist segdisttest defsim:
	g++ -o $@ $^ `pkg-config --libs openhrp3.1` $(GL_LIB) -lglfw $(OPENCV_LIB)

tridist.o: tridist.cpp
segdisttest.o: segdisttest.cpp
cpoints.o: cpoints.cpp
ConstraintForceSolverExt.o: ConstraintForceSolverExt.cpp

tridist.o segdisttest.o cpoints.o:
	g++ -g  -c $< `pkg-config --cflags openhrp3.1`

clip: FEM.o clip.o DeformableObject.o
clip_sparse: FEM_sparse.o clip_sparse.o DeformableObject_sparse.o
04_stopper_rotation: FEM.o 04_stopper_rotation.o DeformableObject.o
04_stopper_rotation_sparse: FEM_sparse.o 04_stopper_rotation_sparse.o DeformableObject_sparse.o
01_collar_sparse: FEM_sparse.o 01_collar_sparse.o DeformableObject_sparse.o
09_part_g: FEM.o 09_part_g.o DeformableObject.o
09_part_g_sparse: FEM_sparse.o 09_part_g_sparse.o DeformableObject_sparse.o
cylinder: FEM.o cylinder.o DeformableObject.o
cube: FEM.o cube.o DeformableObject.o
deftest: FEM.o deftest.o DeformableObject.o LCPsolver.o
lcptest: lcptest.o LCPsolver.o
lcptest-sofa: lcptest-sofa.o LCPcalc.o
lcptest-amelif: lcptest-amelif.o gaussSeidel.o
simple_cube: FEM.o simple_cube.o DeformableObject.o
face_check: face_check.o

simple_cube cube clip cylinder 04_stopper_rotation 09_part_g face_check deftest clip_sparse 04_stopper_rotation_sparse 01_collar_sparse 09_part_g_sparse lcptest lcptest-sofa lcptest-amelif:
	g++ -o $@ $^ $(GL_LIB) -lglfw  $(OPENCV_LIB) -lboost_iostreams -L/Users/fkanehiro/amelif/lib #-lamelif

FEM.o: FEM.cpp FEM.h
FEM_sparse.o: FEM_sparse.cpp FEM_sparse.h
DeformableObject.o: DeformableObject.cpp DeformableObject.h FEM.h
DeformableObject_sparse.o: DeformableObject_sparse.cpp DeformableObject_sparse.h
DeformableLink.o: DeformableLink.cpp DeformableLink.h
LinkExt.o: LinkExt.cpp LinkExt.h
clip.o: clip.cpp FEM.h 
clip_sparse.o: clip_sparse.cpp
04_stopper_rotation.o: 04_stopper_rotation.cpp FEM.h
04_stopper_rotation_sparse.o: 04_stopper_rotation_sparse.cpp
01_collar_sparse.o: 01_collar_sparse.cpp
09_part_g.o: 09_part_g.cpp FEM.h
09_part_g_sparse.o: 09_part_g_sparse.cpp
cylinder.o: cylinder.cpp FEM.h
cube.o: cube.cpp FEM.h
deftest.o: deftest.cpp FEM.h
lcptest.o: lcptest.cpp LCPsolver.h
lcptest-sofa.o: lcptest-sofa.cpp LCPcalc.h
lcptest-amelif.o: lcptest-amelif.cpp gaussSeidel.h
LCPcalc.o: LCPcalc.cpp LCPcalc.h
gaussSeidel.o: gaussSeidel.cpp gaussSeidel.h
simple_cube.o: simple_cube.cpp FEM.h
face_check.o: face_check.cpp
LCPsolver.o: LCPsolver.cpp  LCPsolver.h

:


FEM.o simple_cube.o cube.o cylinder.o clip.o DeformableObject.o 04_stopper_rotation.o 09_part_g.o face_check.o LCPsolver.o deftest.o lcptest.o lcptest-sofa.o lcptest-amelif.o gaussSeidel.o defsim.o  ConstraintForceSolverExt.o DeformableLink.o LinkExt.o clip_sparse.o FEM_sparse.o DeformableObject_sparse.o 04_stopper_rotation_sparse.o 09_part_g_sparse.o 01_collar_sparse.o:
	g++ -g -Wall -O2 -c $< -I$(EIGEN_DIR) -I$(OPENCV_DIR)/include/opencv `pkg-config --cflags openhrp3.1` -I/Users/fkanehiro/amelif/include


clean:
	-rm -rf *.o *~ clip cylinder cube simple_cube 04_stopper_rotation 09_part_g face_check deftest lcptest lcptest-sofa lcptest-amelif defsim cublas cpoints tridist segdisttest 04_stopper_rotation_sparse 09_part_g_sparse clip_sparse 01_collar_sparse





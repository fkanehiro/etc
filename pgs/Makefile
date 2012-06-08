all: pgstest1 pgstest2 pgstest3 pgs1 pgs2 pgs3

CXXFLAGS = -I/usr/include/eigen3 -I/home/kanehiro/openrtp/include/OpenHRP-3.1 -DNDEBUG -O3 -msse -msse2 -msse3 -mssse3 -msse4.1 #-DEIGEN_DEFAULT_TO_ROW_MAJOR
pgstest1: pgstest.cpp LCPSolver.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS)

pgstest2: pgstest.cpp LCPSolver.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS) -DUSE_EIGEN

pgstest3: pgstest.cpp LCPSolver.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS) -DUSE_EIGEN -DVECTORIZE

pgs1: pgs.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS) 

pgs2: pgs.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS) -DUSE_EIGEN

pgs3: pgs.cpp
	g++ -o $@ $^ -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET $(CXXFLAGS) -DUSE_EIGEN -DVECTORIZE





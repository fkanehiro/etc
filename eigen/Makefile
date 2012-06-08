all: sparse sparseGS vectest1 vectest2 vectest3 vectest4

CXXFLAGS = -g -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET `pkg-config --cflags eigen3`
sparse: sparse.cpp
	g++ -o $@ $(CXXFLAGS) $^
sparseGS: sparseGS.cpp
	g++ -o $@ $(CXXFLAGS) $^
vectest1: vectest.cpp
	g++ -o $@ $(CXXFLAGS) $^
vectest2: vectest.cpp
	g++ -DNDEBUG -o $@ $(CXXFLAGS) $^
vectest3: vectest.cpp
	g++ -DNDEBUG -O3 -o $@ $(CXXFLAGS) $^
vectest4: vectest.cpp
	g++ -DNDEBUG -O3 -msse -msse2 -msse3 -mssse3 -msse4.1 -o $@ $(CXXFLAGS) $^

Sources = src/cubic_spline_planner.cpp src/polynomials.cpp src/frenet_optimal_trajectory.cpp tests/test.cpp
Target = test.out

# general compiler settings
CXXFLAGS = -std=c++11 
LDFLAGS = -I/usr/include/python2.7 -lpython2.7 

all:
	$(CXX) $(CXXFLAGS) $(Sources) -o $(Target) $(LDFLAGS)

clean:
	@$(RM) $(Target)

.PHONY: clean

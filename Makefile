all: route

route: route.o
	$(CXX) $(PG) $(LIBS) $(FINAL_FLAGS) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) -lm

%.o: %.cpp
	$(CXX) -MMD $(PG) $(INCLUDES) $(FINAL_FLAGS) $(CXXFLAGS) -O3 -std=c++11 -c -o $@ $<

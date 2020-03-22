CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
SIMULATOR_DEPS=utils.o graphics.o world.o
GRAPH_DEPS=graph.o factors.o 
SFML=-lsfml-graphics -lsfml-window -lsfml-system
SLAM_DEPS=$(GRAPH_DEPS) $(SIMULATOR_DEPS) print_results.o

target: 2D

2D: 2D_slam.o $(SLAM_DEPS)
	$(CC) 2D_slam.o $(SLAM_DEPS) $(SFML) -o 2D.out

1D: 1D_slam.o $(SLAM_DEPS)
	$(CC) 1D_slam.o $(SLAM_DEPS) $(SFML) -o 1D.out

nav: navigation.o $(SIMULATOR_DEPS)
	$(CC) navigation.o $(SIMULATOR_DEPS) $(SFML) -o nav.out

graph: test_graph.o $(GRAPH_DEPS)
	$(CC) test_graph.o $(GRAPH_DEPS) -o graph.out

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm *.o *.out


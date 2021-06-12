CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
SIMULATOR_DEPS=utils.o graphics.o world.o
GRAPH_DEPS=graph.o factors.o
SFML=-lsfml-graphics -lsfml-window -lsfml-system -pthread
SLAM_DEPS=$(GRAPH_DEPS) $(SIMULATOR_DEPS) print_results.o slam_utils.o friendly_graph.o
NAV_DEPS=$(SIMULATOR_DEPS) plan.o search.o simulator_world.o

target: 2D 1D nav test_graph

2D: 2D_slam.o $(SLAM_DEPS)
	$(CC) 2D_slam.o $(SLAM_DEPS) $(SFML) -o 2D.out

1D: 1D_slam.o $(SLAM_DEPS)
	$(CC) 1D_slam.o $(SLAM_DEPS) $(SFML) -o 1D.out

nav: navigation.o $(NAV_DEPS)
	$(CC) -g navigation.o $(NAV_DEPS) $(SFML) -o nav.out

test_graph: test_graph.o $(GRAPH_DEPS)
	$(CC) test_graph.o $(GRAPH_DEPS) -o test_graph.out

icp_test: test/icp_test.o icp.o utils.o
	$(CC) test/icp_test.o icp.o utils.o -o icp_test.out

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -g -c -o $@ $<

clean:
	rm *.o *.out


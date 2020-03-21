CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
DEPS=utils.o graphics.o
NON_CC_DEPS=world.h world.inl graph.h factors.h print_results.h
SFML=-lsfml-graphics -lsfml-window -lsfml-system

target: 2D

2D: 2D_slam.o $(DEPS)
	$(CC) 2D_slam.o $(DEPS) $(SFML) -o 2D.out

1D: 1D_slam.o $(DEPS)
	$(CC) 1D_slam.o $(DEPS) $(SFML) -o 1D.out

nav: navigation.o $(DEPS)
	$(CC) navigation.o $(DEPS) $(SFML) -o nav.out

navigation.o: navigation.cpp world.inl world.h
	$(CC) $(CFLAGS) -c -o $@ $<

2D_slam.o: 2D_slam.cpp $(NON_CC_DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

1D_slam.o: 1D_slam.cpp $(NON_CC_DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm *.o *.out


CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
DEPS=utils.o graphics.o
NON_CC_DEPS=world.h world.inl graph.h factors.h print_results.h

target: 2D

2D: 2D_slam.o $(DEPS)
	$(CC) 2D_slam.o $(DEPS) -lsfml-graphics -lsfml-window -lsfml-system -o 2D.out

1D: 1D_slam.o $(DEPS)
	$(CC) 1D_slam.o $(DEPS) -lsfml-graphics -lsfml-window -lsfml-system -o 1D.out

2D_slam.o: 2D_slam.cpp $(NON_CC_DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

1D_slam.o: 1D_slam.cpp $(NON_CC_DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm *.o *.out


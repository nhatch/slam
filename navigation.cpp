#include "world.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

extern const bool IS_2D { true };

const double GRID_RES = 0.5;

int main()
{
	struct termios old_tio, new_tio;
	unsigned char c;

  /* termios fiddling copied from https://shtrom.ssji.net/skb/getc.html */
	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO,&old_tio);

	/* we want to keep the old setting to restore them a the end */
	new_tio=old_tio;

	/* disable canonical mode (buffered i/o) and local echo */
  unsigned int flags = (uint32_t)~ICANON & (uint32_t)~ECHO & (uint32_t)~ISIG;
	new_tio.c_lflag &= flags;
	new_tio.c_cc[VTIME] = 1;
	new_tio.c_cc[VMIN] = 0;

	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

  constexpr int N = 1;
  World<N> w;
  w.addLandmark(3., 1.);
  w.addLandmark(6., -1.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., -3.);
  w.addLandmark(3.1, 1.);
  w.addLandmark(0.1, 1.);
  w.startSimulation();
  w.renderOdom();

  bool diag = false;
  bool truth = false;

  std::cout << "Type 'q' to quit, 'wasd' to move around, 't' to view ground truth.\n";
	do {
    double x_dist = diag ? 1.414*GRID_RES : GRID_RES;
		c=getchar();
    switch(c) {
    case 'w':
      w.moveRobot(0.0, x_dist);
      break;
    case 's':
      w.moveRobot(0.0, -x_dist);
      break;
    case 'a':
      w.moveRobot(M_PI/4, 0.0);
      diag = !diag;
      break;
    case 'd':
      w.moveRobot(-M_PI/4, 0.0);
      diag = !diag;
      break;
    case 't':
      truth = !truth;
      break;
    default:
      break;
    }
    truth ? w.renderTruth() : w.renderOdom();
	} while(c!='q');

	/* restore the former settings */
	tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);

	return 0;
}

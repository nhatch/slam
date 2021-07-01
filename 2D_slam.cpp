
#include <Eigen/LU>
#include "graph.h"
#include "factors.h"
#include "utils.h"
#include "slam_utils.h"
#include "constants.h"
using namespace NavSim;

int main() {
  collectDataAndRunSLAM();
  return 0;
}

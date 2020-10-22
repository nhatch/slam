
#include "utils.h"
#include "icp.h"
#include <random>
#include <iostream>

int main(int argc, char** argv) {
  //transform_t tf = toTransform({1.0, 0.5, M_PI/3});
  transform_t tf = toTransform({0.1, 0.05, M_PI/15});

  points_t cloud({});
  cloud.push_back({0,0,1});
  cloud.push_back({1,0,1});
  cloud.push_back({2,0,1});
  cloud.push_back({0,0.5,1});
  cloud.push_back({0,1,1});
  cloud.push_back({2,2,1});
  int n = cloud.size();

  points_t cloud2({});
  for (point_t p : cloud) {
    cloud2.push_back(tf * p);
  }

  std::cout << "Pre-associated clouds:\n";
  transform_t sol = associated_closed_form(cloud, cloud2);
  std::cout << "Ground truth:\n" << toPose(tf, 0.0) << std::endl;
  std::cout << "ICP:\n" << toPose(sol, 0.0) << std::endl;

  // Shuffle cloud2
  for (int i = 0; i < n; i++) {
    int idx = std::rand() % (n-i);
    point_t tmp = cloud2[idx];
    cloud2[idx] = cloud2[i];
    cloud2[i] = tmp;
  }

  std::cout << "\nNon-associated clouds:\n";
  transform_t sol2 = do_icp(cloud, cloud2);
  std::cout << "Ground truth:\n" << toPose(tf, 0.0) << std::endl;
  std::cout << "ICP:\n" << toPose(sol2, 0.0) << std::endl;

  return 0;
}

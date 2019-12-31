To test a simple graph:

```
g++ test_graph.cpp
./a.out
```

To test a more complicated graph (SLAM, but only in one dimension):

```
g++ world.cpp 1D_slam.cpp -o line.out
./line.out
```

To test planar SLAM:

```
g++ world.cpp 2D_slam.cpp -o plane.out
./plane.out
```

This implementation does not address data association or loop closure issues. The optimizer is simply gradient descent, and it uses dense Jacobians. At some point I may try to speed up the optimization by taking advantage of sparsity in the graph.

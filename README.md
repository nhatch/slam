## Usage

```
make
./2D.out
```

The ground truth trajectory is shown in black, odometry in blue, and the smoothed trajectory in green.

You can also do a simpler version, SLAM in only one dimension:

```
make 1D
./1D.out
```

To try an even simpler graph (not really involving SLAM at all):

```
g++ test_graph.cpp
./a.out
```

This implementation does not address data association or loop closure issues. The optimizer is simply gradient descent, and it uses dense Jacobians. At some point I may try to speed up the optimization by taking advantage of sparsity in the graph.

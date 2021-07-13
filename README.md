# Kinematic-Tree-IK-Solver
This repository provides two implementations of inverse kinematics solver for any generic kinematic tree given the URDF. The first implementation is done with pyDrake and the second implementation is done with Pinocchio + SciPy.

## Installing Dependencies

### PyDrake Implementation:
#### Pip Dependencies: 
[meshcat](https://github.com/rdeits/meshcat-python): Provides a remotely-controllable 3D viewer

[numpy](https://numpy.org/): Library for scientific computing

#### Installing pyDrake:
[pydrake](https://drake.mit.edu/pydrake/) contains python bindings which encompass a subset of the original Drake C++ functionality.  This cannot be installed with pip.  Various methods of installing drake are provided [here](https://drake.mit.edu/python_bindings.html).


### Pinocchio + SciPy Implementation:
#### Conda Dependencies: 
[Pinocchio](https://stack-of-tasks.github.io/pinocchio/): Provides URDF parser and forward kinematics functions

[SciPy](https://www.scipy.org/): Provides SLSQP optimizer.


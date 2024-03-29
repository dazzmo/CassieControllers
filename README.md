# Controllers

A work-in-progress testing platform for controller development and simulation for robotic systems. See the `examples/` folder and `README.md` files therein for full descriptions of the current examples in this repository:

- `examples/arm/`: A simple 3 degree-of-freedom robotic arm. The control task is to track trajectories with the end-effector.
- `examples/cassie:` A model of the bipedal robot Cassie. The objective will eventually be to control the feet and centre-of-mass to track reference trajectories from a higher-level planner.

For an explanation of the controllers in this repository, see `examples/arm/README.md`.

## Requirements and installation

We require the following libraries to run the controllers and their simulations. 
* [glog](https://github.com/google/glog)
* [Mujoco](https://github.com/google-deepmind/mujoco) (at least v3.0.0)
* [Eigen](https://gitlab.com/libeigen/eigen) (at least v3.3)
* [qpOASES](https://github.com/coin-or/qpOASES) (latest version on master before November 2023)

For code generation of kinodynamic quantities of rigid-body systems described by URDF files, we also require the installation of the following:
* [CasADi](https://github.com/casadi/casadi)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio)

If you are working in Linux, install all of these dependencies with the following code.

    cd <path_to_download>
    mkdir build && cd build
    cmake ..
    sudo make install

When building qpOASES, you will need to ensure it builds a shared libary (which is not the default). Change the `cmake ..` step to

    cmake .. -DBUILD_SHARED_LIBS=ON

Similarly, when building Pinocchio, you most likely won't need the python bindings (which have a number of extra dependencies). We recommend building with:

    cmake .. -DBUILD_PYTHON_INTERFACE=OFF

## Building the code

We recommend creating a `build/` folder in the root directory of this repository in which to build the code. To build all examples, enter the following into a terminal:

    mkdir build && cd build
    cmake ..
    make

You can then navigate to `build/examples/arm/` or `build/examples/cassie/` and run `./arm_simulator` or `./cassie_sim`, respectfully.

## Contact

For any questions, please contact Damian Abood (dabo9333@uni.sydney.edu.au) or Nic Barbara (nicholas.barbara@sydney.edu.au).
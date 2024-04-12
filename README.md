# Operational Space Controllers

A testing platform for controller development and simulation for robotic systems that utilise the controllers developed in the [damotion](https://github.com/dazzmo/damotion) library. See the `examples/` folder and `README.md` files therein for full descriptions of the current examples in this repository:

- `examples/arm/`: A simple 3-degree-of-freedom robotic arm. The control task is to track trajectories with the end-effector.
- `examples/cassie:` A model of the bipedal robot Cassie. The objective will eventually be to control the feet and centre of mass to track reference trajectories from a higher-level planner.

## Requirements and installation

We require the following libraries to run the examples:
* [glog](https://github.com/google/glog)
* [damotion](https://github.com/dazzmo/damotion) 
* [Mujoco](https://github.com/google-deepmind/mujoco) (at least v3.0.0)

If you are working in Linux, install all of these dependencies with the following code.

    cd <path_to_download>
    mkdir build && cd build
    cmake ..
    sudo make install

## Building the code

We recommend creating a `build/` folder in the root directory of this repository in which to build the code. To build an example, enter the following into a terminal:

    mkdir build && cd build
    cmake ..
    make <example>

You can then navigate to `build/examples/<example>/` and execute the given example

## Contact

For any questions, please contact Damian Abood (dabo9333@uni.sydney.edu.au) or Nic Barbara (nicholas.barbara@sydney.edu.au).

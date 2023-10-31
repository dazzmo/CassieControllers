# controllers
A work-in-progress testing platform for controller development and simulation for robotic systems.

# Requirements
We require the following libraries to run the controllers and their simulations:
* [glog](https://github.com/google/glog)
* [Mujoco](https://github.com/google-deepmind/mujoco) (at least v3.0.0)
* [Eigen](https://gitlab.com/libeigen/eigen) (at least v3.3)
* [qpOASES](https://github.com/coin-or/qpOASES)

For code generation of kinodynamic quantities of rigid-body systems described URDF files, we also require the installation of the following:
* [Pinicchio](https://github.com/stack-of-tasks/pinocchio)
* [CasADi](https://github.com/casadi/casadi)

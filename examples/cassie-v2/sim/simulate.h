#pragma once

#include <chrono>
#include <iostream>
#include <mujoco/mujoco.h>

#include "mujoco_sim.h"

// TODO: Work these into the simulator class
const double SIM_REFERESH_RATE = 1.0 / 60.0;

// Get real time as a double
double real_time_seconds() {
    auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    auto time_micro = tmp.count();
    return time_micro / 1e6;
}

// Simulator class for Cassie
class CassieSimulator : public MujocoSimulator {
   public:
    ~CassieSimulator() = default;

    // Overload the ResetModel function for special initialisation
    using MujocoSimulator::ResetModel;
    int ResetModel();

   private:    

    // Initial state from cassie-mujoco-sim when pelvis is not a free joint
    const double QPOS_INIT_STAND[28] = { 
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};

    // Initial state from cassie-mujoco-sim from full Cassie model
    const double QPOS_INIT[35] = {0, 0, 1.01, 1, 0, 0, 0,
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
};

// Initialise Cassie state
int CassieSimulator::ResetModel() {

    // Number of states
    int nq = m_->nq;

    // Check if fixed to stand or not
    const double* qpos0;
    if (nq == 28) {
        qpos0 = QPOS_INIT_STAND;
    } else {
        qpos0 = QPOS_INIT;
    }

    // Reset data and initialise
    mj_resetData(m_, d_);
    mju_copy(d_->qpos, qpos0, nq);

    return 0;
}

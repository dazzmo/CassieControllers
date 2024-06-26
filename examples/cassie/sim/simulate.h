#pragma once

#include <chrono>
#include <iostream>

#include <glog/logging.h>
#include <mujoco/mujoco.h>

#include "mujoco_sim.h"
#include "controller/osc.h"

const double SIM_REFERESH_RATE = 1.0 / 60.0;

// Initial state from cassie-mujoco-sim from full Cassie model
const double QPOS_INIT[35] = {0, 0, 1.01, 1, 0, 0, 0,
    0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
    -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};

// Get real time as a double
double real_time_seconds() {
    auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    auto time_micro = tmp.count();
    return time_micro / 1e6;
}

// Initialise Cassie state
int init_cassie_model(MujocoSimulator& sim) {

    // Number of states and initial pose
    int nq = sim.m_->nq;
    const double* qpos0 = QPOS_INIT;

    // Reset data and initialise
    mj_resetData(sim.m_, sim.d_);
    mju_copy(sim.d_->qpos, qpos0, nq);

    return 0;
}

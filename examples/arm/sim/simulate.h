#ifndef SIM_SIMULATE_H
#define SIM_SIMULATE_H

#pragma once

#include <chrono>
#include <iostream>

#include <glog/logging.h>
#include <mujoco/mujoco.h>

#include "mujoco_sim.h"

const double SIM_REFERESH_RATE = 1.0 / 60.0;

// Get real time as a double
double real_time_seconds() {
    auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    auto time_micro = tmp.count();
    return time_micro / 1e6;
}

#endif/* SIM_SIMULATE_H */

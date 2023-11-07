#include <glog/logging.h>

#include <chrono>
#include <cstdio>
#include <cstring>

#include "controller/osc.h"
#include "mujoco_sim.h"

// Useful for tracking real-time visuals
double time_in_seconds() {
    auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    auto time_micro = tmp.count();
    return time_micro / 1000000.0;
}

int main(int argc, const char** argv) {
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator and load model
    MujocoSimulator& sim = MujocoSimulator::getInstance();

    sim.LoadModel("./scene.xml");
    sim.Init();

    // Create controller
    double freq = 100.0;
    // Create OSC model
    ArmModel arm;
    // Create OSC controller for arm model
    controller::osc::Options opt;
    opt.include_constraint_forces = true;
    controller::osc::OperationalSpaceController c(arm);
    c.CreateOSC(opt);
    c.SetTorqueWeight(1e-6);

    // Simulate the model
    mjtNum t_ctrl = sim.GetSimulatorTime();
    while (!sim.WindowShouldClose()) {
        // Run simulator at a reasonable frame rate in real time
        // double simstart = time_in_seconds();
        double simstart = sim.GetSimulatorTime();
        // while (time_in_seconds() - simstart < 1.0 / 60.0) {

        while (sim.GetSimulatorTime() - simstart < 1.0 / 60.0) {
            // Apply control at desired frequency within simulator
            if (sim.GetSimulatorTime() - t_ctrl > 1.0 / freq) {
                // Update model state
                c.GetModel().UpdateState(arm.size().nq, sim.GetModelConfiguration(),
                                         arm.size().nv, sim.GetModelVelocity());
                // Compute control based on updated model state
                c.UpdateControl(sim.GetSimulatorTime(),
                                c.GetModel().state().q, c.GetModel().state().v);

                t_ctrl = sim.GetSimulatorTime();
            }

            LOG(INFO) << "u: " << c.ControlOutput().transpose();
            sim.ApplyControl(c.ControlOutput().data(), c.GetModel().size().nu);
            sim.ForwardStep();
        }
        sim.UpdateSceneAndRender();
    }

    return 0;
}
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


    // Construct controller (frequency in Hz)
    const double freq = 2e2;
    ArmOSC* c = new ArmOSC();
    if (c->Init() != ControllerStatus::SUCCESS) {
        mju_error("Could not initialise controller");
    }
    c->SetControlFrequency(freq);


    // Simulate the model
    mjtNum t_ctrl = sim.GetSimulatorTime();
    while (!sim.WindowShouldClose()) {

        // Run simulator at a reasonable frame rate in real time
        double simstart = time_in_seconds();
        while (time_in_seconds() - simstart < 1.0 / 60.0) {

            // Apply control at desired frequency within simulator
            if (sim.GetSimulatorTime() - t_ctrl > 1.0 / freq) {

                // Store physics state in controller
                c->MapMujocoState(sim.GetModelConfiguration(), sim.GetModelVelocity());

                // Print to terminal

                std::cout << "qacc (sim): ";
                for (int i = 0; i < sim.GetModelNv(); i++) {
                    std::cout << *(sim.GetModelAcceleration() + i) << '\t';
                }
                std::cout << '\n';
                // std::cout << "u: " << c->ctrl().transpose() << '\n';
                sim.PrintDynamicsCoefficients();

                // Update controller params
                c->Update(sim.GetSimulatorTime());
                t_ctrl = sim.GetSimulatorTime();
            }

            sim.ApplyControl(c->ctrl().data(), c->nu());
            sim.ForwardStep();
        }
        sim.UpdateSceneAndRender();
    }

    return 0;
}
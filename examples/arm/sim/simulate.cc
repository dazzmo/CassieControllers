#include <glog/logging.h>

#include <cstdio>
#include <cstring>

#include "controller/osc.h"
#include "mujoco_sim.h"

int main(int argc, const char** argv) {
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator
    MujocoSimulator& sim = MujocoSimulator::getInstance();

    sim.LoadModel("./scene.xml");
    sim.Init();

    // Create controller
    ArmOSC c();

    mjtNum t_ctrl = sim.GetSimulatorTime();
    while (!sim.WindowShouldClose()) {
        mjtNum simstart = sim.GetSimulatorTime();
        while (sim.GetSimulatorTime() - simstart < 1.0 / 60.0) {
            // Apply control at desired frequency
            if (sim.GetSimulatorTime() - t_ctrl > 1.0 / freq) {
                // c->MapMujocoState(sim.GetModelConfiguration(), sim.GetModelVelocity());
                std::cout << "qacc (sim): ";
                for (int i = 0; i < sim.GetModelNv(); i++) {
                    std::cout << *(sim.GetModelAcceleration() + i) << '\t';
                }
                std::cout << '\n';
                // std::cout << "u: " << c->ctrl().transpose() << '\n';
                sim.PrintDynamicsCoefficients();
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
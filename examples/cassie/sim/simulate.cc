#include <cstdio>
#include <cstring>

#include "mujoco_sim.h"
#include "controller/osc_leg.h"

int main(int argc, const char** argv) {
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator
    MujocoSimulator &sim = MujocoSimulator::getInstance();

    sim.LoadModel("./agility_cassie/scene.xml");
    sim.Init();

    // make controller
    const double freq = 2e2;
    CassieLegOSC* c = new CassieLegOSC();
    if(c->Init() != ControllerStatus::SUCCESS) {
        mju_error("Could not initialise controller");
    }
    c->SetControlFrequency(freq);

    mjtNum t_ctrl = sim.GetSimulatorTime();
    while (!sim.WindowShouldClose()) {

        mjtNum simstart = sim.GetSimulatorTime();
        while (sim.GetSimulatorTime() - simstart < 1.0 / 60.0) {
            
            // Apply control at desired frequency
            if (sim.GetSimulatorTime() - t_ctrl > 1.0 / freq) {
                c->MapMujocoState(sim.GetModelConfiguration(), sim.GetModelVelocity());
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
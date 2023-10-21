#include <cstdio>
#include <cstring>

#include "mujoco_sim.h"
#include "controller/os_ctrl_leg.h"

int main(int argc, const char** argv) {
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;
    // google::SetLogDestination(google::INFO, "./");

    // Create simulator
    MujocoSimulator &sim = MujocoSimulator::getInstance();

    sim.LoadModel("./agility_cassie/scene.xml");
    sim.Init();

    // make controller
    const double freq = 1e3;
    CassieLegOSC* c = new CassieLegOSC();
    if(c->Init(8, 8, 5) != ControllerStatus::SUCCESS) {
        mju_error("Could not initialise controller");
    }
    c->SetControlFrequency(freq);

    Eigen::Vector3d r(0.1, 0.2, -0.8);
    mjtNum t_ctrl = sim.GetSimulatorTime();
    while (!sim.WindowShouldClose()) {

        mjtNum simstart = sim.GetSimulatorTime();
        while (sim.GetSimulatorTime() - simstart < 1.0 / 60.0) {
            
            // Apply control at desired frequency
            if (sim.GetSimulatorTime() - t_ctrl > 1.0 / freq) {
                c->MapMujocoState(sim.GetModelConfiguration(), sim.GetModelVelocity());
                c->GetEndEffectorTaskMap()["ankle"]->SetReference(r);
                // c->Update(sim.GetSimulatorTime());
                t_ctrl = sim.GetSimulatorTime();
            }

            // sim.ApplyControl(c->ctrl().data(), c->nu());            
            sim.ForwardStep();
        }
        sim.UpdateSceneAndRender();
    }

    return 0;
}
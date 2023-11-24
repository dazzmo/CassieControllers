#include "simulate.h"

int main() {
    
    
    // Create simulator and load model
    CassieSimulator& sim = CassieSimulator::getInstance(); // TODO: This line broken :(
    sim.LoadModel("./agility_cassie/scene.xml");
    sim.Init();

    // Set the initial pose for Cassie
    sim.ResetModel();

    // Simulate the model
    double real_start;
    double sim_start;
    while (!sim.WindowShouldClose()) {

        // Render at 60 Hz and make sure simulator and real-time are in sync
        real_start = real_time_seconds();
        sim_start = sim.GetSimulatorTime();
        while (real_time_seconds() - real_start < SIM_REFERESH_RATE) {
            if (sim.GetSimulatorTime() - sim_start < SIM_REFERESH_RATE) {
                sim.ForwardStep();
            } 
        }
        sim.UpdateSceneAndRender();
    } 

    return 0;
}
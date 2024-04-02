#include "simulate_fixed.h"

int main() {
    // Initialise logging
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator and load model
    MujocoSimulator& sim = MujocoSimulator::getInstance();
    sim.LoadModel("./agility_cassie/scene_fixed.xml");
    sim.Init();

    // Set controller options
    double frequency = 1000.0;

    // Create an operational space controller model for Cassie
    CassieFixedOSC cassie_osc;

    // TODO Here we could set some parameters such as cost weightings and whatnot

    // Set the initial pose for Cassie
    init_cassie_model(sim);

    // Simulate the model
    double real_start;
    double sim_start;
    double t_ctrl = sim.GetSimulatorTime();

    while (!sim.WindowShouldClose()) {
        // Render at 60 Hz and make sure simulator and real-time are in sync
        real_start = real_time_seconds();
        sim_start = sim.GetSimulatorTime();
        while (real_time_seconds() - real_start < SIM_REFERESH_RATE) {
            if (sim.GetSimulatorTime() - sim_start < SIM_REFERESH_RATE) {
                // Apply controller at desired frequency within simulator
                if (sim.GetSimulatorTime() - t_ctrl > 1.0 / frequency) {
                    // Update model state
                    cassie_osc.UpdateState(
                        CASSIE_FIXED_NQ, sim.GetModelConfiguration(),
                        CASSIE_FIXED_NV, sim.GetModelVelocity());

                    // Update references
                    cassie_osc.UpdateReferences(sim.GetSimulatorTime());
                    // Compute controls
                    cassie_osc.Solve();
                    // Update timer and log
                    t_ctrl = sim.GetSimulatorTime();
                }

                // Apply controls and step forward
                sim.ApplyControl(cassie_osc.CurrentControlSolution().data(),
                                 CASSIE_FIXED_NU);
                sim.ForwardStep();
            }
        }
        sim.UpdateSceneAndRender();
    }

    return 0;
}
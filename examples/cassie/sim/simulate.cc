#include <glog/logging.h>

#include <chrono>
#include <cstdio>
#include <cstring>

#include "controller/osc_leg.h"
#include "controllers/osc/osc.h"
#include "mujoco_sim.h"

// Useful for tracking real-time visuals
double RealTimeSeconds() {
    auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    auto time_micro = tmp.count();
    return time_micro / 1e6;
}

int main(int argc, const char** argv) {
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator and load model
    MujocoSimulator& sim = MujocoSimulator::getInstance();

    sim.LoadModel("./agility_cassie/scene.xml");
    sim.Init();

    // Create OSC model
    CassieLegOSC leg;
    
    // Create OSC controller for leg model
    controller::osc::Options opt;
    opt.frequency = 500.0;
    opt.qpoases_print_level = qpOASES::PrintLevel::PL_NONE;
    controller::osc::OperationalSpaceController c(leg, opt);
    c.Init();
    c.SetControlWeighting(Eigen::Vector<double, 5>(1e-6, 1e-6, 1e-6, 1e-6, 1e-6));

    // Simulate the model
    double t_ctrl = sim.GetSimulatorTime();

    double sim_time = 0.0, real_time = 0.0;
    // c.StartRamp(0.0, 5e-1, OutputPrescale::RampType::RAMP_UP);

    while (!sim.WindowShouldClose()) {
        // Run simulator at a reasonable frame rate in real time
        double sim_start = sim.GetSimulatorTime();
        double real_start = RealTimeSeconds();
        while (RealTimeSeconds() - real_start < 1.0 / 60.0) {
            // Compute simulation up to next frame in real-time
            if (sim.GetSimulatorTime() - sim_start < 1.0 / 60.0) {
                // Apply control at desired frequency within simulator
                if (sim.GetSimulatorTime() - t_ctrl > 1.0 / opt.frequency) {
                    // Update model state
                    c.GetModel().UpdateState(leg.size().nq, sim.GetModelConfiguration(),
                                             leg.size().nv, sim.GetModelVelocity());
                    // Compute control based on updated model state
                    c.UpdateControl(sim.GetSimulatorTime(),
                                    c.GetModel().state().q,
                                    c.GetModel().state().v);

                    t_ctrl = sim.GetSimulatorTime();
                    LOG(INFO) << "u: " << c.ControlOutput().transpose();
                }
                // Zero-order hold on control signal
                sim.ApplyControl(c.ControlOutput().data(), c.GetModel().size().nu);
                sim.ForwardStep();
            }
        }
        sim.UpdateSceneAndRender();
    }

    return 0;
}
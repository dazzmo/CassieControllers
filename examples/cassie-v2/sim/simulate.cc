#include "simulate.h"

int main() {

    // Initialise logging
    google::InitGoogleLogging("simulate");
    FLAGS_logtostderr = 1;

    // Create simulator and load model
    MujocoSimulator& sim = MujocoSimulator::getInstance();
    sim.LoadModel("./agility_cassie/scene.xml");
    sim.Init();

    // Set controller options
    controller::osc::Options opt;
    opt.frequency = 500.0;
    opt.qpoases_print_level = qpOASES::PrintLevel::PL_NONE;

    // Create an operational space controller model for Cassie leg
    CassieLegOSC leg_ctrl;
    controller::osc::OperationalSpaceController ctrl(leg_ctrl, opt);
    ctrl.Init();
    ctrl.SetControlWeighting(Eigen::Vector<double, 5>(1e-6, 1e-6, 1e-6, 1e-6, 1e-6));

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
                if (sim.GetSimulatorTime() - t_ctrl > 1.0 / opt.frequency) {

                    // Update model state
                    ctrl.GetModel().UpdateState(leg_ctrl.size().nq, sim.GetModelConfiguration(),
                                                leg_ctrl.size().nv, sim.GetModelVelocity());

                    // Compute controls
                    ctrl.UpdateControl(sim.GetSimulatorTime(), 
                                       ctrl.GetModel().state().q,
                                       ctrl.GetModel().state().v);

                    // Update timer and log
                    t_ctrl = sim.GetSimulatorTime();
                    LOG(INFO) << "u: " << ctrl.ControlOutput().transpose();
                }

                // Apply controls and step forward
                sim.ApplyControl(ctrl.ControlOutput().data(), ctrl.GetModel().size().nu);
                sim.ForwardStep();
            } 
        }
        sim.UpdateSceneAndRender();
    } 

    return 0;
}
#include "controllers/controller.h"

using namespace controller;

Controller::Controller(const DynamicModel &m) : d_() {
    m_ = const_cast<DynamicModel *>(&m);
    LOG(INFO) << "Controller constructor";
}

ReturnStatus Controller::Init() {
    // Starting time
    SetCurrentTime(0.0);

    // Setup program (user-defined)
    SetupController();

    return ReturnStatus::SUCCESS;
}

void Controller::Update(double t) {
    SetCurrentTime(t);
    if (UpdateControl() != ReturnStatus::SUCCESS) {
    };
}

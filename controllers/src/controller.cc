#include "controllers/controller.h"

using namespace controller;

Controller::Controller(Dimension nu) : d_() {
    LOG(INFO) << "Controller constructor";
    u_ = ActuationVector::Zero(nu);
}
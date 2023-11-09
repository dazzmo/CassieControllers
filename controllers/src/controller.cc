#include "controllers/controller.h"

using namespace controller;

Controller::Controller(Dimension nu, const Options& opt) {
    LOG(INFO) << "Controller constructor";
    u_ = ActuationVector::Zero(nu);
}
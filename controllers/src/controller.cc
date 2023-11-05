#include "controllers/controller.h"

using namespace controller;

Controller::Controller() : d_() {
    LOG(INFO) << "Controller constructor";
}

void Controller::AddModel(const DynamicModel &m) {
    m_ = const_cast<DynamicModel *>(&m);
}
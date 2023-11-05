#include "controllers/model.h"

using namespace controller;

void DynamicModel::UpdateModel(const ConfigurationVector &q, const TangentVector &v) {
    ComputeMassMatrix(q, dynamics_.M);
    ComputeBiasVector(q, v, dynamics_.h);
    ComputeActuationMap(q, dynamics_.B);
}
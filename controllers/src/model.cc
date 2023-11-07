#include "controllers/model.h"

using namespace controller;

/**
 * @brief Updates the model state from measured configuration and velocity vectors
 * q and v.
 *
 * @param nq
 * @param q
 * @param nv
 * @param v
 */
void DynamicModel::UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v) {
    // Naive copy
    memcpy(state_.q.data(), q, sizeof(Scalar) * nq);
    memcpy(state_.v.data(), v, sizeof(Scalar) * nv);
}

/**
 * @brief Updates the dynamic quantities of the model.
 * 
 * @param q 
 * @param v 
 */
void DynamicModel::UpdateModel(const ConfigurationVector &q, const TangentVector &v) {
    ComputeMassMatrix(q, dynamics_.M);
    ComputeBiasVector(q, v, dynamics_.h);
    ComputeActuationMap(q, dynamics_.B);
}
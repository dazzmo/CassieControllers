#include "controllers/model.h"

void DynamicModel::AddSite(const std::string &name) {
    sites_[name] = new Site(sz_);
}

void DynamicModel::RemoveSite(const std::string &name) {
    sites_.erase(name);
}

void DynamicModel::UpdateModel(const ConfigurationVector &q, const TangentVector &v) {
    // Compute  sites
    for (auto c : sites_) {
        ComputeSite(c.first, q, v, *c.second);
    }

    ComputeMassMatrix(q, M_);
    ComputeBiasVector(q, v, h_);
    ComputeActuationMap(q, B_);
}
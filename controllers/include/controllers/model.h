#ifndef INCLUDE_CONTROLLERS_MODEL_HPP
#define INCLUDE_CONTROLLERS_MODEL_HPP

#include <map>
#include <memory>

#include "controllers/types.h"
#include "eigen3/Eigen/Core"

namespace controller {

class DynamicModel {
   public:
    struct Size {
        Size(Dimension nq, Dimension nv, Dimension nu) {
            assert(nq > 0 && "nq is not greater than 0");
            assert(nv > 0 && "nv is not greater than 0");
            assert(nu >= 0 && "nu is negative");

            this->nq = nq;
            this->nv = nv;
            this->nu = nu;
        }
        // Dimension of configuration vector
        Dimension nq;
        // Dimension of configuration tangent vector
        Dimension nv;
        // Number of actuators
        Dimension nu;
    };

    struct State {
        State(const Size &sz) {
            q = ConfigurationVector::Zero(sz.nq);
            v = TangentVector::Zero(sz.nv);
            a = TangentVector::Zero(sz.nv);
        }
        ConfigurationVector q;
        TangentVector v;
        TangentVector a;
    };

    struct Bounds {
        Bounds(const Size &sz) {
            qmin = ConfigurationVector::Zero(sz.nq);
            qmax = ConfigurationVector::Zero(sz.nq);
            vmax = TangentVector::Zero(sz.nv);
            amax = TangentVector::Zero(sz.nv);
            umax = ActuationVector::Zero(sz.nu);
        }
        ConfigurationVector qmin;
        ConfigurationVector qmax;
        TangentVector vmax;
        TangentVector amax;
        ActuationVector umax;
    };

    struct Dynamics {
        Dynamics(const Size &sz) {
            M = Matrix::Zero(sz.nv, sz.nv);
            h = Vector::Zero(sz.nv);
            B = Matrix::Zero(sz.nv, sz.nu);
        }
        Matrix M;
        Vector h;
        Matrix B;
    };

    DynamicModel(const Size &sz) : sz_(sz),
                                   state_(sz),
                                   initial_state_(sz),
                                   bounds_(sz),
                                   dynamics_(sz) {
    }

    ~DynamicModel() {}

    virtual void UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v);
    void UpdateModel(const ConfigurationVector &q, const TangentVector &v);

    const Size &size() const { return sz_; }
    Bounds &bounds() { return bounds_; }
    State &state() { return state_; }
    State &initial_state() { return initial_state_; }
    const Dynamics &dynamics() const { return dynamics_; }

   protected:
    virtual void ComputeMassMatrix(const ConfigurationVector &q, Matrix &M) = 0;
    virtual void ComputeBiasVector(const ConfigurationVector &q, const TangentVector &v, Vector &h) = 0;
    virtual void ComputeActuationMap(const ConfigurationVector &q, Matrix &B) = 0;

   private:
    // Size of dynamic model
    Size sz_;

    State state_;
    State initial_state_;
    Bounds bounds_;
    Dynamics dynamics_;
};

}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_MODEL_HPP */

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

    struct Control {
        Control(const Size &sz) {
            u = ActuationVector::Zero(sz.nu);
        }
        ActuationVector u;
    };

    struct Bounds {
        Bounds(const Size &sz) {
            ql = ConfigurationVector::Zero(sz.nq);
            qu = ConfigurationVector::Zero(sz.nq);
            vmax = TangentVector::Zero(sz.nv);
            amax = TangentVector::Zero(sz.nv);
        }
        ConfigurationVector ql;
        ConfigurationVector qu;
        TangentVector vmax;
        TangentVector amax;
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
                                   state_init_(sz),
                                   ctrl_(sz),
                                   bounds_(sz),
                                   dynamics_(sz) {
    }

    ~DynamicModel() {}


    void UpdateModel(const ConfigurationVector &q, const TangentVector &v);

    const Size &size() const { return sz_; }
    const Bounds &bounds() const { return bounds_; }
    const State &state() const { return state_; }
    const Control &ctrl() const { return ctrl_; }

   protected:
    virtual void ComputeMassMatrix(const ConfigurationVector &q, Matrix &M) = 0;
    virtual void ComputeBiasVector(const ConfigurationVector &q, const TangentVector &v, Vector &h) = 0;
    virtual void ComputeActuationMap(const ConfigurationVector &q, Matrix &B) = 0;

   private:
    // Size of dynamic model
    Size sz_;
    // Number of contact points
    Dimension nc_;
    // Number of constraints
    Dimension ng_;

    State state_;
    State state_init_;
    Control ctrl_;
    Bounds bounds_;
    Dynamics dynamics_;

};

}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_MODEL_HPP */

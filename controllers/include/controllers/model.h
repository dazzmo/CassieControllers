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

    struct Site {
        Site(const Size &sz) {
            x = Vector3::Zero();
            J = Matrix::Zero(3, sz.nv);
            dJdq = Vector3::Zero();
            lambda = Vector3::Zero();
        }

        Vector3 x;
        Matrix J;
        Vector3 dJdq;
        Vector3 lambda;
    };

    struct Constraint {
        Constraint(Dimension n, const Size &sz) {
            c = Vector::Zero(n);
            J = Matrix::Zero(n, sz.nv);
            dJdq = Vector::Zero(n);
            lambda = Vector::Zero(n);
        }

        Vector c;
        Matrix J;
        Vector dJdq;
        Vector lambda;
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

    ~DynamicModel() {
        for (auto &c : sites_) {
            // Delete all contact points
            c.second.release();
        }
    }

    void AddSite(const std::string &name);
    void RemoveSite(const std::string &name);

    void UpdateModel(const ConfigurationVector &q, const TangentVector &v);

    const Size &size() const { return sz_; }
    const State &state() const { return state_; }
    const Control &ctrl() const { return ctrl_; }
    const Constraint &GetConstraint(const std::string &name) { return *constraints_[name]; }
    const Site &GetSite(const std::string &name) { return *sites_[name]; }

   protected:
    virtual void ComputeMassMatrix(const ConfigurationVector &q, Matrix &M) = 0;
    virtual void ComputeBiasVector(const ConfigurationVector &q, const TangentVector &v, Vector &h) = 0;
    virtual void ComputeActuationMap(const ConfigurationVector &q, Matrix &B) = 0;
    virtual void ComputeSite(const std::string &name,
                             const ConfigurationVector &q, const ConfigurationVector &v,
                             Site &site) = 0;
    virtual void ComputeConstraint(const std::string &name,
                                   const ConfigurationVector &q, const ConfigurationVector &v,
                                   Constraint &constraint) = 0;

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

    std::map<std::string, std::unique_ptr<Site>> sites_;
    std::map<std::string, std::unique_ptr<Constraint>> constraints_;
};

}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_MODEL_HPP */

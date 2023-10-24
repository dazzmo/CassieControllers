#include "controller/osc_leg.h"

int main(void) {
    CassieLegOSC* c = new CassieLegOSC();
    c->Init();

    c->qpos() << 0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681;
    Eigen::VectorXd q_new(c->nq());
    c->InverseKinematics(q_new, Eigen::Vector3d(0.1, 0.2, -0.8), c->qpos());
    return 0;
}
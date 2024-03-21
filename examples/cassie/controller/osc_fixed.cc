#include "osc_fixed.h"

/**
 * @brief Remaps the measured state to the state expected by our model
 *
 * @param nq
 * @param q
 * @param nv
 * @param v
 */
void CassieFixedOSC::UpdateState(int nq, const double *q, int nv,
                                 const double *v) {
    Eigen::VectorXd qpos(CASSIE_FIXED_NQ), qvel(CASSIE_FIXED_NV);

    // Map Mujoco data to appropriate joints
    qpos << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13], q[14], q[15],
        q[16], q[21], q[22], q[23], q[24], q[27];
    qvel << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12], v[13], v[14],
        v[15], v[19], v[20], v[21], v[22], v[25];

    // Update parameter vectors in OSC
    osc_.SetParameter("qpos", qpos);
    osc_.SetParameter("qvel", qvel);
}
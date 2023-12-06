#include "osc_fixed.h"

/**
 * @brief Remaps the measured state to the state expected by our model
 * TODO: This will need to be updated when we move to the full Cassie model (leg no longer at position 0)
 * 
 * @param nq 
 * @param q 
 * @param nv 
 * @param v 
 */
void CassieFixedOSC::UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v) {
    state().q << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13],
                 q[14], q[15], q[16], q[21], q[22], q[23], q[24], q[27];

    state().v << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12],
                 v[13], v[14], v[15], v[19], v[20], v[21], v[22], v[25];
}
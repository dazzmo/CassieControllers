// #include "osc.h"

// /**
//  * @brief Remaps the measured state to the state expected by our model
//  * 
//  * @param nq 
//  * @param q 
//  * @param nv 
//  * @param v 
//  */
// void CassieOSC::UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v) {
//     // Pinocchio currently has quaternions as q = [x, y, z, w]
//     state().q << q[0], q[1], q[2], q[4], q[5], q[6], q[3],
//                  q[7], q[8], q[9], q[14], q[15], q[16], q[17], q[20],
//                  q[21], q[22], q[23], q[28], q[29], q[30], q[31], q[34];

//     state().v << v[0], v[1], v[2], v[3], v[4], v[5],
//                  v[6], v[7], v[8], v[12], v[13], v[14], v[15], v[18],
//                  v[19], v[20], v[21], v[25], v[26], v[27], v[28], v[31];
// }
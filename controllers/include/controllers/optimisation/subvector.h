#ifndef CONTROLLERS_OPTIMISATION_SUBVECTOR_HPP
#define CONTROLLERS_OPTIMISATION_SUBVECTOR_HPP

#include "controllers/types.h"

namespace controller {
namespace optimisation {

struct SubVector {
    SubVector(Index start, Dimension sz) {
        assert(start >= 0 && "Starting index is negative!");
        assert(sz >= 0 && "Size is negative!");

        this->start = start;
        this->sz = sz;
        vec = Vector::Zero(sz);
    }

    SubVector() {
        this->start = 0;
        this->sz = 0;
        vec = Vector::Zero(0);
    }

    /**
     * @brief Inserts this variable directly after v in the overall vector
     *
     * @param v
     */
    void InsertAfter(const SubVector& v) {
        this->start = v.start + v.sz;
    }

    Index start;
    Dimension sz;
    Vector vec;
};

}  // namespace optimisation
}  // namespace controller
#endif /* CONTROLLERS_OPTIMISATION_SUBVECTOR_HPP */

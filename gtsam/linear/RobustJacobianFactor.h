//
// Created by Fan Jiang on 2/26/20.
//

#ifndef GTSAM_ROBUSTJACOBIANFACTOR_H
#define GTSAM_ROBUSTJACOBIANFACTOR_H

#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {
  class RobustJacobianFactor : public JacobianFactor {

  protected:
    double scale_;

  public:
    /** Copy constructor */
    RobustJacobianFactor(const RobustJacobianFactor& jf) : JacobianFactor(jf), scale_(jf.scale_) {}

    /** Construct an n-ary factor
     * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
     *         collection of keys and matrices making up the factor. */
    template<typename TERMS>
    RobustJacobianFactor(const TERMS &terms, const Vector &b, const double scale = 1.0,
                         const SharedDiagonal &model = SharedDiagonal()) :
            JacobianFactor(terms, b, model) {
      scale_ = scale;
//      std::cout << "Scale: " << scale_ << std::endl;
    }

    /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
     *  instead of in block terms.  Note that only the active view of the provided augmented matrix
     *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
     *  factor. */
    template<typename KEYS>
    RobustJacobianFactor(
            const KEYS &keys, const VerticalBlockMatrix &augmentedMatrix, const double scale = 1.0,
            const SharedDiagonal &sigmas = SharedDiagonal()) : JacobianFactor(keys, augmentedMatrix, sigmas) {
      scale_ = scale;
    }

    Vector error_vector(const VectorValues& c) const override; /** (A*x-b)/sigma */
    double error(const VectorValues& c) const override; /**  0.5*(A*x-b)'*D*(A*x-b) */

  };
}

#endif //GTSAM_ROBUSTJACOBIANFACTOR_H

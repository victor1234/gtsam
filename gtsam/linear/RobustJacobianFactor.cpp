//
// Created by Fan Jiang on 2/26/20.
//

#include <gtsam/linear/RobustJacobianFactor.h>

namespace gtsam {

  Vector RobustJacobianFactor::error_vector(const VectorValues &c) const {
    return JacobianFactor::error_vector(c);
  }

  double RobustJacobianFactor::error(const VectorValues& c) const {
    Vector weighted = error_vector(c);
    return 0.5 * weighted.dot(weighted);
  }
}
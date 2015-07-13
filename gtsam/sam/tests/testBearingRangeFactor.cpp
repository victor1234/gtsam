/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBearingRangeFactor.cpp
 *  @brief Unit tests for BearingRangeFactor Class
 *  @author Frank Dellaert
 *  @date July 2015
 */

#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace serializationTestHelpers;

Key poseKey(1);
Key pointKey(2);

typedef BearingRangeFactor<Pose2, Point2> BearingRangeFactor2D;
BearingRangeFactor2D::T measurement2D(1, 2);
static SharedNoiseModel model2D(noiseModel::Isotropic::Sigma(2, 0.5));
BearingRangeFactor2D factor2D(poseKey, pointKey, 1, 2, model2D);

typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;
BearingRangeFactor3D::T measurement3D(Pose3().bearing(Point3(1, 0, 0)), 0);
static SharedNoiseModel model3D(noiseModel::Isotropic::Sigma(3, 0.5));
BearingRangeFactor3D factor3D(poseKey, pointKey,
                              Pose3().bearing(Point3(1, 0, 0)), 1, model3D);

/* ************************************************************************* */
// Export Noisemodels
// See http://www.boost.org/doc/libs/1_32_0/libs/serialization/doc/special.html
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic);

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization2D) {
  EXPECT(equalsObj<BearingRangeFactor2D>(factor2D));
  EXPECT(equalsXML<BearingRangeFactor2D>(factor2D));
  EXPECT(equalsBinary<BearingRangeFactor2D>(factor2D));
}

/* ************************************************************************* */
TEST(BearingRangeFactor, 2D) {
  // Serialize the factor
  std::string serialized = serializeXML(factor2D);

  // And de-serialize it
  BearingRangeFactor2D factor;
  deserializeXML(serialized, factor);

  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey, Point2(-4.0, 11.0));

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor.expression(poseKey, pointKey),
                                      values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization3D) {
  EXPECT(equalsObj<BearingRangeFactor3D>(factor3D));
  EXPECT(equalsXML<BearingRangeFactor3D>(factor3D));
  EXPECT(equalsBinary<BearingRangeFactor3D>(factor3D));
}

/* ************************************************************************* */
TEST(BearingRangeFactor, 3D) {
  // Serialize the factor
  std::string serialized = serializeXML(factor3D);

  // And de-serialize it
  BearingRangeFactor3D factor;
  deserializeXML(serialized, factor);

  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose3());
  values.insert(pointKey, Point3(1, 0, 0));

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor.expression(poseKey, pointKey),
                                      values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

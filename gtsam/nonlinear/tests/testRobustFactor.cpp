//
// Created by Fan Jiang on 2/26/20.
//

#include <gtsam/linear/RobustJacobianFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/range/adaptor/map.hpp>
#include <boost/shared_ptr.hpp>


using namespace std;
using namespace gtsam;


/* ************************************************************************* */
//TEST(RobustJacobianFactor, MoreOptimizationWithHuber) {
//
//  NonlinearFactorGraph fg;
//  if (true) {
//    fg += PriorFactor<Pose2>(0, Pose2(0, 0, 0), noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(2),
//                                                                           noiseModel::Isotropic::Sigma(3, 1)));
//    fg += BetweenFactor<Pose2>(0, 1, Pose2(10, 0, 0),
//                               noiseModel::Isotropic::Sigma(3, 0.1));
//    fg += PriorFactor<Pose2>(1, Pose2(7, 0, 0), noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(2),
//                                                                           noiseModel::Isotropic::Sigma(3, 1)));
//  } else {
//    fg += PriorFactor<Pose2>(0, Pose2(0, 0, 0), noiseModel::Isotropic::Sigma(3, 1));
//    fg += BetweenFactor<Pose2>(0, 1, Pose2(1, 0, 0),
//                               noiseModel::Isotropic::Sigma(3, 1));
//    fg += PriorFactor<Pose2>(1, Pose2(0.8, 0, 0), noiseModel::Isotropic::Sigma(3, 1));
//  }
//
//  Values init;
//  init.insert(0, Pose2(1, 0, 0));
//  init.insert(1, Pose2(1, 0, 0));
//
//  Values expected;
//  expected.insert(0, Pose2(0, 0, 0));
//  expected.insert(1, Pose2(10, 0, 0));
//
//  LevenbergMarquardtParams params;
//  // params.setVerbosityLM("SUMMARY");
//  gtsam::NonlinearConjugateGradientOptimizer::Parameters cg_params;
//  cg_params.setErrorTol(0);
//  cg_params.setMaxIterations(10000);
//  cg_params.setRelativeErrorTol(0);
//  cg_params.setAbsoluteErrorTol(0);
//  // cg_params.setVerbosity("ERROR");
//  auto cg_result = NonlinearConjugateGradientOptimizer(fg, init, cg_params).optimize();
//  cg_result.print("CG: ");
//  cout << fg.error(cg_result) << endl;
//
//  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
//  gn_result.print("GN: ");
//  cout << fg.error(gn_result) << endl;
//
//  auto lm_result = LevenbergMarquardtOptimizer(fg, init, params).optimize();
//  lm_result.print("LM: ");
//  cout << fg.error(lm_result) << endl;
//
//  auto dl_result = DoglegOptimizer(fg, init).optimize();
//  dl_result.print("DL: ");
//  cout << fg.error(dl_result) << endl;
////  EXPECT(assert_equal(expected, GaussNewtonOptimizer(fg, init).optimize())
////  );
////  EXPECT(assert_equal(expected, LevenbergMarquardtOptimizer(fg, init, params).optimize())
////  );
////  EXPECT(assert_equal(expected, DoglegOptimizer(fg, init).optimize())
////  );
//}

/* ************************************************************************* */
TEST(RobustJacobianFactor, RobustMeanCalculation) {

  NonlinearFactorGraph fg;

  Values init;

  Values expected;

  auto huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(20),
                                          noiseModel::Isotropic::Sigma(1, 1));

  vector<double> pts{-10,-3,-1,1,3,10,10000};
  for(auto pt : pts) {
    fg += PriorFactor<double>(0, pt, huber);
  }

  init.insert(0, 100.0);
  expected.insert(0, 0.0);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("TRYLAMBDA");
  params.setAbsoluteErrorTol(1e-20);
  params.setRelativeErrorTol(1e-20);

  gtsam::NonlinearConjugateGradientOptimizer::Parameters cg_params;
  cg_params.setErrorTol(0);
  cg_params.setMaxIterations(10000);
  cg_params.setRelativeErrorTol(0);
  cg_params.setAbsoluteErrorTol(0);
  cg_params.setVerbosity("ERROR");
  auto cg_result = NonlinearConjugateGradientOptimizer(fg, init, cg_params).optimize();
  cg_result.print("CG: ");
  cout << fg.error(cg_result) << endl << endl << endl;
//
//  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
//  gn_result.print("GN: ");
//  cout << fg.error(gn_result) << endl << endl << endl;

  auto lm_result = LevenbergMarquardtOptimizer(fg, init, params).optimize();
  lm_result.print("LM: ");
  cout << fg.error(lm_result) << endl << endl << endl;
//
//  auto dl_result = DoglegOptimizer(fg, init).optimize();
//  dl_result.print("DL: ");
//  cout << fg.error(dl_result) << endl << endl << endl;
//  EXPECT(assert_equal(expected, GaussNewtonOptimizer(fg, init).optimize())
//  );
//  EXPECT(assert_equal(expected, LevenbergMarquardtOptimizer(fg, init, params).optimize())
//  );
//  EXPECT(assert_equal(expected, DoglegOptimizer(fg, init).optimize())
//  );
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

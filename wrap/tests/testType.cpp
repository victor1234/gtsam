/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testType.cpp
 * @brief  unit test for parsing a fully qualified type
 * @author Frank Dellaert
 * @date   Nov 30, 2014
 **/

#include <wrap/Qualified.h>
#include <wrap/utilities.h>

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_push_back_actor.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace wrap;
using namespace BOOST_SPIRIT_CLASSIC_NS;

typedef rule<BOOST_SPIRIT_CLASSIC_NS::phrase_scanner_t> Rule;

//******************************************************************************
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct type_grammar: public grammar<type_grammar> {

  Qualified& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  type_grammar(Qualified& result) :
      result_(result) {
  }

/// Definition of type grammar
  template<typename ScannerT>
  struct definition {

    typedef rule<ScannerT> Rule;

    Rule basisType_p, keywords_p, eigenType_p, stlType_p, className_p,
        namepsace_p, namespace_del_p, qualified_p, type_p;

    definition(type_grammar const& self) {
      basisType_p = (str_p("string") | "bool" | "size_t" | "int" | "double"
          | "char" | "unsigned char")[assign_a(self.result_.name)];

      eigenType_p = (str_p("Vector") | "Matrix")[assign_a(self.result_.name)];

      keywords_p = (str_p("const") | "static" | "namespace" | "void"
          | basisType_p);

      //Rule for STL Containers (class names are lowercase)
      stlType_p = (str_p("vector") | "list");

      className_p = (lexeme_d[upper_p >> *(alnum_p | '_')] - eigenType_p
          - keywords_p) | stlType_p;

      namepsace_p = lexeme_d[lower_p >> *(alnum_p | '_')] - keywords_p;

      namespace_del_p = namepsace_p[push_back_a(self.result_.namespaces)]
          >> str_p("::");

      qualified_p = *namespace_del_p
          >> className_p[assign_a(self.result_.name)];

      type_p = basisType_p | eigenType_p;
    }

    Rule const& start() const {
      return type_p;
    }

  };
};

//******************************************************************************
TEST( spirit, grammar ) {
  // Create grammar that will place result in actual
  Qualified actual;
  type_grammar type_g(actual);

  // a basic type
  EXPECT(parse("double", type_g, space_p).full);
  EXPECT(actual.name=="double");

  // an Eigen type
  EXPECT(parse("Vector", type_g, space_p).full);
  EXPECT(actual.name=="Vector");
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

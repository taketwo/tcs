#ifndef FACTORY_WEIGHT_COMPUTER_FACTORY_H
#define FACTORY_WEIGHT_COMPUTER_FACTORY_H

#include <boost/any.hpp>

#include "factory.h"
#include "graph/weight.h"

namespace factory
{

template <typename WeightComputer>
class WeightComputerFactory : public Factory
{

public:

  WeightComputerFactory ()
  : Factory ("Weight Computer")
  , xyz_weight_ ("xyz weight", "-z", 3.0f)
  , normal_weight_ ("normal weight", "-n", 0.01f)
  , color_weight_ ("color weight", "-c", 3.0f)
  , curvature_weight_ ("curvature weight", "-b", 0.0001f)
  , weight_threshold_ ("weight threshold", "--weight-threshold", 0.00001f)
  {
    add (&xyz_weight_);
    add (&normal_weight_);
    add (&color_weight_);
    add (&curvature_weight_);
    add (&weight_threshold_);
  }

  WeightComputer
  instantiate (int argc, char** argv)
  {
    parse (argc, argv);
    using namespace pcl::graph::weight;
    return WeightComputer (tag::xyz::scale = static_cast<float> (xyz_weight_),
                           tag::normal::scale = static_cast<float> (normal_weight_),
                           tag::curvature::scale = static_cast<float> (curvature_weight_),
                           tag::color::scale = static_cast<float> (color_weight_),
                           tag::weight::threshold = static_cast<float> (weight_threshold_));
  }

private:

  NumericOption<float> xyz_weight_;
  NumericOption<float> normal_weight_;
  NumericOption<float> color_weight_;
  NumericOption<float> curvature_weight_;
  NumericOption<float> weight_threshold_;

};

}

#endif /* FACTORY_WEIGHT_COMPUTER_FACTORY_H */


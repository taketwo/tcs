#ifndef FACTORY_WEIGHT_COMPUTER_FACTORY_H
#define FACTORY_WEIGHT_COMPUTER_FACTORY_H

#include <boost/any.hpp>

#include "factory.h"
#include "graph/weight.h"

namespace factory
{

template <typename PointT, typename Graph>
class WeightComputerFactory : public Factory
{

public:

  typedef std::function<void (Graph&)> ComputeWeightsOperator;

  WeightComputerFactory ()
  : Factory ("Weight Computer")
  , distance_weight_ ("distance weight", "-z", 1.0f)
  , normal_weight_ ("normal weight", "-n", 0.08f)
  , color_weight_ ("color weight", "-c", 0.5f)
  , curvature_weight_ ("curvature weight", "-b", 0.0006f)
  , convex_discount_ ("convex discount", "--convex-discount", 0.01f)
  {
    add (&distance_weight_);
    add (&normal_weight_);
    add (&color_weight_);
    add (&curvature_weight_);
    add (&convex_discount_);
    add (&weightless_);
  }

  const ComputeWeightsOperator&
  instantiate (int argc, char** argv)
  {
    parse (argc, argv);

    using namespace pcl::graph::weight;

    if (weightless_.policy == WeightlessOption::SMALL_WEIGHT_COERCE_TO_THRESHOLD)
    {
      typedef
        weight_computer<PointT,
                        terms<
                          tag::normalized<tag::xyz>
                        , tag::drop_if_convex<tag::normal>
                        , tag::drop_if_convex<tag::curvature>
                        , tag::normalized<tag::color>
                        >,
                        pcl::graph::weight::function::gaussian,
                        policy::coerce
                        > computer_type;
      std::shared_ptr<computer_type> computer (new computer_type (tag::xyz::scale = static_cast<float> (distance_weight_),
                                                                  tag::normal::scale = static_cast<float> (normal_weight_),
                                                                  tag::curvature::scale = static_cast<float> (curvature_weight_),
                                                                  tag::color::scale = static_cast<float> (color_weight_),
                                                                  tag::weight::threshold = static_cast<float> (weightless_.threshold)));
      computer_ = computer;
      operator_ = std::bind (&computer_type::template operator()<Graph>, &(*computer), std::placeholders::_1);
      return operator_;
    }
    else
    {
      typedef
        weight_computer<PointT,
                        terms<
                          tag::normalized<tag::xyz>
                        , tag::drop_if_convex<tag::normal>
                        , tag::drop_if_convex<tag::curvature>
                        , tag::normalized<tag::color>
                        >,
                        pcl::graph::weight::function::gaussian,
                        policy::remove
                        > computer_type;
      std::shared_ptr<computer_type> computer (new computer_type (tag::xyz::scale = static_cast<float> (distance_weight_),
                                                                  tag::normal::scale = static_cast<float> (normal_weight_),
                                                                  tag::curvature::scale = static_cast<float> (curvature_weight_),
                                                                  tag::color::scale = static_cast<float> (color_weight_),
                                                                  tag::weight::threshold = static_cast<float> (weightless_.threshold)));
      computer_ = computer;
      operator_ = std::bind (&computer_type::template operator()<Graph>, &(*computer), std::placeholders::_1);
      return operator_;
    }
  }

private:

  struct WeightlessOption : Option
  {

    enum SmallWeightPolicy
    {
      SMALL_WEIGHT_REMOVE_EDGE,
      SMALL_WEIGHT_COERCE_TO_THRESHOLD
    };

    WeightlessOption ()
    : Option ("+/- threshold", "--weightless")
    , threshold (1e-5)
    {
    }

    virtual void parse (int argc, char** argv)
    {
      pcl::console::parse (argc, argv, key.c_str (), threshold);
      policy = threshold < 0 ? SMALL_WEIGHT_REMOVE_EDGE : SMALL_WEIGHT_COERCE_TO_THRESHOLD;
      threshold = std::fabs (threshold);
    }

    virtual std::vector<ValueInfo> getValueInfo ()
    {
      return { std::make_pair ("threshold", boost::lexical_cast<std::string> (threshold)),
               std::make_pair ("policy", policy == SMALL_WEIGHT_REMOVE_EDGE ? "REMOVE" : "COERCE") };
    }

    float threshold;
    SmallWeightPolicy policy;

  };

  NumericOption<float> distance_weight_;
  NumericOption<float> normal_weight_;
  NumericOption<float> color_weight_;
  NumericOption<float> curvature_weight_;
  NumericOption<float> convex_discount_;
  WeightlessOption weightless_;

  boost::any computer_;
  ComputeWeightsOperator operator_;

};

}

#endif /* FACTORY_WEIGHT_COMPUTER_FACTORY_H */


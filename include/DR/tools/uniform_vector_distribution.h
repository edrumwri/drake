#pragma once

#include <random>

#include <drake/common/eigen_types.h>

#include <DR/tools/input_stream_operators.h>

namespace DR {
/**
  Uniform real vector distribution for a `drake::VectorX<T>` populated with random real numbers.

  Each component of the vector is sampled independently from a continuous uniform distribution on the
  range [min, max] with equal probability throughout the range.  The URNG should be real-valued and deliver numbers in
  the range [0, 1].
  Note: sampled real number interval may have inclusive or exclusive bounds.

  Satisfies C++ named requirements for
  [RandomNumberDistribution](https://en.cppreference.com/w/cpp/named_req/RandomNumberDistribution).
*/
template <typename T = double>
class UniformVectorDistribution {
  static_assert(std::is_floating_point<T>::value, "result_type must be a floating point type");

 public:
  /** The type of the range of the distribution. */
  typedef drake::VectorX<T> result_type;

  /** Parameter type. */
  struct param_type {
    typedef UniformVectorDistribution<T> distribution_type;
    param_type() = delete;
    explicit param_type(result_type lb, result_type ub) : lb_(lb), ub_(ub) {
      // Check attribute bounds are correct dims.
      DR_DEMAND(ub_.rows() == lb_.rows());
      // Check that lower_bound <= upper_bound.
      DR_DEMAND(0.0 <= (ub_ - lb_).minCoeff());
      // Check that lower_bound and upper_bound are finite.
      DR_DEMAND(std::isfinite(lb_.norm()));
      DR_DEMAND(std::isfinite(ub_.norm()));
    }

    result_type lb() const { return lb_; }

    result_type ub() const { return ub_; }

    int size() const { return lb_.rows(); }

    friend bool operator==(const param_type& v1, const param_type& v2) { return v1.lb_ == v2.lb_ && v1.ub_ == v2.ub_; }

    friend bool operator!=(const param_type& v1, const param_type& v2) { return !(v1 == v2); }

   private:
    result_type lb_;
    result_type ub_;
  };

 public:
  /// No default constructor.
  UniformVectorDistribution() = delete;

  /**
    Constructs a UniformVectorDistribution object.

    @param lb The inclusive lower bound of the distribution.
    @param ub The inclusive upper bound of the distribution.
   */
  explicit UniformVectorDistribution(result_type lb, result_type ub) : param_(lb, ub) {}

  /**
    Sets the size of the vector and initializes upper and lower bounds at zero.
    @param size the number of rows in the `return_type` vector.  This size cannot be changed after it is set.
   */
  explicit UniformVectorDistribution(int size) : param_(result_type::Zero(size), result_type::Zero(size)) {}

  explicit UniformVectorDistribution(const param_type& param) : param_(param) {}

  /**
    Resets the distribution state.

    Does nothing for the uniform vector distribution.
   */
  void reset() {}

  result_type lb() const { return param_.lb(); }

  result_type ub() const { return param_.ub(); }

  result_type size() const { return param_.size(); }

  /**
    Returns the parameter set of the distribution.
   */
  param_type param() const { return param_; }

  /**
    Sets the parameter set of the distribution.
    @param param The new parameter set of the distribution.
   */
  void param(const param_type& param) {
    // Do not let the vector from the distribution change size.
    DR_DEMAND(param.size() == param_.size());
    param_ = param;
  }

  /**
    Returns the inclusive lower bound of the distribution range.
   */
  result_type min() const { return this->lb(); }

  /**
    Returns the inclusive upper bound of the distribution range.
   */
  result_type max() const { return this->ub(); }

  /**
    Generating functions.
   */
  template <typename URNG>
  result_type operator()(URNG& random_generator) {  // NOLINT: Permit mutable URNG reference.
    return this->operator()(random_generator, param_);
  }

  template <typename URNG>
  result_type operator()(URNG& random_generator, const param_type& param) {  // NOLINT: Permit mutable URNG reference.
    return (std::uniform_real_distribution<double>{0.0, 1.0}(random_generator) * (param.ub() - param.lb())) +
           param.lb();
  }

  friend bool operator==(const UniformVectorDistribution& dist1, const UniformVectorDistribution& dist2) {
    return dist1.param_ == dist2.param_;
  }

  param_type param_;
};

/**
  Return true if two uniform vector distributions have different parameters.
 */
template <typename T>
inline bool operator!=(const UniformVectorDistribution<T>& dist1, const UniformVectorDistribution<T>& dist2) {
  return !(dist1 == dist2);
}

/**
  Inserts a `UniformVectorDistribution` random number distribution `dist` into the output stream `os`.

  @param os An output stream.
  @param dist A `UniformVectorDistribution` random number distribution.

  @returns The output stream with the state of `dist` inserted.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const UniformVectorDistribution<T>& dist) {
  os << "lb=[ " << dist.param().lb().transpose() << " ], ub=[ " << dist.param().ub().transpose() << " ]";
  return os;
}

/**
  Extracts parameters for a `UniformVectorDistribution` random number distribution `dist` from an input stream `is`.
  Example to extract a distribution of `drake::Vector3<T>` with range:
  ```
  std::iostream ios;
  // Upper bound.
  ios << "1 2 3";
  // Lower bound.
  ios << "4 5 6";
  UniformVectorDistribution<T> dist(3);
  ios >> dist;
  // dist.lb() is now {1,2,3}
  // dist.ub() is now {4,5,6}
  ```

  @param is An input stream.
  @param dist A `UniformVectorDistribution` random number distribution.
  @returns The input stream with the state of `dist` extracted.
 */
template <typename T>
std::istream& operator>>(
    std::istream& is,
    UniformVectorDistribution<T>& dist) {  // NOLINT: Permit mutable UniformVectorDistribution reference.
  drake::VectorX<T> lower_bound(dist.size());
  drake::VectorX<T> upper_bound(dist.size());
  // Use the istream operator for `Eigen::MatrixBase`.
  is >> lower_bound >> upper_bound;
  // Set distribution parameters.
  dist.param(UniformVectorDistribution<T>::param_type(lower_bound, upper_bound));
  return is;
}

}  // namespace DR

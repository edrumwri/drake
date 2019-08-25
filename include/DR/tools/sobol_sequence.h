#include <algorithm>

#include <drake/common/eigen_types.h>

#include <DR/common/exception.h>

#pragma once

namespace DR {

/**
 A low-discrepancy, quasi-random sequence.
 @note a nearly direct implementation from Numerical Recipes in C, pp. 312-313.
 */
template <typename T>
class SobolSequence {
 public:
  explicit SobolSequence(int dim) : dim_(dim) {
    DR_DEMAND(dim <= kMaxDim);
    Initialize();
  }

  /// Draws the next sample from the Sobol sequence.
  drake::VectorX<T> Sample() {
    drake::VectorX<T> x(dim_);
    uint64_t im = in_++;
    int rightmost_zero = [this, &im]() -> int {
      // Find the rightmost zero bit.
      for (int j = 0; j < kMaxBit; ++j) {
        if (!(im & 1)) return j;
        im >>= 1;
      }

      DRAKE_UNREACHABLE();
    }();
    im = rightmost_zero * kMaxDim;

    // XOR the appropriate direction number into each component of the vector and convert to a floating point number.
    for (int k = 0; k < std::min(dim_, kMaxDim); ++k) {
       ix_[k] ^= iv_[im + k];
       x[k] = ix_[k] * fac_;
    }

    return x;
  }

 private:
  void Initialize() {
    for (int k = 0; k < kMaxDim; ++k) ix_[k] = 0;
    in_ = 0;
    DR_DEMAND(iv_[0] == 1);
    fac_ = 1.0 / (1L << kMaxBit);
    for (int j = 0, k = 0; j < kMaxBit; ++j, k+= kMaxDim)
      iu_[j] = &iv_[k];

    // To allow both 1D and 2D addressing.
    for (int k = 0; k < kMaxDim; ++k) {
      for (int j = 0; j < static_cast<int>(mdeg_[k]); ++j) iu_[j][k] <<= (kMaxBit - 1 - j);
      // Stored values only require normalization.
      for (int j = mdeg_[k]; j < kMaxBit; ++j) {
        uint64_t ipp = ip_[k];
        int i = iu_[j - mdeg_[k]][k];
        i ^= (i >> mdeg_[k]);
        for (int l = mdeg_[k]-1; l >= 1; --l) {
          if (ipp & 1) i ^= iu_[j-l][k];
          ipp >>= 1;
        }
        iu_[j][k] = i;
      }
    }
  }

  static const int kMaxDim{6};
  static const int kMaxBit{30};

  // The dimensionality of the vector that will be drawn from the sequence.
  const int dim_;

  // These poor variable names are used to match the text in Numerical Recipes in C.
  double fac_;
  uint64_t in_;             // The iterate in the sequence.
  uint64_t ix_[kMaxDim];
  uint64_t* iu_[kMaxBit];
  uint64_t mdeg_[kMaxDim] = { 1, 2, 3, 3, 4, 4 };
  uint64_t ip_[kMaxDim] = { 0, 1, 1, 2, 1, 4 };
  uint64_t iv_[kMaxDim * kMaxBit] = { 1, 1, 1, 1, 1, 1, 3, 1, 3, 3, 1, 1, 5, 7, 7, 3, 3, 5, 15, 11, 5, 15, 13, 9 };
};

}  // namespace DR

// Instantiate templates.
extern template class DR::SobolSequence<double>;

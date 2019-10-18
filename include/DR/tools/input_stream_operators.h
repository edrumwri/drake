#pragma once

#include <drake/common/eigen_types.h>

namespace DR {
template <typename Derived>
std::istream& operator>>(std::istream& s, Eigen::MatrixBase<Derived>& m) {
  for (int i = 0; i < m.rows(); ++i)
    for (int j = 0; j < m.cols(); ++j) s >> m(i, j);

  return s;
}
}  // namespace DR

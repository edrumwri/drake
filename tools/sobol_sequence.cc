#include <DR/tools/sobol_sequence.h>

template <typename T> const int DR::SobolSequence<T>::kMaxDim;
template <typename T> const int DR::SobolSequence<T>::kMaxBit;

// Instantiate templates.
namespace DR {
template class SobolSequence<double>;
}

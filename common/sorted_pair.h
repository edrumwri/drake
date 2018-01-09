#pragma once

#include <algorithm>
#include <utility>

#include "drake/common/is_equality_comparable.h"
#include "drake/common/is_less_than_comparable.h"

/// @file
/// Provides drake::MakeSortedPair and drake::SortedPair for storing two
/// values of a certain type in sorted order.

namespace drake {

/// This class is similar to the std::pair class. However, this class uses a
/// pair of homogeneous types (std::pair can use heterogeneous types) and sorts
/// the first and second values (the first value is lower). Thus the SortedPair
/// class is able to be used to generate keys (e.g., for std::map, etc.) from
/// pairs of objects.
///
/// The data are stored as `const` types to prevent user modification- which
/// could break the sorted ordering- while permitting the familiar `first`
/// and `second` member data accesses provided by std::pair.
///
/// @tparam T A template type that provides the operators `operator==` and
///           `operator<` and supports default construction.
template <class T>
struct SortedPair {
  static_assert(is_equality_comparable<T>::value, "SortedPair can only be used"
      "with a class that provides the equality operator (operator==).");
  static_assert(is_less_than_comparable<T>::value, "SortedPair can only be used"
      "with a class that provides the less than operator (operator<).");

  typedef T type;

  const T first{};                 /// The smaller of the two objects.
  const T second{};                /// The larger of the two objects.

  /// The default constructor creates `first`and `second` using their
  /// respective default constructors.
  SortedPair() = default;
  SortedPair(const SortedPair& s) = default;

  /// Move constructor.
  SortedPair(SortedPair&& s) {
    const_cast<T&>(first) = std::move(const_cast<T&>(s.first));
    const_cast<T&>(second) = std::move(const_cast<T&>(s.second));
  }

  /// Rvalue reference constructor, permits constructing with std::unique_ptr
  /// types, for example.
  SortedPair(T&& a, T&& b) {
    if (a < b) {
      const_cast<T&>(first) = std::move(const_cast<T&>(a));
      const_cast<T&>(second) = std::move(const_cast<T&>(b));
    } else {
      const_cast<T&>(first) = std::move(const_cast<T&>(b));
      const_cast<T&>(second) = std::move(const_cast<T&>(a));
    }
  }

  /// Move assignment operator.
  SortedPair& operator=(SortedPair&& s) {
    const_cast<T&>(first) = std::move(const_cast<T&>(s.first));
    const_cast<T&>(second) = std::move(const_cast<T&>(s.second));
    return *this;
  }

  /// Constructs a %SortedPair from two objects.
  SortedPair(const T& a, const T& b) : first(a), second(b) {
    if (first > second)
      std::swap(const_cast<T&>(first), const_cast<T&>(second));
  }

  /// Type-converting copy constructor.
  template <class U>
  SortedPair(const SortedPair<U>& p) : first(p.first), second(p.second) { }

  SortedPair& operator=(const SortedPair& p) {
    // This block of code is necessary since `first` and `second` are const
    // objects.
    const_cast<T&>(first) = p.first;
    const_cast<T&>(second) = p.second;
    return *this;
  }

  // Resets the objects.
  void set(const T& a, const T& b) {
    if (a < b) {
      const_cast<T&>(first) = a;
      const_cast<T&>(second) = b;
    } else {
      const_cast<T&>(first) = b;
      const_cast<T&>(second) = a;
    }
  }
};

/// Two pairs of the same type are equal iff their members are equal after
/// sorting.
template <class T>
inline bool operator==(const SortedPair<T>& x, const SortedPair<T>& y) {
  return x.first == y.first && x.second == y.second;
}

// Compares two pairs in the following way: `x < y` is true iff when
// `x.first < y.first` or `x.first == y.first and x.second < y.second)`.
template <class T>
inline bool operator<(const SortedPair<T>& x, const SortedPair<T>& y) {
  return x.first < y.first || (x.first == y.first && x.second < y.second);
}

/// Determine whether two SortedPair objects are not equal using `operator==`.
template <class T> inline bool operator!=(
    const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x == y);
}

/// Determines whether `x > y` using `operator<`.
template <class T>
inline bool operator>(const SortedPair<T>& x, const SortedPair<T>& y) {
  return y < x;
}

/// Determines whether `x <= y` using `operator<`.
template <class T>
inline bool operator<=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(y < x);
}

/// Determines whether `x >= y` using `operator<`.
template <class T>
inline bool
operator>=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x < y);
}

/// @brief A convenience wrapper for creating a sorted pair from two objects.
/// @param x  The first object.
/// @param y  The second object.
/// @return A newly-constructed SortedPair object.
template <class T>
inline SortedPair<T>
MakeSortedPair(const T& x, const T& y) {
  return SortedPair<T>(x, y);
}

}  // namespace drake

namespace std {

/// Implements std::swap().
template <class T>
void swap(drake::SortedPair<T>& t, drake::SortedPair<T>& u) {
  std::swap(const_cast<T&>(t.first), const_cast<T&>(u.first));
  std::swap(const_cast<T&>(t.second), const_cast<T&>(u.second));
}

/// Implements std::hash().
template <class T>
struct hash<drake::SortedPair<T>> {
  std::size_t operator()(const drake::SortedPair<T>& s) const noexcept {
    // Formula below is taken from boost::hash_combine().
    const std::size_t h1 = std::hash<T>()(s.first);
    const std::size_t h2 = std::hash<T>()(s.second);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

}  // namespace std


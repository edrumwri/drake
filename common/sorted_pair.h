#pragma once

#include <algorithm>

/// @file
/// Provides drake::make_sorted_pair and drake::sorted_pair for storing two
/// values of a certain type in sorted order.

namespace drake {

/// This class is quite similar to the C++ pair class
/// However, this class uses a pair of homogeneous
/// types pair can use heterogeneous types) and sorts the first and
/// second values (the first value is lower). Thus the sorted_pair class is
/// able to be used to generate keys (e.g., for STL map, etc.) from pairs of
/// objects.
template<class T>
struct sorted_pair
{
  typedef T type;    

  const T first;                 ///< @c first is a copy of the first object
  const T second;                ///< @c second is a copy of the second object

  // _GLIBCXX_RESOLVE_LIB_DEFECTS
  // 265.  std::pair::pair() effects overly restrictive
  /** The default constructor creates @c first and @c second using their
    *  respective default constructors.  */
  sorted_pair() : first(), second() { }

  /** Two objects may be passed to a @c pair constructor to be copied.  */
  sorted_pair(const T& __a, const T& __b) : first(__a), second(__b) 
  { 
    if (__a < __b) 
    {
      T* first_nc = (T*) &first;
      T* second_nc = (T*) &second;
      std::swap(*first_nc, *second_nc);
    } 
  }

  /** There is also a templated copy ctor for the @c pair class itself.  */
  template<class _U>
  sorted_pair(const sorted_pair<_U>& __p) : first(__p.first), second(__p.second) { }

  /** Assignment operator */
  void operator=(const sorted_pair& __p)
  {
    // get the two objects as non-const
    T* first_nc = (T*) &first;
    T* second_nc = (T*) &second;
    
    *first_nc = __p.first;
    *second_nc = __p.second;
  }
};

/// Two pairs of the same type are equal iff their members are equal.
template<class T>
inline bool
operator==(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{ 
return __x.first == __y.first && __x.second == __y.second; 
}

template<class T>
inline bool
operator<(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{
  return __x.first < __y.first || (!(__y.first < __x.first) && __x.second < __y.second); 
}

/// Uses @c operator== to find the result.
template<class T>
inline bool
operator!=(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{ return !(__x == __y); }

/// Uses @c operator< to find the result.
template<class T>
inline bool
operator>(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{ return __y < __x; }

/// Uses @c operator< to find the result.
template<class T>
inline bool
operator<=(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{ return !(__y < __x); }

/// Uses @c operator< to find the result.
template<class T>
inline bool
operator>=(const sorted_pair<T>& __x, const sorted_pair<T>& __y)
{ return !(__x < __y); }

/**
*  @brief A convenience wrapper for creating a pair from two objects.
*  @param  x  The first object.
*  @param  y  The second object.
*  @return   A newly-constructed pair<> object of the appropriate type.
*
*  The standard requires that the objects be passed by reference-to-const,
*  but LWG issue #181 says they should be passed by const value.  We follow
*  the LWG by default.
*/
// _GLIBCXX_RESOLVE_LIB_DEFECTS
// 181.  make_pair() unintended behavior
template<class T>
inline sorted_pair<T>
make_sorted_pair(const T __x, const T __y)
{ return sorted_pair<T>(__x, __y); }

}  // namespace drake


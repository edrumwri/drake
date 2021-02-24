#pragma once

#include <cstring>
#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/drake_copyable.h"

namespace drake {

  struct ConstCharHash {
    // Fowler–Noll–Vo hash function (FNV-1).
    size_t operator()(const char* key) const {
      // These values from https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function#FNV_prime
      // are specialized for 64-bit size_t.
      const size_t kFnvOffsetBasis = 14695981039346656037U;
      const size_t kFnvPrime = 1099511628211;

      size_t hash = kFnvOffsetBasis;
      for (const char* i = key; *i != '\0'; ++i) {
        hash *= kFnvPrime;
        hash ^= *i;
      }

      return hash;
    }
  };

  struct ConstCharEqual {
    constexpr bool operator()(const char* lhs, const char* rhs) const {
      return strcmp(lhs, rhs) == 0;
    }
  };


/**
 An unordered map storing elements of type T that is keyed on strings
 represented using const char*.
 */
template <typename T>
class ConstCharKeyUnorderedMap {
 public:
 ConstCharKeyUnorderedMap() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstCharKeyUnorderedMap)
  typedef typename std::unordered_map<const char*, T, ConstCharHash, ConstCharEqual>::iterator iterator;
  typedef typename std::unordered_map<const char*, T, ConstCharHash, ConstCharEqual>::const_iterator const_iterator;
  typedef typename std::unordered_map<const char*, T, ConstCharHash, ConstCharEqual>::size_type size_type;

  /// Constant time operation for querying whether the container is empty.
  bool empty() const { return hashtable_.empty(); }

  /// Constant time operation for querying the number of key/element pairs in the container.
  size_type size() const { return hashtable_.size(); }

  void clear() noexcept {
    hashtable_.clear();
    keys_.clear();
  }

  iterator find(const char* key) {
    return hashtable_.find(key);
  }

  const_iterator find(const char* key) const {
    return hashtable_.find(key);
  }

#if 0
  /// Returns a reference to the mapped value of the element with key
  /// equivalent to `key`. If no such element exists, an exception is thrown.
  T& at(const char* key ) { return hashtable_.at(key); }
  const T& at(const char* key) const { return hashtable_.at(key); }
#endif

  /// Reference to the mapped value of the new element if no element with key key existed. Otherwise a reference to the mapped value of the existing element whose key is equivalent to key.
  T& operator[](const std::string& key) {
    return operator[](key.c_str());
  }

  /// Reference to the mapped value of the new element if no element with key key existed. Otherwise a reference to the mapped value of the existing element whose key is equivalent to key.
  T& operator[](const char* key) {
    // Attempt to find the value.
    auto iter = hashtable_.find(key);
    if (iter != hashtable_.end()) return iter->second;

    // Value not found. Insert it into the hashtable.
    auto key_bool_pair = keys_.insert(std::string(key));
    return hashtable_[key_bool_pair.first->c_str()];
  }

  iterator end() noexcept { return hashtable_.end(); }
  const_iterator end() const noexcept { return hashtable_.end(); }

 private:
  std::unordered_set<std::string> keys_;
  std::unordered_map<const char*, T, ConstCharHash, ConstCharEqual> hashtable_;
};

}  // namespace drake

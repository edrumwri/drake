#pragma once

#include <cstring>
#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/const_char_key_unordered_map.h"

namespace drake {

/**
 An unordered multimap storing elements of type T that is keyed on strings
 represented using const char*.
 */
template <typename T>
class ConstCharKeyUnorderedMultimap {
 public:
 ConstCharKeyUnorderedMultimap() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstCharKeyUnorderedMultimap)
  typedef std::pair<const char*, T> value_type;
  typedef typename std::unordered_multimap<const char*, T, ConstCharHash, ConstCharEqual>::iterator iterator;
  typedef typename std::unordered_multimap<const char*, T, ConstCharHash, ConstCharEqual>::const_iterator const_iterator;
  typedef typename std::unordered_multimap<const char*, T, ConstCharHash, ConstCharEqual>::size_type size_type;

  /// Constant time operation for querying whether the container is empty.
  bool empty() const { return hashtable_.empty(); }

  /// Constant time operation for querying the number of key/element pairs in the container.
  size_type size() const { return hashtable_.size(); }

  void clear() noexcept {
    hashtable_.clear();
    keys_.clear();
  }

  size_type count(const char* key) const { return hashtable_.count(key); }

  std::pair<iterator, iterator> equal_range(const char* key) { return hashtable_.equal_range(key); }
  std::pair<const_iterator,const_iterator> equal_range(const char* key ) const { return hashtable_.equal_range(key); }

  iterator find(const char* key) { return hashtable_.find(key); }
  const_iterator find(const char* key) const { return hashtable_.find(key); }

  /// Returns a reference to the mapped value of the element with key
  /// equivalent to `key`. If no such element exists, an exception is thrown.
  T& at(const char* key ) { return hashtable_.at(key); }
  const T& at(const char* key) const { return hashtable_.at(key); }

  iterator insert(const std::pair<std::string, T>& value) {
    auto key_bool_pair = keys_.insert(value.first);
    return hashtable_.insert(std::make_pair(key_bool_pair->c_str(), value.second));
  }

  iterator insert(const value_type& value) {
    auto key_bool_pair = keys_.insert(std::string(value.first));
    return hashtable_.insert(std::make_pair(key_bool_pair->c_str(), value.second));
  }

 private:
  std::unordered_multiset<std::string> keys_;
  std::unordered_multimap<const char*, T, ConstCharHash, ConstCharEqual> hashtable_;
};

}  // namespace drake

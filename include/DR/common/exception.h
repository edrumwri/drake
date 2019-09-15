#pragma once

#include <csignal>
#include <iostream>

#include <drake/common/drake_assert.h>
#include <drake/common/drake_assertion_error.h>

/**
This file wraps the DRAKE_DEMAND definitions used for condition checking in
Drake and replicates the DRAKE_UNREACHABLE macro.
*/

// DR_DEMAND macro that should be called by users.
// See DR_DEMAND_WITH_MESSAGE for parameter definitions.
#define DR_DEMAND(condition, ...) DR_DEMAND_(condition, ##__VA_ARGS__, 2, 1)

#ifdef NDEBUG
// if not debugging, default to behavior of DRAKE_ASSERT
#define DR_ASSERT(condition, ...) DRAKE_ASSERT(condition);
#else  // NDEBUG
// if debugging, use DR_DEMAND.
#define DR_ASSERT(condition, ...) DR_DEMAND_(condition, ##__VA_ARGS__, 2, 1)
#endif  // NDEBUG

/**
*ATTENTION*: use DR_DEMAND to call this macro.

Copies behavior of DRAKE_DEMAND, defined in drake/common/drake_assert.h while adding a custom error message.

@param condition is an expression that can evalueate to a bool value.
@param msg is an expression that can be used to construct a std::string.  The
       string will follow the error message reported by DRAKE_DEMAND.
*/
#define DR_DEMAND_WITH_MESSAGE(condition, msg)                                                          \
  do {                                                                                                  \
    typedef ::drake::assert::ConditionTraits<typename std::remove_cv<decltype(condition)>::type> Trait; \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");                            \
    if (!Trait::Evaluate(condition)) {                                                                  \
      std::cerr << "abort: Failure at " << __FILE__ << ":" << __LINE__ << " in " << __func__ << "()"    \
                << ": condition '" << #condition << "' failed." << std::endl;                           \
      std::abort();                                                                                     \
    }                                                                                                   \
  } while (0)

// Dispatcher for DR_DEMAND macros.
#define DR_DEMAND_(condition, msg, n, ...) DR_DEMAND##n(condition, msg)

// Overloads of the DR_DEMAND macro.
#define DR_DEMAND1(condition, ...) DRAKE_DEMAND(condition)
#define DR_DEMAND2(condition, msg) DR_DEMAND_WITH_MESSAGE(condition, msg)

#define DR_UNREACHABLE() std::abort();


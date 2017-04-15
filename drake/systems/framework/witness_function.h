#pragma once

#include "drake/systems/framework/discrete_event.h"

namespace drake {
namespace systems {

template <class T>
class WitnessFunction {
 public:
  virtual ~WitnessFunction() {}

  enum TriggerType {
    /// This witness function will never be triggered.
    kNone = 0,

    /// Witness function triggers when the function crosses zero after an
    /// initial positive evaluation.
    kPositiveThenNegative = 1,

    /// Witness function triggers when the function crosses zero after an
    /// initial negative evaluation.
    kNegativeThenPositive = 2,

    /// Witness function triggers *any time* the function crosses zero.
    /// Convenience definition for equivalence to bitwise OR of
    /// kPositiveThenNegative and kNegativeThenPositive.
    kCrossesZero = 3,
  };

  /// Derived classes will override this function to get the type of event
  /// that the witness function will trigger.
  virtual typename DiscreteEvent<T>::ActionType get_action_type() const = 0;

  /// Derived classes will override this function to get the witness function
  /// trigger type.
  virtual TriggerType get_trigger_type() const = 0;

  /// Derived classes will override this function gets the time tolerance with
  /// which to isolate the first witness trigger.
  virtual T get_time_isolation_tolerance() const = 0;

  /// Derived classes will override this function to evaluate the witness
  /// function at the given context.
  virtual T Evaluate(const Context<T>& context) = 0;

  /// Derived classes will override this function to return the positive "dead"
  /// band.
  virtual T get_positive_dead_band() const = 0;

  /// Derived classes will override this function to return the negative "dead"
  /// band.
  virtual T get_negative_dead_band() const = 0;

  /// Derived classes will override this function to get the time that the
  /// witness function should trigger, given two times and two witness function
  /// evaluations.
  virtual T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                                const std::pair<T, T>& time_and_witness_valuef)
                                const = 0;

  /// Gets the time that the witness function should trigger, given two times
  /// and two function evaluations. Calls do_get_trigger_time().
  /// @throws std::logic_error If the difference between the two times is
  ///         greater than the time isolation tolerance or the second time
  ///         is less than the first time.
  T get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                     const std::pair<T, T>& time_and_witness_valuef) const {
    if (time_and_witness_value0.first > time_and_witness_valuef.first) {
      throw std::logic_error("First time should not be greater than the second"
                                 "time.");
    }
    if (time_and_witness_valuef.first - time_and_witness_value0.first >
        get_time_isolation_tolerance()) {
      throw std::logic_error("Difference in times is greater than the time"
                                 "isolation tolerance.");
    }

    return do_get_trigger_time(time_and_witness_value0,
                               time_and_witness_valuef);
  }

  /// Checks whether the witness function should trigger using given
  /// values at w0 and wf. Note that this function is not specific to a
  /// particular witness function.
  bool should_trigger(const T& w0, const T& wf) const {
    TriggerType ttype = get_trigger_type();

    // Get the positive and negative dead bands.
    T positive_dead = get_positive_dead_band();
    T negative_dead = get_negative_dead_band();

    if (ttype | TriggerType::kPositiveThenNegative) {
      if (w0 > positive_dead && wf < negative_dead)
        return true;
    }

    if (ttype | TriggerType::kNegativeThenPositive) {
      if (w0 < negative_dead && wf > positive_dead)
        return true;
    }

    // No triggers triggered.
    return false;
  }
};

}  // namespace systems
}  // namespace drake
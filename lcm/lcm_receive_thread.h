#pragma once

#include <atomic>
#include <thread>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace lcm {

/**
 * (Advanced.)  Maintains a thread that receives LCM messages and dispatches
 * the messages to the appropriate message handlers.
 *
 * @warning Almost no Drake uses of LCM should require a background thread.
 * Please use DrakeLcmInterface::HandleSubscriptions() or
 * drake::systems::lcm::LcmInterfaceSystem instead.
 */
class DRAKE_DEPRECATED("2019-08-01",
    "There is no replacement.  This class is trivially simple; feel free to "
    "copy and adapt it instead, if needed.")
LcmReceiveThread {
 public:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmReceiveThread)
#pragma GCC diagnostic pop

  /**
   * A constructor that instantiates the thread.
   *
   * @param[in] lcm A pointer to the LCM instance through which to access the
   * LCM network. This parameter cannot be `nullptr` and must remain valid for
   * the lifetime of this object.
   */
  explicit LcmReceiveThread(::lcm::LCM* lcm);

  /**
   * The destructor that ensures the thread that receives LCM message is
   * stopped.
   */
  ~LcmReceiveThread();

  /**
   * Stops the LCM receive thread. This stops the reception of LCM messages.
   */
  void Stop();

 private:
  // Loops waiting for LCM messages and dispatching them to the appropriate
  // subscriber message handlers when they arrive.
  void Looper();

  // Whether to stop lcm_thread_.
  std::atomic_bool stop_{false};

  // A pointer to the LCM instance.
  ::lcm::LCM* const lcm_{nullptr};

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;
};

}  // namespace lcm
}  // namespace drake

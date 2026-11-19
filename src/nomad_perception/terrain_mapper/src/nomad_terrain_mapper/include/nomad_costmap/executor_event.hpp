#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/waitable.hpp>
#include <atomic>
#include <functional>
#include <stdexcept>

namespace nomad {
// Only notifications coalesce. The owner's input FIFO is never overwritten.
// All callbacks execute on the node's existing mutually exclusive group.
class ExecutorEvent final : public rclcpp::Waitable {
  rclcpp::GuardCondition guard_;
  std::atomic<bool> pending_{false};
  std::function<void()> callback_;
  size_t index_ = 0;
  void add(rcl_wait_set_t& wait_set) {
    if (rcl_wait_set_add_guard_condition(&wait_set, &guard_.get_rcl_guard_condition(), &index_) != RCL_RET_OK)
      throw std::runtime_error("Cannot attach native event guard to ROS wait set");
  }
  bool ready(const rcl_wait_set_t& wait_set) const {
    return pending_.load(std::memory_order_acquire) &&
      index_ < wait_set.size_of_guard_conditions && wait_set.guard_conditions[index_] != nullptr;
  }
  void dispatch(const std::shared_ptr<void>& data) { if (data) callback_(); }
public:
  ExecutorEvent(rclcpp::Context::SharedPtr context, std::function<void()> callback)
    : guard_(context), callback_(std::move(callback)) {}
  void notify() {
    pending_.store(true, std::memory_order_release);
    guard_.trigger();
  }
  size_t get_number_of_ready_guard_conditions() override { return 1; }
  // Humble uses pointer/non-const signatures. Jazzy also supplies the newer
  // reference/const signatures. Providing both keeps one implementation.
  void add_to_wait_set(rcl_wait_set_t* s) override { add(*s); }
  void add_to_wait_set(rcl_wait_set_t& s) { add(s); }
  bool is_ready(rcl_wait_set_t* s) override { return ready(*s); }
  bool is_ready(const rcl_wait_set_t& s) { return ready(s); }
  std::shared_ptr<void> take_data() override {
    return pending_.exchange(false, std::memory_order_acq_rel) ? std::make_shared<bool>(true) : nullptr;
  }
  void execute(std::shared_ptr<void>& data) override { dispatch(data); }
  void execute(const std::shared_ptr<void>& data) { dispatch(data); }
};
}

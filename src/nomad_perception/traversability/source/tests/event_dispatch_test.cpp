#include "nomad_costmap/executor_event.hpp"
#include "nomad_costmap/snapshot_writer.hpp"
#include <cassert>
#include <future>
#include <iostream>

int main(int argc, char** argv) {
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nomad_native_event_contract_test");
  std::promise<void> entered, retriggered, finished;
  auto entered_future = entered.get_future();
  auto retriggered_future = retriggered.get_future();
  auto finished_future = finished.get_future();
  unsigned callbacks = 0;
  auto event = std::make_shared<nomad::ExecutorEvent>(node->get_node_base_interface()->get_context(), [&] {
    if (++callbacks == 1) {
      entered.set_value();
      assert(retriggered_future.wait_for(3s) == std::future_status::ready);
    } else if (callbacks == 2) finished.set_value();
  });
  node->get_node_waitables_interface()->add_waitable(event, nullptr);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread producer([&] {
    std::this_thread::sleep_for(20ms);
    event->notify();
    assert(entered_future.wait_for(3s) == std::future_status::ready);
    // Must not lose the wakeup that arrives while its callback is executing.
    for (int i = 0; i < 100; ++i) event->notify();
    retriggered.set_value();
  });
  assert(executor.spin_until_future_complete(finished_future, 5s) == rclcpp::FutureReturnCode::SUCCESS);
  producer.join();
  assert(callbacks == 2);
  executor.remove_node(node);
  node->get_node_waitables_interface()->remove_waitable(event, nullptr);
  event.reset();node.reset();
  rclcpp::shutdown();

  std::promise<void> writing, release;
  auto writing_future = writing.get_future();
  auto release_future = release.get_future();
  std::vector<int> written;
  nomad::SnapshotWriter writer([&](const nomad::Json& result) {
    int generation = result.at("generation");
    if (generation == 1) {
      writing.set_value();
      assert(release_future.wait_for(3s) == std::future_status::ready);
    }
    written.push_back(generation);
  });
  writer.submit(std::make_shared<const nomad::Json>(nomad::Json{{"generation",1}}));
  assert(writing_future.wait_for(3s) == std::future_status::ready);
  for (int i = 2; i <= 100; ++i)
    writer.submit(std::make_shared<const nomad::Json>(nomad::Json{{"generation",i}}));
  release.set_value();
  writer.stop();
  assert((written == std::vector<int>{1,100}));
  assert(writer.fault().empty());
  nomad::SnapshotWriter failed([](const nomad::Json&) {throw std::runtime_error("test write error");});
  failed.submit(std::make_shared<const nomad::Json>(nomad::Json::object()));
  failed.stop();
  assert(failed.fault() == "test write error");
  std::cout << "ROS guard wakeup without a polling timer, retrigger during callback, "
    "nonblocking latest diagnostic writer and write failure reporting passed\n";
}

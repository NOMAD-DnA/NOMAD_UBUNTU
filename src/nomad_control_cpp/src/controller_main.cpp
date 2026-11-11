#include "nomad_control_cpp/controller_node.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
} // namespace

int main(int argc, char* argv[]) {
    // Own signal handling: on Ctrl+C/SIGTERM send 0A before ROS shuts down.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::shared_ptr<nomad::ControllerNode> node;
    int result = 0;
    try {
        node = nomad::ControllerNode::create();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (rclcpp::ok() && !interrupted) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    } catch (const std::exception& error) {
        std::cerr << "Controller error: " << error.what() << '\n';
        result = 1;
    }
    if (node) node->prepareShutdown();
    node.reset();
    rclcpp::shutdown();
    return result;
}

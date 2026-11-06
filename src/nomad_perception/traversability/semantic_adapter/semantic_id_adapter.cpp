#include "semantic_mapping.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class SemanticAdapter : public rclcpp::Node {
 public:
  explicit SemanticAdapter(nomad_semantic::Mapping mapping)
    : Node("nomad_semantic_id_adapter"), mapping_(std::move(mapping)) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(mapping_.output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(mapping_.input_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr input) {
        auto output = std::make_unique<sensor_msgs::msg::Image>();
        std::string error;
        if (mapping_.convert(*input, *output, error)) publisher_->publish(std::move(output));
        else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected semantic image: %s", error.c_str());
      });
    if (std::string(publisher_->get_topic_name()) == subscription_->get_topic_name())
      throw std::runtime_error("Resolved input/output topics must differ");
    for (const auto &qos : {publisher_->get_actual_qos(), subscription_->get_actual_qos()}) {
      const auto actual = qos.get_rmw_qos_profile();
      if (actual.history != RMW_QOS_POLICY_HISTORY_KEEP_LAST || actual.depth != 5 ||
          actual.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ||
          actual.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE)
        throw std::runtime_error("Adapter actual DDS QoS differs from depth5 / best-effort / volatile");
    }
    RCLCPP_INFO(get_logger(), "Temporary ADE150 -> field12: %s -> %s; wood/gravel/leaves unavailable",
      mapping_.input_topic.c_str(), mapping_.output_topic.c_str());
  }
 private:
  nomad_semantic::Mapping mapping_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
int main(int argc, char **argv) {
  try {
    const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::string mapping_path, model_path;
    for (size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--mapping" && i + 1 < args.size()) mapping_path = args[++i];
      else if (args[i] == "--source-model-config" && i + 1 < args.size()) model_path = args[++i];
      else throw std::runtime_error("Usage: nomad_semantic_id_adapter --mapping FILE --source-model-config FILE [--ros-args ...]");
    }
    if (mapping_path.empty() || model_path.empty()) throw std::runtime_error("Both mapping and source model config are required");
    auto mapping = nomad_semantic::Mapping::load(nomad_semantic::read_json(mapping_path),
      nomad_semantic::read_json(model_path));
    auto node = std::make_shared<SemanticAdapter>(std::move(mapping));
    rclcpp::spin(node);
    rclcpp::shutdown(); return 0;
  } catch (const std::exception &error) {
    std::cerr << "Semantic adapter: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}

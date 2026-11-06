// RGB topic -> FCN-ResNet18 SUNRGB-D TensorRT -> raw SUNRGB-D mono8 IDs.
// Camera ownership stays outside; a complete validated JSON LUT supplies field12.
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
constexpr int kInputW = 512;
constexpr int kInputH = 512;
constexpr int kOutputW = 640;
constexpr int kOutputH = 360;
constexpr int kSourceClasses = 21;
constexpr std::array<float, 3> kMean{0.485F, 0.456F, 0.406F};
constexpr std::array<float, 3> kStd{0.229F, 0.224F, 0.225F};
constexpr std::array<const char*, 12> kField12Names{
    "ignore", "person", "vehicle", "wood", "soil", "gravel", "paved", "rock",
    "leaves", "vegetation", "artificial", "sky"};

std::vector<std::string> read_classes(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open source classes file: " + path);
  std::vector<std::string> labels;
  for (std::string line; std::getline(file, line);) {
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#') continue;
    const auto last = line.find_last_not_of(" \t\r\n");
    labels.push_back(line.substr(first, last - first + 1));
  }
  if (labels.size() != kSourceClasses)
    throw std::runtime_error("Expected official SUNRGB-D classes.txt with exactly 21 names");
  return labels;
}

std::array<std::uint8_t, kSourceClasses> read_mapping(
    const std::string& path, const std::vector<std::string>& labels) {
  cv::FileStorage file(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (!file.isOpened()) throw std::runtime_error("Cannot open SUNRGB-D mapping JSON: " + path);
  const auto schema_version = file["schema_version"];
  const auto unknown = file["unknown_target_id"];
  if (!schema_version.isInt() || static_cast<int>(schema_version) != 1)
    throw std::runtime_error("mapping.schema_version must be 1");
  if (!unknown.isInt() || static_cast<int>(unknown) != 0)
    throw std::runtime_error("mapping.unknown_target_id must be ignore(0)");
  const auto source_count = file["source_class_count"];
  if (!source_count.isInt() || static_cast<int>(source_count) != kSourceClasses)
    throw std::runtime_error("mapping.source_class_count must be 21");
  const auto project_classes = file["project_classes"];
  if (!project_classes.isSeq() || project_classes.size() != 12)
    throw std::runtime_error("mapping.project_classes must define 12 target IDs");
  for (std::size_t i = 0; i < kField12Names.size(); ++i) {
    const auto name = project_classes[static_cast<int>(i)];
    if (!name.isString() || static_cast<std::string>(name) != kField12Names[i])
      throw std::runtime_error("mapping.project_classes order differs from field12 contract");
  }
  const auto entries = file["classes"];
  if (!entries.isSeq() || entries.size() != kSourceClasses)
    throw std::runtime_error("Mapping must contain exactly 21 source class entries");
  std::array<std::uint8_t, kSourceClasses> lut{};
  std::array<bool, kSourceClasses> seen{};
  for (const auto& entry : entries) {
    const auto source = entry["source_id"];
    const auto target = entry["target_id"];
    const auto name = entry["source_name"];
    if (!source.isInt() || !target.isInt())
      throw std::runtime_error("Mapping IDs must be integers");
    const int source_id = static_cast<int>(source);
    const int target_id = static_cast<int>(target);
    if (source_id < 0 || source_id >= kSourceClasses || seen[source_id])
      throw std::runtime_error("Duplicate or out-of-range source_id in mapping");
    if (target_id < 0 || target_id >= 12)
      throw std::runtime_error("target_id must be in 0..11");
    if (!name.isString() || static_cast<std::string>(name) != labels[source_id])
      throw std::runtime_error("Mapping source_name differs from official classes.txt");
    lut[source_id] = static_cast<std::uint8_t>(target_id);
    seen[source_id] = true;
  }
  for (bool present : seen)
    if (!present) throw std::runtime_error("Missing source class in mapping");
  return lut;
}

void cuda_check(cudaError_t result, const char* operation) {
  if (result != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
}

std::string shape_string(const nvinfer1::Dims& shape) {
  std::ostringstream text;
  text << '[';
  for (int i = 0; i < shape.nbDims; ++i) text << (i ? "," : "") << shape.d[i];
  return text.str() + ']';
}

std::size_t element_count(const nvinfer1::Dims& shape) {
  std::size_t count = 1;
  if (shape.nbDims < 1) throw std::runtime_error("Tensor has no dimensions");
  for (int i = 0; i < shape.nbDims; ++i) {
    if (shape.d[i] <= 0 || count > std::numeric_limits<std::size_t>::max() /
                                   static_cast<std::size_t>(shape.d[i]))
      throw std::runtime_error("Unresolved or overflowing tensor shape " + shape_string(shape));
    count *= static_cast<std::size_t>(shape.d[i]);
  }
  return count;
}

class TrtLogger final : public nvinfer1::ILogger {
 public:
  explicit TrtLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {}
  void log(Severity severity, const char* message) noexcept override {
    try {
      if (severity <= Severity::kERROR) RCLCPP_ERROR(logger_, "TensorRT: %s", message);
      else if (severity == Severity::kWARNING) RCLCPP_WARN(logger_, "TensorRT: %s", message);
    } catch (...) { /* TensorRT logger must not throw. */ }
  }
 private:
  rclcpp::Logger logger_;
};

template <class T> struct TrtDeleter {
  void operator()(T* object) const noexcept { if (object) object->destroy(); }
};
template <class T> using TrtPtr = std::unique_ptr<T, TrtDeleter<T>>;

float half_to_float(std::uint16_t bits) {
  const auto exponent = static_cast<unsigned>((bits >> 10U) & 31U);
  const auto mantissa = static_cast<unsigned>(bits & 1023U);
  float value;
  if (exponent == 0) value = std::ldexp(static_cast<float>(mantissa), -24);
  else if (exponent == 31)
    value = mantissa ? std::numeric_limits<float>::quiet_NaN()
                     : std::numeric_limits<float>::infinity();
  else value = std::ldexp(static_cast<float>(1024U + mantissa),
                         static_cast<int>(exponent) - 25);
  return (bits & 0x8000U) ? -value : value;
}

class FcnEngine final {
 public:
  FcnEngine(const std::string& path, int device, rclcpp::Logger logger)
      : logger_(logger), trt_logger_(logger), device_(device) {
    try { initialize(path); } catch (...) { cleanup_cuda(); throw; }
  }
  ~FcnEngine() { cleanup_cuda(); }
  FcnEngine(const FcnEngine&) = delete;
  FcnEngine& operator=(const FcnEngine&) = delete;

  int class_count() const noexcept { return classes_; }

  cv::Mat infer(const std::vector<float>& input) {
    if (input.size() != input_elements_)
      throw std::runtime_error("Preprocessed input tensor size mismatch");
    cuda_check(cudaSetDevice(device_), "cudaSetDevice");
    cuda_check(cudaMemcpyAsync(bindings_[input_index_], input.data(),
                               input_elements_ * sizeof(float), cudaMemcpyHostToDevice, stream_),
               "input copy");
    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr))
      throw std::runtime_error("TensorRT enqueueV2 failed");
    void* destination = output_half_ ? static_cast<void*>(half_output_.data())
                                     : static_cast<void*>(float_output_.data());
    cuda_check(cudaMemcpyAsync(destination, bindings_[output_index_], output_bytes_,
                               cudaMemcpyDeviceToHost, stream_), "output copy");
    cuda_check(cudaStreamSynchronize(stream_), "inference synchronization");
    return output_half_ ? classify(half_output_) : classify(float_output_);
  }

 private:
  void initialize(const std::string& path) {
    cuda_check(cudaSetDevice(device_), "cudaSetDevice");
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Cannot open TensorRT engine: " + path);
    const auto length = file.tellg();
    if (length <= 0) throw std::runtime_error("TensorRT engine is empty");
    std::vector<char> bytes(static_cast<std::size_t>(length));
    file.seekg(0);
    if (!file.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
      throw std::runtime_error("Could not read complete TensorRT engine");
    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) throw std::runtime_error("createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) throw std::runtime_error("Could not deserialize TensorRT engine");
    if (engine_->hasImplicitBatchDimension())
      throw std::runtime_error("Expected explicit-batch ONNX engine");
    if (engine_->getNbBindings() != 2)
      throw std::runtime_error("Expected exactly one input and one output binding");
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      if (engine_->isShapeBinding(i) || !engine_->isExecutionBinding(i))
        throw std::runtime_error("Shape-only bindings are unsupported");
      (engine_->bindingIsInput(i) ? input_index_ : output_index_) = i;
    }
    if (input_index_ < 0 || output_index_ < 0)
      throw std::runtime_error("Missing input or output binding");
    if (engine_->getBindingDataType(input_index_) != nvinfer1::DataType::kFLOAT)
      throw std::runtime_error("Expected float32 input binding");
    if (engine_->getBindingFormat(input_index_) != nvinfer1::TensorFormat::kLINEAR ||
        engine_->getBindingFormat(output_index_) != nvinfer1::TensorFormat::kLINEAR)
      throw std::runtime_error("Expected linear NCHW input/output tensor formats");
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("createExecutionContext failed");
    const auto expected = nvinfer1::Dims4{1, 3, kInputH, kInputW};
    auto input_shape = engine_->getBindingDimensions(input_index_);
    if (input_shape.nbDims != 4) throw std::runtime_error("Expected NCHW input");
    bool dynamic = false;
    for (int i = 0; i < 4; ++i) {
      if (input_shape.d[i] == -1) dynamic = true;
      else if (input_shape.d[i] != expected.d[i])
        throw std::runtime_error("Expected [1,3,512,512] input; got " + shape_string(input_shape));
    }
    if (dynamic && !context_->setBindingDimensions(input_index_, expected))
      throw std::runtime_error("Could not set [1,3,512,512] input dimensions");
    if (!context_->allInputDimensionsSpecified())
      throw std::runtime_error("Input dimensions remain unresolved");
    input_shape = context_->getBindingDimensions(input_index_);
    const auto output_shape = context_->getBindingDimensions(output_index_);
    if (output_shape.nbDims != 4 || output_shape.d[0] != 1 ||
        output_shape.d[1] < 2 || output_shape.d[1] > 256)
      throw std::runtime_error("Expected [1,C,H,W] scores with 2..256 classes; got " + shape_string(output_shape));
    input_elements_ = element_count(input_shape);
    output_elements_ = element_count(output_shape);
    classes_ = output_shape.d[1];
    output_h_ = output_shape.d[2];
    output_w_ = output_shape.d[3];
    const auto output_type = engine_->getBindingDataType(output_index_);
    if (output_type != nvinfer1::DataType::kFLOAT && output_type != nvinfer1::DataType::kHALF)
      throw std::runtime_error("Expected float32 or float16 output scores, not class-index tensor");
    output_half_ = output_type == nvinfer1::DataType::kHALF;
    output_bytes_ = output_elements_ * (output_half_ ? sizeof(std::uint16_t) : sizeof(float));
    if (output_half_) half_output_.resize(output_elements_);
    else float_output_.resize(output_elements_);
    bindings_.assign(2, nullptr);
    cuda_check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    cuda_check(cudaMalloc(&bindings_[input_index_], input_elements_ * sizeof(float)), "input allocation");
    cuda_check(cudaMalloc(&bindings_[output_index_], output_bytes_), "output allocation");
    RCLCPP_INFO(logger_, "FCN engine input=%s %s float32; output=%s %s %s; classes=%d",
                engine_->getBindingName(input_index_), shape_string(input_shape).c_str(),
                engine_->getBindingName(output_index_), shape_string(output_shape).c_str(),
                output_half_ ? "float16" : "float32", classes_);
  }

  template <class T> static float score(T value) { return static_cast<float>(value); }
  static float score(std::uint16_t value) { return half_to_float(value); }

  template <class T> cv::Mat classify(const std::vector<T>& scores) const {
    // Match the B2 comparison path: interpolate scores before selecting IDs.
    // Interpolating a finished class-ID mask would alter the decision rule.
    cv::Mat labels(kInputH, kInputW, CV_8UC1, cv::Scalar(0));
    cv::Mat best_scores(kInputH, kInputW, CV_32FC1,
                        cv::Scalar(-std::numeric_limits<float>::infinity()));
    cv::Mat native_plane(output_h_, output_w_, CV_32FC1);
    cv::Mat expanded_plane;
    const auto native_pixels = static_cast<std::size_t>(output_h_) * output_w_;
    const auto expanded_pixels = static_cast<std::size_t>(kInputH) * kInputW;
    auto* best = best_scores.ptr<float>();
    for (int class_id = 0; class_id < classes_; ++class_id) {
      auto* plane = native_plane.ptr<float>();
      const auto offset = static_cast<std::size_t>(class_id) * native_pixels;
      for (std::size_t pixel = 0; pixel < native_pixels; ++pixel) {
        const float value = score(scores[offset + pixel]);
        if (!std::isfinite(value))
          throw std::runtime_error("Non-finite output score; mask not published");
        plane[pixel] = value;
      }
      cv::resize(native_plane, expanded_plane, {kInputW, kInputH}, 0, 0, cv::INTER_LINEAR);
      const auto* expanded = expanded_plane.ptr<float>();
      for (std::size_t pixel = 0; pixel < expanded_pixels; ++pixel) {
        const float value = expanded[pixel];
        if (!std::isfinite(value))
          throw std::runtime_error("Non-finite interpolated score; mask not published");
        if (value > best[pixel]) {
          best[pixel] = value;
          labels.data[pixel] = static_cast<std::uint8_t>(class_id);
        }
      }
    }
    cv::Mat resized;
    cv::resize(labels, resized, {kOutputW, kOutputH}, 0, 0, cv::INTER_NEAREST);
    return resized;
  }

  void cleanup_cuda() noexcept {
    cudaSetDevice(device_);
    if (stream_) cudaStreamSynchronize(stream_);
    for (void* binding : bindings_) if (binding) cudaFree(binding);
    bindings_.clear();
    if (stream_) cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }

  rclcpp::Logger logger_;
  TrtLogger trt_logger_;
  int device_ = 0;
  TrtPtr<nvinfer1::IRuntime> runtime_;
  TrtPtr<nvinfer1::ICudaEngine> engine_;
  TrtPtr<nvinfer1::IExecutionContext> context_;
  int input_index_ = -1;
  int output_index_ = -1;
  int classes_ = 0;
  int output_h_ = 0;
  int output_w_ = 0;
  std::size_t input_elements_ = 0;
  std::size_t output_elements_ = 0;
  std::size_t output_bytes_ = 0;
  bool output_half_ = false;
  std::vector<void*> bindings_;
  std::vector<float> float_output_;
  std::vector<std::uint16_t> half_output_;
  cudaStream_t stream_ = nullptr;
};

std::vector<float> preprocess(const sensor_msgs::msg::Image& image) {
  int channels = 0;
  int conversion = -1;
  if (image.encoding == "rgb8") channels = 3;
  else if (image.encoding == "bgr8") { channels = 3; conversion = cv::COLOR_BGR2RGB; }
  else if (image.encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2RGB; }
  else if (image.encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2RGB; }
  else throw std::runtime_error("Unsupported image encoding: " + image.encoding);
  if (image.width == 0 || image.height == 0 ||
      image.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      image.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      static_cast<std::uint64_t>(image.step) < static_cast<std::uint64_t>(image.width) * channels ||
      static_cast<std::uint64_t>(image.data.size()) <
          static_cast<std::uint64_t>(image.step) * image.height)
    throw std::runtime_error("Invalid image dimensions, stride, or data length");
  // cv::resize reads the source; this const_cast never permits modification.
  cv::Mat source(static_cast<int>(image.height), static_cast<int>(image.width),
                 CV_MAKETYPE(CV_8U, channels),
                 const_cast<std::uint8_t*>(image.data.data()), image.step);
  cv::Mat resized, rgb;
  cv::resize(source, resized, {kInputW, kInputH}, 0, 0, cv::INTER_LINEAR);
  if (conversion >= 0) cv::cvtColor(resized, rgb, conversion);
  else rgb = resized;
  const auto plane = static_cast<std::size_t>(kInputW) * kInputH;
  std::vector<float> tensor(plane * 3);
  for (int y = 0; y < kInputH; ++y) {
    const auto* row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kInputW; ++x) {
      const auto offset = static_cast<std::size_t>(y) * kInputW + x;
      for (int channel = 0; channel < 3; ++channel)
        tensor[static_cast<std::size_t>(channel) * plane + offset] =
            (static_cast<float>(row[x][channel]) / 255.0F - kMean[channel]) / kStd[channel];
    }
  }
  return tensor;
}

class FcnNode final : public rclcpp::Node {
 public:
  FcnNode() : Node("nomad_fcn_preview") {
    const auto path = declare_parameter<std::string>("engine_path", "");
    const auto input = declare_parameter<std::string>("input_topic", "/zed/zed_node/left/image_rect_color");
    const auto output = declare_parameter<std::string>("output_topic", "/nomad_preview/sunrgbd/class_ids");
    const auto field_output = declare_parameter<std::string>("field12_output_topic", "/nomad_preview/sunrgbd/field12_raw");
    const auto classes_path = declare_parameter<std::string>("classes_path", "");
    const auto mapping_path = declare_parameter<std::string>("mapping_path", "");
    const auto rate = declare_parameter<double>("rate_hz", 1.0);
    const auto device = declare_parameter<int>("device_id", 0);
    const auto expected_classes = declare_parameter<int>("expected_classes", kSourceClasses);
    if (path.empty()) throw std::runtime_error("engine_path parameter is required");
    if (classes_path.empty() || mapping_path.empty())
      throw std::runtime_error("classes_path and mapping_path parameters are required");
    if (output == field_output)
      throw std::runtime_error("Raw source IDs and field12 IDs must use different topics");
    if (!std::isfinite(rate) || rate <= 0.0 || rate > 100.0)
      throw std::runtime_error("rate_hz must be finite and in (0,100]");
    if (device < 0 || expected_classes != kSourceClasses)
      throw std::runtime_error("Invalid device_id or expected_classes");
    const auto labels = read_classes(classes_path);
    field12_lut_ = read_mapping(mapping_path, labels);
    engine_ = std::make_unique<FcnEngine>(path, device, get_logger());
    if (engine_->class_count() != expected_classes)
      throw std::runtime_error("Engine class count differs from expected_classes");
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output, qos);
    field_publisher_ = create_publisher<sensor_msgs::msg::Image>(field_output, qos);
    receive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inference_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = receive_group_;
    subscription_ = create_subscription<sensor_msgs::msg::Image>(input, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
          std::lock_guard<std::mutex> guard(latest_mutex_);
          latest_ = std::move(image);
          ++received_sequence_;
        }, subscription_options);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, [this]() { process_latest(); }, inference_group_);
    RCLCPP_INFO(get_logger(), "Waiting for RGB on %s; raw SUNRGB-D IDs -> %s (640x360 mono8, %.3g Hz maximum). No camera device opened.",
                input.c_str(), output.c_str(), rate);
    RCLCPP_INFO(get_logger(), "Validated SUNRGB-D21 -> field12 mapping: %s; final IDs -> %s. Existing B2 adapter must be stopped while this publisher is active.",
                mapping_path.c_str(), field_output.c_str());
  }

 private:
  void process_latest() {
    sensor_msgs::msg::Image::ConstSharedPtr source;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> guard(latest_mutex_);
      if (!latest_ || received_sequence_ == processed_sequence_) return;
      source = latest_;
      sequence = received_sequence_;
    }
    // One cached frame and one in-flight inference; never replay old frames.
    processed_sequence_ = sequence;
    try {
      const auto begin = std::chrono::steady_clock::now();
      const auto tensor = preprocess(*source);
      const auto prepared = std::chrono::steady_clock::now();
      const auto ids = engine_->infer(tensor);
      const auto finished = std::chrono::steady_clock::now();
      sensor_msgs::msg::Image message;
      message.header = source->header;  // Original capture time and optical frame.
      message.height = kOutputH;
      message.width = kOutputW;
      message.encoding = "mono8";
      message.is_bigendian = 0;
      message.step = kOutputW;
      message.data.assign(ids.data, ids.data + static_cast<std::size_t>(kOutputW) * kOutputH);
      auto field_message = message;
      for (auto& id : field_message.data) {
        if (id >= field12_lut_.size())
          throw std::runtime_error("Unexpected raw ID outside validated SUNRGB-D mapping");
        id = field12_lut_[id];
      }
      publisher_->publish(std::move(message));
      field_publisher_->publish(std::move(field_message));
      const auto prep_ms = std::chrono::duration<double, std::milli>(prepared - begin).count();
      const auto inference_and_argmax_ms =
          std::chrono::duration<double, std::milli>(finished - prepared).count();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Published raw FCN and field12 IDs; preprocess=%.1f ms, TensorRT+copies+argmax=%.1f ms, capture=%d.%09u frame=%s",
          prep_ms, inference_and_argmax_ms, source->header.stamp.sec,
          source->header.stamp.nanosec, source->header.frame_id.c_str());
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                           "FCN frame rejected, no mask published: %s", error.what());
    }
  }

  std::unique_ptr<FcnEngine> engine_;
  std::mutex latest_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  std::uint64_t received_sequence_ = 0;
  std::uint64_t processed_sequence_ = 0;
  std::array<std::uint8_t, kSourceClasses> field12_lut_{};
  rclcpp::CallbackGroup::SharedPtr receive_group_;
  rclcpp::CallbackGroup::SharedPtr inference_group_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr field_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<FcnNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("nomad_fcn_preview"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

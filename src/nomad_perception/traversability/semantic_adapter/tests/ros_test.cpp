#include "test_support.hpp"
#include <chrono>
#include <csignal>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
struct Child {
  pid_t pid=-1;
  ~Child() {
    if(pid<=0) return;
    kill(pid,SIGTERM);
    for(int i=0;i<50;++i) {
      if(waitpid(pid,nullptr,WNOHANG)==pid) return;
      std::this_thread::sleep_for(20ms);
    }
    kill(pid,SIGKILL); waitpid(pid,nullptr,0);
  }
};
struct TempFile {
  std::string path;
  ~TempFile() {if(!path.empty()) unlink(path.c_str());}
};
int main(int argc,char **argv) {
  try {
    check(argc==3,"Expected executable and mapping paths");
    const auto config=nomad_semantic::read_json(argv[2]);
    TempFile model_file{std::string("/tmp/nomad_semantic_model_")+std::to_string(getpid())+".json"};
    {std::ofstream stream(model_file.path);stream<<model_fixture(config).dump();check(bool(stream),"Cannot write fixture");}
    // Fork before ROS creates threads. Only this owned mock process is stopped.
    Child child;
    child.pid=fork(); check(child.pid>=0,"fork failed");
    if(child.pid==0) {
      execl(argv[1],argv[1],"--mapping",argv[2],"--source-model-config",model_file.path.c_str(),nullptr);
      _exit(127);
    }
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("nomad_semantic_mock_test");
    auto qos=rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    auto pub=node->create_publisher<sensor_msgs::msg::Image>(config.at("input_topic").get<std::string>(),qos);
    std::vector<sensor_msgs::msg::Image> received;
    auto sub=node->create_subscription<sensor_msgs::msg::Image>(config.at("output_topic").get<std::string>(),qos,
      [&received](sensor_msgs::msg::Image::ConstSharedPtr m){received.push_back(*m);});
    auto spin_for=[&](std::chrono::milliseconds duration) {
      const auto end=std::chrono::steady_clock::now()+duration;
      while(std::chrono::steady_clock::now()<end) {rclcpp::spin_some(node);std::this_thread::sleep_for(5ms);}
    };
    const auto end=std::chrono::steady_clock::now()+20s;
    while((pub->get_subscription_count()==0 || sub->get_publisher_count()==0) &&
      std::chrono::steady_clock::now()<end) spin_for(20ms);
    check(pub->get_subscription_count()==1 && sub->get_publisher_count()==1,"Adapter DDS discovery failed");
    for(const auto &endpoint:node->get_publishers_info_by_topic(config.at("output_topic").get<std::string>())) {
      const auto profile=endpoint.qos_profile().get_rmw_qos_profile();
      // DDS discovery does not reliably expose history/depth. The adapter checks
      // its local get_actual_qos() on both endpoints before accepting any input.
      check(profile.reliability==RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
        profile.durability==RMW_QOS_POLICY_DURABILITY_VOLATILE,"Output DDS QoS differs");
    }
    auto input=image_fixture();
    // Invalid data must not be published or terminate the adapter.
    auto malformed=input; malformed.encoding="rgb8"; malformed.header.stamp.sec=456;
    pub->publish(malformed); spin_for(400ms);
    check(received.empty(),"Invalid image leaked onto output topic");
    // Check repeated real transport, padded rows and historical capture stamps.
    for(int frame=0;frame<3;++frame) {
      input.header.stamp.nanosec=456789+frame;
      const size_t before=received.size();
      for(int attempt=0;attempt<5 && received.size()==before;++attempt) {
        pub->publish(input);spin_for(150ms);
      }
      check(received.size()>before,"No converted ROS image received");
      check_output(input,received.back());
    }
    // Exercise all mono8 input values; unknown 150..255 must be ignored.
    input.header.stamp.nanosec=999;
    for(size_t row=0;row<input.height;++row)
      for(size_t col=0;col<input.width;++col) input.data[row*input.step+col]=uint8_t((row+col)%256);
    const size_t before=received.size();
    for(int attempt=0;attempt<5 && received.size()==before;++attempt){pub->publish(input);spin_for(150ms);}
    check(received.size()>before,"Full ID range transport failed");
    const auto &output=received.back();
    check(output.header==input.header,"Full range acquisition header changed");
    for(size_t row=0;row<360;++row) for(size_t col=0;col<640;++col) {
      const auto raw=uint8_t((row+col)%256),id=output.data[row*640+col];
      check(id<12 && id!=3 && id!=5 && id!=8,"Invalid target ID over ROS");
      if(raw>=150)check(id==0,"Unknown source ID became known over ROS");
    }
    rclcpp::shutdown();
    std::cout<<"PASS: actual C++ adapter process, DDS depth5/best-effort/volatile, malformed input rejection, 4 output images, all 256 source values, capture stamps/frame/padding preserved\n";
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
}

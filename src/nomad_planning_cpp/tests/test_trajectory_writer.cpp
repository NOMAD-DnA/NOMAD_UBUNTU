#include "nomad_planning_cpp/trajectory_writer.hpp"
#include <atomic>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
using namespace std::chrono_literals;
static void require(bool ok) { if (!ok) { std::cerr << "trajectory test failed\n"; std::exit(1); } }
static std::string read(const std::string& p) {std::ifstream f(p);std::ostringstream s;s<<f.rdbuf();return s.str();}
int main() {
  const std::string path="/tmp/nomad_trajectory_test_"+std::to_string(::getpid());
  std::atomic<int> sampled{0}, errors{0};
  const auto caller=std::this_thread::get_id(); std::atomic<bool> background{false};
  {
    nomad_planning_cpp::TrajectoryWriter writer(path,[&]{background=std::this_thread::get_id()!=caller;++sampled;return "{\"kind\":\"pose\",\"t_ns\":1794273405174123456,\"x\":1,\"y\":2}";},[&](const std::string&){++errors;});
    for(int i=0;i<100&&sampled.load()<3;++i)std::this_thread::sleep_for(10ms);
    writer.stop();
  }
  const auto content=read(path);
  require(background&&sampled>=3&&sampled<=12&&errors==0);
  require(content.find("1794273405174123456")!=std::string::npos);
  require(content.find("\"event\":\"started\"")!=std::string::npos&&content.find("\"event\":\"closed\"")!=std::string::npos);
  bool rejected=false;
  try {nomad_planning_cpp::TrajectoryWriter writer(path,[]{return "";},[](const std::string&){});}catch(...){rejected=true;}
  require(rejected&&read(path)==content);::unlink(path.c_str());
  {
    nomad_planning_cpp::TrajectoryWriter writer(path,[]{return std::string(2048,'x');},[&](const std::string&){++errors;},1024);
    for(int i=0;i<100&&!errors;++i)std::this_thread::sleep_for(10ms);
    writer.stop();
  }
  require(errors==1&&read(path).size()<1024);::unlink(path.c_str());
  require(::symlink("/dev/null",path.c_str())==0);rejected=false;
  try {nomad_planning_cpp::TrajectoryWriter writer(path,[]{return "";},[](const std::string&){});}catch(...){rejected=true;}
  require(rejected);::unlink(path.c_str());
  std::cout<<"trajectory sampling / persistence / exclusive creation / size failure PASS\n";
}

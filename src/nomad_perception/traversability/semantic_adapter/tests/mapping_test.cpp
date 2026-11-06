#include "test_support.hpp"
#include <functional>

int main(int argc, char **argv) {
  try {
    check(argc==2, "Expected mapping path");
    auto config=nomad_semantic::read_json(argv[1]);
    auto model=model_fixture(config);
    auto mapping=nomad_semantic::Mapping::load(config, model);
    auto input=image_fixture(); sensor_msgs::msg::Image output; std::string error;
    check(mapping.convert(input,output,error),error); check_output(input,output);
    for (auto id : mapping.lookup) check(id<12 && id!=3 && id!=5 && id!=8, "Unsupported output ID");
    for (size_t i=150; i<256; ++i) check(mapping.lookup[i]==0,"Unknown source became known");
    for (const auto &change : std::vector<std::function<void(sensor_msgs::msg::Image&)>>{
        [](auto &m){m.encoding="rgb8";}, [](auto &m){m.width=1280;},
        [](auto &m){m.height=0;}, [](auto &m){m.header.frame_id="base_link";},
        [](auto &m){m.header.stamp.sec=0;m.header.stamp.nanosec=0;},
        [](auto &m){m.header.stamp.sec=-1;}, [](auto &m){m.header.stamp.nanosec=1000000000u;},
        [](auto &m){m.step=639;}, [](auto &m){m.data.pop_back();},
        [](auto &m){m.step=UINT32_MAX;}, [](auto &m){m.is_bigendian=2;}}) {
      auto broken=input; change(broken);
      check(!mapping.convert(broken,output,error) && !error.empty(),"Malformed image accepted");
    }
    for (const auto &change : std::vector<std::function<void(nomad_semantic::Json&)>>{
        [](auto &c){c["classes"][1]["source_id"]=0;},
        [](auto &c){c["classes"][0]["target_id"]=12;},
        [](auto &c){c["classes"][0]["target_id"]=-1;},
        [](auto &c){c["classes"][0]["target_id"]=5;},
        [](auto &c){c["classes"][0]["target_id"]=1.5;},
        [](auto &c){c["classes"][0]["source_name"]="sky";},
        [](auto &c){c["unknown_target_id"]=6;},
        [](auto &c){c["project_classes"][0]="paved";},
        [](auto &c){c["output_topic"]=c["input_topic"];},
        [](auto &c){c["classes"].erase(c["classes"].begin());}}) {
      auto broken=config; change(broken); bool rejected=false;
      try { (void)nomad_semantic::Mapping::load(broken,model); }
      catch (const std::exception&) {rejected=true;}
      check(rejected,"Malformed / incompatible mapping accepted");
    }
    model["id2label"]["0"]="sky";
    bool rejected=false;
    try {(void)nomad_semantic::Mapping::load(config,model);} catch(const std::exception&){rejected=true;}
    check(rejected,"Model schema mismatch accepted");
    std::cout<<"PASS: pixel categories, padding, acquisition header, unknown IDs, 11 malformed images, 10 bad mappings, source model mismatch\n";
    return 0;
  } catch (const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}

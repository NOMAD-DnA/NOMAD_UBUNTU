// Read-only Linux resource sampling. No ROS, device control, or shell commands.
#pragma once
#include <cmath>
#include <limits>
#include <set>
#include <sys/utsname.h>

namespace resources {
static std::string read(const fs::path& p, std::size_t cap=8192) {
  std::ifstream f(p, std::ios::binary); std::string s(cap, '\0');
  if (!f) return {};
  f.read(s.data(), s.size()); s.resize(f.gcount()); return s;
}
static double number(const fs::path& p) {
  try { return std::stod(read(p)); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
}
static std::string numeric(double x) {
  if (!std::isfinite(x)) return "null";
  std::ostringstream o; o << std::setprecision(10) << x; return o.str();
}
struct Cpu { std::uint64_t total=0, idle=0; };
static std::map<std::string,Cpu> cpus() {
  std::map<std::string,Cpu> result; std::istringstream in(read("/proc/stat", 65536)); std::string line;
  while(std::getline(in,line)) {
    std::istringstream row(line); std::string name; row >> name;
    if(name.compare(0,3,"cpu")) continue;
    Cpu c; std::uint64_t v; unsigned i=0;
    // guest/guest_nice are already included in user/nice; never double count.
    while(row >> v) { if(i<8)c.total+=v; if(i==3||i==4)c.idle+=v; ++i; }
    if(i>=4)result[name]=c;
  } return result;
}
static double busy(const Cpu& a,const Cpu& b) {
  if(b.total<=a.total||b.idle<a.idle) return NAN;
  const auto total=b.total-a.total, idle=b.idle-a.idle;
  if(idle>total)return NAN;
  return 100.*(total-idle)/total;
}
struct Proc {
  int pid=0, ppid=0, threads=0; std::uint64_t start=0, ticks=0;
  double rss=0, percent=NAN; std::string name, group;
};
static std::string group_for(const std::vector<std::string>& args,int pid) {
  if(pid==::getpid())return "measurement";
  for(const auto& s:args) {
    const std::string n=fs::path(s).filename().string();
    if(n.find("nomad_plann")==0||n=="planner_node")return "planning_gpp_lpp";
    if(n.find("nomad_control")==0||n.find("nomad_vio_duty")==0||n=="controller_node")return "control";
    if(n.find("vesc")==0)return "vesc_driver";
    if(n.find("slope_worker")!=std::string::npos)return "slope";
    if(n=="nomad_mapper"||n=="nomad_runtime_manager"||n=="nomad_monitor")return "mapping";
    if(n.find("semantic")!=std::string::npos||n.find("nomad_fcn")==0)return "semantic";
    if(n=="external_stereo_cloud")return "stereo_cloud";
    if(n=="oak_publisher"||n=="zed_wrapper"||n=="zed_camera"||n=="zed_components")return "camera";
    if(n.find("ydlidar")==0)return "lidar";
    if(n=="module_log_tail"||n=="monitor_probe"||n=="slope_control.py")return "measurement";
    if(n.find("nomad_")==0)return "other_nomad";
  } return "other";
}
static std::map<int,Proc> processes(unsigned& failures, const std::map<int,Proc>& previous) {
  std::map<int,Proc> out; std::error_code ec; const long pages=::sysconf(_SC_PAGESIZE);
  for(fs::directory_iterator it("/proc",ec),end;!ec&&it!=end;it.increment(ec)) {
    const auto n=it->path().filename().string();
    if(n.empty()||n.find_first_not_of("0123456789")!=std::string::npos)continue;
    const auto raw=read(it->path()/"stat"); const auto l=raw.find('('), r=raw.rfind(')');
    if(l==std::string::npos||r==std::string::npos) { ++failures; continue; }
    std::istringstream fields(raw.substr(r+2)); std::vector<std::string> f; std::string s;
    while(fields>>s)f.push_back(s);
    if(f.size()<22){++failures;continue;}
    try {
      Proc p; p.pid=std::stoi(n);p.ppid=std::stoi(f[1]);p.name=raw.substr(l+1,r-l-1);
      p.ticks=std::stoull(f[11])+std::stoull(f[12]);p.threads=std::stoi(f[17]);p.start=std::stoull(f[19]);
      p.rss=std::max(0LL,std::stoll(f[21]))*double(pages)/(1024.*1024.);
      const auto old=previous.find(p.pid);
      if(old!=previous.end()&&old->second.start==p.start) p.group=old->second.group;
      else {
        auto cmd=read(it->path()/"cmdline");std::vector<std::string> args;
        std::istringstream tokens(cmd);while(std::getline(tokens,s,'\0'))args.push_back(s);
        p.group=group_for(args,p.pid);
      }
      out[p.pid]=p;
    } catch (...) { ++failures; }
  } return out;
}
static std::map<std::string,double> memory() {
  std::map<std::string,double> m; std::istringstream in(read("/proc/meminfo",32768));std::string line;
  while(std::getline(in,line)) { std::istringstream row(line);std::string key;double v;
    if(row>>key>>v){if(key.back()==':')key.pop_back();m[key]=v/1024.;} }
  return m;
}
class Sampler {
  std::map<std::string,Cpu> last_cpu;
  std::map<int,Proc> last_proc;
  Clock::time_point last_time{};
  std::string boot, model, kernel;
  long ticks;
  std::uint64_t sequence=0;
  // 2026-11-13: the Jetson GPU 'load' file is a short-window value; one read per second mostly hit idle
  // gaps between 1 Hz inferences (always ~0). gpu_tick() is called ~20x/s between samples; sample()
  // reports the mean (load_percent, same field as before) plus max_percent and the read count.
  std::string gpu_load_path;bool gpu_path_checked=false;
  double gpu_sum=0,gpu_max=NAN;unsigned gpu_count=0;
  const std::string& gpu_path(const std::string& board) {
    if(!gpu_path_checked&&board=="jetson") {
      gpu_path_checked=true;
      for(const auto* base:{"/sys/devices/gpu.0","/sys/devices/17000000.gv11b"})
        if(std::isfinite(number(fs::path(base)/"load"))){gpu_load_path=std::string(base)+"/load";break;}
    }
    return gpu_load_path;
  }
public:
  void gpu_tick(const std::string& board) {
    const auto& path=gpu_path(board);if(path.empty())return;
    const double v=number(path);if(!std::isfinite(v)||v<0||v>1000)return;  // per-mille; out of range ignored
    gpu_sum+=v/10.;++gpu_count;gpu_max=std::isfinite(gpu_max)?std::max(gpu_max,v/10.):v/10.;
  }
  Sampler():ticks(::sysconf(_SC_CLK_TCK)) {
    boot=first_line("/proc/sys/kernel/random/boot_id");model=read("/proc/device-tree/model",256);
    model.erase(std::remove(model.begin(),model.end(),'\0'),model.end());
    struct utsname u{};if(!::uname(&u))kernel=std::string(u.release)+" "+u.machine;
  }
  std::string sample(const std::string& board,const std::string& session) {
    const auto begin=Clock::now();const auto current=cpus();unsigned failures=0;
    auto ps=processes(failures,last_proc);const auto mem=memory();
    const double dt=last_time==Clock::time_point{}?NAN:std::chrono::duration<double>(begin-last_time).count();
    const bool valid=std::isfinite(dt)&&dt>0&&dt<=5&&current.count("cpu")&&last_cpu.count("cpu");
    for(auto& [pid,p]:ps) {
      auto prev=last_proc.find(pid);
      if(valid&&ticks>0&&prev!=last_proc.end()&&prev->second.start==p.start&&p.ticks>=prev->second.ticks)
        p.percent=100.*double(p.ticks-prev->second.ticks)/ticks/dt;
    }
    std::vector<Proc> selected;std::set<int> chosen;
    // Preserve known modules and the sampler, plus the busiest OS processes.
    for(const auto& [pid,p]:ps)if(p.group!="other"&&selected.size()<48){selected.push_back(p);chosen.insert(pid);}
    std::vector<Proc> all;for(const auto& [pid,p]:ps)all.push_back(p);
    std::sort(all.begin(),all.end(),[](const Proc&a,const Proc&b){
      const double x=std::isfinite(a.percent)?a.percent:-1,y=std::isfinite(b.percent)?b.percent:-1;
      return x!=y?x>y:a.pid<b.pid;});
    for(const auto& p:all)if(!chosen.count(p.pid)&&selected.size()<64){selected.push_back(p);chosen.insert(p.pid);}
    auto m=[&](const std::string& key){auto it=mem.find(key);return it==mem.end()?NAN:it->second;};
    auto freq=[](const std::string& id){return number("/sys/devices/system/cpu/"+id+"/cpufreq/scaling_cur_freq")/1000.;};
    std::ostringstream o;o<<std::setprecision(17)<<"{\"kind\":\"telemetry\",\"schema\":1,\"source\":"<<quote(board=="pi"?std::string("resources_pi5"):"resources_"+board)
      <<",\"board\":"<<quote(board)<<",\"t\":"<<std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
      <<",\"sample_monotonic_s\":"<<std::chrono::duration<double>(begin.time_since_epoch()).count()
      <<",\"session\":"<<quote(session)<<",\"seq\":"<<++sequence<<",\"boot_id\":"<<quote(boot)
      <<",\"model\":"<<quote(model)<<",\"kernel\":"<<quote(kernel)<<",\"cpu\":{\"state\":"<<quote(valid?"live":"warming_up")
      <<",\"interval_s\":"<<numeric(dt)<<",\"online_cores\":"<<(current.empty()?0:current.size()-1)
      <<",\"total_percent\":"<<numeric(valid?busy(last_cpu.at("cpu"),current.at("cpu")):NAN)<<",\"cores\":[";
    bool comma=false;for(const auto& [id,c]:current)if(id!="cpu") {
      if(comma)o<<',';
      comma=true;
      o<<"{\"id\":"<<quote(id)<<",\"percent\":"<<numeric(valid&&last_cpu.count(id)?busy(last_cpu.at(id),c):NAN)
        <<",\"frequency_mhz\":"<<numeric(freq(id))<<'}';
    }
    o<<"]},\"memory\":{\"total_mib\":"<<numeric(m("MemTotal"))<<",\"available_mib\":"<<numeric(m("MemAvailable"))
      <<",\"used_mib\":"<<numeric(m("MemTotal")-m("MemAvailable"))<<",\"swap_total_mib\":"<<numeric(m("SwapTotal"))
      <<",\"swap_used_mib\":"<<numeric(m("SwapTotal")-m("SwapFree"))<<"},\"gpu\":{\"load_percent\":";
    double gpu=NAN,gpu_freq=NAN,gpu_peak=NAN;unsigned gpu_reads=0;
    if(board=="jetson") {
      gpu_tick(board);  // at least the read at sample time
      if(gpu_count>0){gpu=gpu_sum/gpu_count;gpu_peak=gpu_max;gpu_reads=gpu_count;}
      gpu_sum=0;gpu_max=NAN;gpu_count=0;
    }
    if(board=="jetson") {
      std::error_code ec;for(fs::directory_iterator it("/sys/class/devfreq",ec),end;!ec&&it!=end;it.increment(ec))
        if(it->path().filename().string().find("gv11b")!=std::string::npos){gpu_freq=number(it->path()/"cur_freq")/1e6;break;}
    }
    o<<numeric(gpu)<<",\"frequency_mhz\":"<<numeric(gpu_freq)<<",\"max_percent\":"<<numeric(gpu_peak)
     <<",\"reads\":"<<gpu_reads<<"},\"temperatures\":[";comma=false;
    std::error_code ec;unsigned zones=0;
    for(fs::directory_iterator it("/sys/class/thermal",ec),end;!ec&&it!=end&&zones<32;it.increment(ec)) {
      if(it->path().filename().string().find("thermal_zone")!=0)continue;
      const auto c=number(it->path()/"temp")/1000.;if(!std::isfinite(c)||c< -50||c>200)continue;
      if(comma)o<<',';
      comma=true;++zones;o<<"{\"name\":"<<quote(first_line(it->path()/"type"))<<",\"celsius\":"<<numeric(c)<<'}';
    }
    o<<"],\"processes\":[";comma=false;
    for(const auto&p:selected) {
      if(comma)o<<',';
      comma=true;
      o<<"{\"pid\":"<<p.pid<<",\"ppid\":"<<p.ppid<<",\"start_ticks\":"<<p.start<<",\"name\":"<<quote(p.name)
        <<",\"group\":"<<quote(p.group)<<",\"cpu_percent_one_core\":"<<numeric(p.percent)
        <<",\"rss_mib\":"<<numeric(p.rss)<<",\"threads\":"<<p.threads<<'}';
    }
    const double scan=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
    o<<"],\"processes_total\":"<<ps.size()<<",\"processes_omitted\":"<<(ps.size()-selected.size())
      <<",\"proc_read_failures\":"<<failures<<",\"scan_ms\":"<<numeric(scan)<<"}\n";
    last_cpu=current;last_proc=std::move(ps);last_time=begin;
    const auto frame=o.str();return frame.size()<=32768?frame:std::string{};
  }
};
} // namespace resources

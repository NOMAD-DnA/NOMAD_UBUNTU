// External read-only log tail. No ROS, sensor, module-control or motor API.
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t alive = 1;
static void stop(int) { alive = 0; }
static std::string quote(const std::string& s) {
  std::ostringstream o; o << '"';
  for (std::size_t i=0; i<s.size(); ++i) {
    const unsigned char c = s[i];
    if (c == '\\' || c == '"') o << '\\' << char(c);
    else if (c == '\n') o << "\\n";
    else if (c == '\r') o << "\\r";
    else if (c == '\t') o << "\\t";
    else if (c < 32 || c == 127) o << '?';
    else if (c < 128) o << char(c);
    else {
      // Logs may contain arbitrary bytes; byte/line truncation may split UTF-8.
      // Preserve valid text and replace invalid bytes so every JSON frame parses.
      const std::size_t n=c>=0xc2&&c<=0xdf?2:c>=0xe0&&c<=0xef?3:c>=0xf0&&c<=0xf4?4:0;
      bool valid=n && i+n<=s.size();
      for(std::size_t k=1;valid&&k<n;++k)valid=(static_cast<unsigned char>(s[i+k])&0xc0)==0x80;
      if(valid&&n>=3){const auto b=static_cast<unsigned char>(s[i+1]);valid=!(c==0xe0&&b<0xa0)&&!(c==0xed&&b>=0xa0)&&!(c==0xf0&&b<0x90)&&!(c==0xf4&&b>=0x90);}
      if(valid){o.write(s.data()+i,n);i+=n-1;}else o << '?';
    }
  }
  return o.str() + '"';
}
static std::string first_line(const fs::path& p) {
  std::ifstream f(p); std::string s; std::getline(f,s); return s;
}
static fs::path newest(const fs::path& dir, const std::string& prefix, const std::string& suffix = "", bool directory = false) {
  fs::path best; fs::file_time_type time = fs::file_time_type::min(); std::error_code ec;
  for (fs::directory_iterator it(dir,ec), end; !ec && it != end; it.increment(ec)) {
    auto name = it->path().filename().string();
    if (name.compare(0,prefix.size(),prefix) || (suffix.size() && (name.size()<suffix.size() || name.compare(name.size()-suffix.size(),suffix.size(),suffix)))) continue;
    if (directory ? !it->is_directory(ec) : !it->is_regular_file(ec)) continue;
    auto current = it->last_write_time(ec);
    if (!ec && current > time) { time = current; best = it->path(); }
  }
  return best;
}
static bool within(const fs::path& p, const fs::path& root) {
  std::error_code ec; auto real = fs::weakly_canonical(p,ec); if(ec) return false;
  auto base = fs::weakly_canonical(root,ec); if(ec) return false;
  auto a=real.begin(),b=base.begin(); for(;b!=base.end();++b,++a) if(a==real.end()||*a!=*b)return false;
  return true;
}
struct File { std::string group, name; fs::path path; };
struct Cursor { ino_t inode=0; dev_t device=0; off_t offset=0; std::string pending; bool initialized=false; };
static std::string tail(const File& f, Cursor& c, std::size_t& budget) {
  const auto cap=std::min<std::size_t>(1024,budget);if(!cap)return {};
  struct stat st{}; if(::stat(f.path.c_str(),&st) || !S_ISREG(st.st_mode)) return {};
  if(!c.initialized || c.inode!=st.st_ino || c.device!=st.st_dev || st.st_size<c.offset) {
    c={st.st_ino,st.st_dev,std::max<off_t>(0,st.st_size-1536),{},true};
    if(c.offset>0) c.pending="[older log omitted]\n";
  }
  bool skipped=false;
  if(st.st_size-c.offset>static_cast<off_t>(cap)) { c.offset=st.st_size-cap; c.pending.clear(); skipped=true; }
  std::ifstream in(f.path,std::ios::binary); if(!in)return {};
  in.seekg(c.offset); std::string text(std::min<std::size_t>(cap,st.st_size-c.offset),'\0');
  in.read(text.data(),text.size()); text.resize(in.gcount()); c.offset += text.size();
  budget-=text.size();
  if(skipped || (c.offset-static_cast<off_t>(text.size())>0 && c.pending=="[older log omitted]\n")) {
    const auto end=text.find('\n'); text=end==std::string::npos ? std::string{} : text.substr(end+1);
  }
  text=c.pending+text; c.pending.clear();
  // Do not emit partial lines: especially important for source code's buffered stdout.
  const auto complete=text.find_last_of('\n');
  if(complete==std::string::npos){c.pending=text.substr(text.size()>1024?text.size()-1024:0);return {};}
  c.pending=text.substr(complete+1); if(c.pending.size()>1024)c.pending=c.pending.substr(c.pending.size()-1024);
  text.resize(complete+1);
  std::string out=skipped?"[older log omitted]\n":"";
  std::istringstream lines(text); std::string line; unsigned count=0;
  while(std::getline(lines,line) && count++<24) { if(line.size()>400)line=line.substr(0,400)+" …"; out+='['+f.name+"] "+line+'\n'; }
  return out;
}
static std::vector<File> files(const std::string& board,const fs::path& root,const std::string& pointer) {
  std::vector<File> out; auto add=[&](std::string group,std::string name,fs::path path){if(!path.empty()&&within(path,root))out.push_back({group,name,path});};
  fs::path run;
  if(board=="pi4") run=newest(root/"log","planning_output_","",true);
  else run=first_line(pointer);
  if(run.empty() || !within(run,root/"log")) return out;
  if(board=="pi") for(auto name:{"gnss","vio","fusion"})add("localization",name,run/(std::string(name)+".log"));
  if(board=="jetson") {
    add("localization","oak",run/"oak.log");
    add("perception","zed",run/"zed.log");
    auto semantic=run/"semantic.log", fcn=newest(run,"semantic_fcn_",".log");
    std::error_code ec; if(!fcn.empty() && (!fs::exists(semantic,ec) || fs::last_write_time(fcn,ec)>fs::last_write_time(semantic,ec)))semantic=fcn;
    add("perception","semantic",semantic);add("perception","semantic_adapter",run/"semantic_adapter.log");
    for(auto name:{"lidar","external_cloud","frame_alias"})add("traversability",name,run/(std::string(name)+".log"));
    const auto id=first_line(run/"costmap_run_id");
    if(!id.empty()&&id!="."&&id!=".."&&id.find('/')==std::string::npos)for(auto name:{"mapper","monitor","supervisor"})add("traversability",name,run/"costmap_runs"/id/(std::string(name)+".log"));
  }
  if(board=="pi4")for(auto name:{"planner","controller","driver"})add("planning",name,run/(std::string(name)+".log"));
  return out;
}
static bool write_frame(const std::string& line) {
  std::size_t sent=0;const auto deadline=Clock::now()+std::chrono::milliseconds(200);
  while(alive&&sent<line.size()&&Clock::now()<deadline) {
    pollfd p{STDOUT_FILENO,POLLOUT,0}; if(::poll(&p,1,20)<0&&errno!=EINTR)return false;
    if(p.revents&(POLLERR|POLLHUP|POLLNVAL))return false;
    if(!(p.revents&POLLOUT))continue;
    const auto n=::write(STDOUT_FILENO,line.data()+sent,std::min<std::size_t>(4096,line.size()-sent));
    if(n>0)sent+=n;else if(n<0&&errno!=EINTR&&errno!=EAGAIN)return false;
  }
  // A partial JSON packet cannot be replaced: close instead of mixing fragments.
  return sent==line.size();
}
int main(int argc,char**argv) {
  std::string board, pointer; fs::path root;bool once=false;
  for(int i=1;i<argc;++i){const std::string a=argv[i];if(a=="--board"&&i+1<argc)board=argv[++i];else if(a=="--root"&&i+1<argc)root=argv[++i];else if(a=="--pointer"&&i+1<argc)pointer=argv[++i];else if(a=="--once")once=true;else return 2;}
  if(board!="pi"&&board!="jetson"&&board!="pi4")return 2;
  if(root.empty())root=board=="jetson"?"/home/nvidia/nomad_ws":board=="pi4"?"/home/user/nomad_cpp_ws":"/home/user/nomad_ws";
  if(pointer.empty())pointer=board=="jetson"?"/tmp/nomad_integrated_jetson_run_dir":"/tmp/nomad_integrated_pi_run_dir";
  std::signal(SIGTERM,stop);std::signal(SIGINT,stop);std::signal(SIGPIPE,SIG_IGN);
  const int flags=::fcntl(STDOUT_FILENO,F_GETFL,0);
  if(flags<0 || ::fcntl(STDOUT_FILENO,F_SETFL,flags|O_NONBLOCK)<0)return 2;
  const auto session=std::to_string(::getpid())+"-"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  std::map<std::string,Cursor> cursors;std::map<std::string,bool> announced;std::uint64_t seq=0;std::size_t first_file=0;
  do {
    const auto next_tick=Clock::now()+std::chrono::seconds(1);
    std::map<std::string,std::string> groups;const auto current=files(board,root,pointer);
    const std::vector<std::string> names=board=="pi"?std::vector<std::string>{"localization"}:board=="pi4"?std::vector<std::string>{"planning"}:std::vector<std::string>{"localization","perception","traversability"};
    for(const auto& group:names)if(!announced[group])groups[group]="[log] Existing module stdout/stderr; read-only, at most 1 update/s.\n"+(current.empty()?std::string("[log] No current run log found yet; waiting.\n"):std::string{});
    // One external process; a strict byte budget keeps noisy logs from competing with sensors.
    std::size_t budget=8192;
    for(std::size_t i=0;i<current.size()&&budget;++i) {
      const auto& file=current[(first_file+i)%current.size()];
      auto text=tail(file,cursors[file.path.string()],budget);
      if(!text.empty()&&groups[file.group].size()<4096)groups[file.group]+=text;
    }
    if(!current.empty())first_file=(first_file+1)%current.size();
    if(cursors.size()>64)for(auto it=cursors.begin();it!=cursors.end();) {
      const bool active=std::any_of(current.begin(),current.end(),[&](const File& f){return f.path.string()==it->first;});
      if(!active)it=cursors.erase(it);else ++it;
    }
    const double now=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    for(auto& [group,text]:groups) {
      text.resize(std::min<std::size_t>(4096,text.size()));announced[group]=true;
      std::ostringstream o;o<<std::setprecision(17)<<"{\"kind\":\"log\",\"source\":"<<quote(group)<<",\"board\":"<<quote(board)<<",\"t\":"<<now<<",\"seq\":"<<++seq<<",\"session\":"<<quote(session)<<",\"text\":"<<quote(text)<<"}\n";
      if(!write_frame(o.str()))return 0;
    }
    if(once)break;
    while(alive&&Clock::now()<next_tick){
      const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(next_tick-Clock::now()).count();
      pollfd input{STDIN_FILENO,POLLIN,0};const int result=::poll(&input,1,std::max(1,static_cast<int>(left)));
      if(result<0&&errno!=EINTR)return 0;
      if(input.revents&(POLLHUP|POLLERR|POLLNVAL))return 0;
      if(input.revents&POLLIN){char discard[128];if(::read(STDIN_FILENO,discard,sizeof(discard))<=0)return 0;}
    }
  }while(alive);
  return 0;
}

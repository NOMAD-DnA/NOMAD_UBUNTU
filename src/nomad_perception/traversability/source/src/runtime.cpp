#include "nomad_costmap/runtime.hpp"
#include <openssl/evp.h>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <spawn.h>
#include <sstream>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
namespace nomad {
  int64_t steady_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  Json read_json(const std::filesystem::path& p)
  {
    std::ifstream f(p);
    if (!f)
      throw std::runtime_error("Cannot read " + p.string());
    Json j;
    f >> j;
    return j;
  }
  void atomic_json(const std::filesystem::path& p, const Json& j)
  {
    if (!p.parent_path().empty())
      std::filesystem::create_directories(p.parent_path());
    auto tmp = p.string() + "." + std::to_string(getpid()) + ".tmp";
    {
      std::ofstream f(tmp);
      f.exceptions(std::ios::failbit | std::ios::badbit);
      f << j.dump() << '\n';
    }
    std::filesystem::rename(tmp, p);
  }
  static std::string hex(const unsigned char* bytes, size_t n)
  {
    std::ostringstream s;
    s << std::hex << std::setfill('0');
    for (size_t i = 0; i < n; ++i)
      s << std::setw(2) << int(bytes[i]);
    return s.str();
  }
  std::string sha256_text(const std::string& s)
  {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int n = 0;
    if (!EVP_Digest(s.data(), s.size(), digest, &n, EVP_sha256(), nullptr))
      throw std::runtime_error("SHA256 failed");
    return hex(digest, n);
  }
  std::string sha256_file(const std::filesystem::path& p)
  {
    std::ifstream f(p, std::ios::binary);
    if (!f)
      throw std::runtime_error("Cannot hash " + p.string());
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> c(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!c||!EVP_DigestInit_ex(c.get(), EVP_sha256(), nullptr))
      throw std::runtime_error("SHA256 initialization failed");
    std::array<char, 65536> b;
    while (f)
    {
      f.read(b.data(), b.size());
      if (!EVP_DigestUpdate(c.get(), b.data(), f.gcount()))
        throw std::runtime_error("SHA256 update failed");
    }
    if (!f.eof())
      throw std::runtime_error("Failed reading " + p.string());
    unsigned char d[EVP_MAX_MD_SIZE];
    unsigned int n = 0;
    if (!EVP_DigestFinal_ex(c.get(), d, &n))
      throw std::runtime_error("SHA256 finalization failed");
    return hex(d, n);
  }
  std::string executable_path()
  {
    std::array<char, 4096>b{};
    auto n = readlink("/proc/self/exe", b.data(), b.size() - 1);
    if (n <= 0)
      throw std::runtime_error("Cannot locate native executable");
    return std::string(b.data(), size_t(n));
  }
  Arguments::Arguments(int argc, char** argv)
  {
    for (int i = 1; i < argc; ++i)
    {
      std::string k = argv[i];
      if (k.rfind("--", 0) != 0)
        throw std::runtime_error("Expected --option, got " + k);
      if (values.count(k))
        throw std::runtime_error("Duplicate option " + k);
      values[k] = (i + 1 < argc&&std::string(argv[i + 1]).rfind("--", 0) != 0)?argv[++i]:"true";
    }
  }
  std::string Arguments::get(const std::string& k, const std::string& fallback)const
  {
    auto i = values.find(k);
    return i == values.end()?fallback:i->second;
  }
  bool Arguments::has(const std::string& k)const
  {
    return values.count(k);
  }
  static bool transfer(int fd, void* memory, size_t n, bool writing)
  {
    auto p = static_cast<unsigned char*>(memory);
    while (n)
    {
      ssize_t r = writing?::send(fd, p, n, MSG_NOSIGNAL) : ::recv(fd, p, n, 0);
      if (r < 0&&errno == EINTR)
        continue;
      if (r <= 0)
        return false;
      p += r;
      n -= size_t(r);
    }
    return true;
  }
  void Channel::fail(const std::string& s)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (fault_.empty()&&!stopping_)
      fault_ = s;
    cv_.notify_all();
  }
  void Channel::start(std::function<void(Json)> callback)
  {
    reader_ = std::thread([this, callback]{
      try{while (!stopping_)
          {
            std::array<unsigned char, 8> h{};
            if (!transfer(fd_, h.data(), h.size(), false))
            {
              fail("IPC peer closed");
              break;
            }
            uint64_t size = 0;
            for (auto b:h)
              size = (size << 8) | b;
            if (size == 0||size > 128 * 1024 * 1024)
              throw std::runtime_error("IPC packet exceeds bound");
            std::vector<uint8_t> data(size);
            if (!transfer(fd_, data.data(), data.size(), false))
            {
              fail("Truncated IPC packet");
              break;
            }
            callback(Json::from_cbor(data));
          }
      }
      catch (const std::exception& e){fail(e.what());
      }
    });
    writer_ = std::thread([this]{
      try{while (!stopping_)
          {
            Json message;
            {std::unique_lock<std::mutex> l(mutex_);
             cv_.wait(l, [this]{
              return stopping_||!outgoing_.empty();
            });
             if (stopping_)
               break;
             message = std::move(outgoing_.front());
             outgoing_.pop_front();
            }
            auto data = Json::to_cbor(message);
            if (data.empty()||data.size() > 128 * 1024 * 1024)
              throw std::runtime_error("IPC output exceeds bound");
            std::array<unsigned char, 8> h{};
            uint64_t n = data.size();
            for (int i = 7; i >= 0; --i)
            {
              h[i] = n & 255;
              n >>= 8;
            }
            if (!transfer(fd_, h.data(), h.size(), true)||!transfer(fd_, data.data(), data.size(), true))
            {
              fail("IPC write failed");
              break;
            }
          }
      }
      catch (const std::exception& e){fail(e.what());
      }
    });
  }
  void Channel::send(Json j)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (stopping_||!fault_.empty())
      throw std::runtime_error("IPC unavailable: " + fault_);
    if (outgoing_.size() >= 200)
    {
      fault_ = "IPC sender queue overflow (200)";
      throw std::runtime_error(fault_);
    }
    outgoing_.push_back(std::move(j));
    cv_.notify_one();
  }
  std::string Channel::fault()const
  {
    std::lock_guard<std::mutex> l(mutex_);
    return fault_;
  }
  void Channel::stop()
  {
    if (stopping_.exchange(true))
      return;
    shutdown(fd_, SHUT_RDWR);
    cv_.notify_all();
    if (reader_.joinable())
      reader_.join();
    if (writer_.joinable())
      writer_.join();
    close(fd_);
  }
  Channel::~Channel()
  {
    stop();
  }
  void Child::launch(const std::vector<std::string>& command, std::function<void(Json)> callback)
  {
    int original[2];
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, original))
      throw std::runtime_error("socketpair failed");
    int p = fcntl(original[0], F_DUPFD_CLOEXEC, 10), c = fcntl(original[1], F_DUPFD_CLOEXEC, 10);
    close(original[0]);
    close(original[1]);
    if (p < 0||c < 0)
      throw std::runtime_error("Cannot allocate IPC descriptors");
    std::vector<std::string> args = command;
    args.insert(args.end(), {"--ipc-fd", "3", "--parent-pid",std::to_string(getpid())});
    std::vector<char*> av;
    for (auto& s:args)
      av.push_back(s.data());
    av.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, c, 3);
    posix_spawn_file_actions_addclose(&actions, c);
    posix_spawn_file_actions_addclose(&actions, p);
    int result = posix_spawn(&pid, args[0].c_str(), &actions, nullptr, av.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(c);
    if (result)
    {
      close(p);
      pid = -1;
      throw std::runtime_error("posix_spawn: " + std::string(strerror(result)));
    }
    channel = std::make_unique<Channel>(p);
    channel->start(std::move(callback));
  }
  bool Child::alive()
  {
    if (pid <= 0)
      return false;
    int status = 0;
    pid_t r;
    do {r=waitpid(pid,&status,WNOHANG);} while(r<0&&errno==EINTR);
    if (r == 0)
      return true;
    pid = -1;
    return false;
  }
  void Child::stop()
  {
    if (channel)
      channel->stop();
    if (pid <= 0)
      return;
    kill(pid, SIGTERM);
    for (int i = 0; i < 50; ++i)
    {
      int status = 0;
      if (waitpid(pid, &status, WNOHANG) == pid)
      {
        pid = -1;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    kill(pid, SIGKILL);
    while(waitpid(pid,nullptr,0)<0&&errno==EINTR){}
    pid = -1;
  }
  Child::~Child()
  {
    stop();
  }
  PublisherLock::PublisherLock(const std::string& topic)
  {
    const char* domain = getenv("ROS_DOMAIN_ID");
    auto key = sha256_text(std::string(domain?domain:"") + topic).substr(0, 20);
    std::string p = "/tmp/nomad_real_perception_" + std::to_string(getuid()) + "_" + key + ".lock";
    fd_ = open(p.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd_ < 0||flock(fd_, LOCK_EX | LOCK_NB))
      throw std::runtime_error("Another costmap publisher owns " + p);
  }
  PublisherLock::~PublisherLock()
  {
    if (fd_ >= 0)
      close(fd_);
  }
  Json frame_json(const Frame& f)
  {
    return {{"stamp_ns", f.stamp}, {"source", f.source}, {"points", points_json(f.points)}, {"point_stamps", f.stamps}, {"pose", pose_json(f.pose)}};
  }
  Frame frame_from_json(const Json& j)
  {
    Frame f;
    f.stamp = j.at("stamp_ns");
    f.source = j.at("source");
    f.points = points_from_json(j.at("points"));
    f.stamps = j.at("point_stamps").get<std::vector<int64_t> >();
    f.pose = pose_from_json(j.at("pose"));
    if (f.points.size() != f.stamps.size())
      throw std::runtime_error("Frame stamp buffer differs");
    return f;
  }
}

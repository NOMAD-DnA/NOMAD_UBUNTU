#include "nomad_costmap/runtime.hpp"
#include <csignal>
#include <cerrno>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <spawn.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
namespace {
  using namespace nomad;
  volatile sig_atomic_t stopping = 0;
  void signal_stop(int)
  {
    stopping = 1;
  }
  Json identity(pid_t pid)
  {
    // Read the actual leader task. QEMU synthesizes /proc/self/stat and can
    // round its start time differently from another process's /proc/PID/stat.
    // Leader TID equals PID, so this remains the same kernel identity on Linux.
    std::string leader = "/proc/" + std::to_string(pid) + "/task/" + std::to_string(pid) + "/stat";
    std::ifstream f(leader);
    std::string line;
    std::getline(f, line);
    if (line.empty())
      return nullptr;
    auto close = line.rfind(')');
    if (close == std::string::npos)
      return nullptr;
    std::istringstream s(line.substr(close + 2));
    std::vector<std::string> fields;
    std::string v;
    while (s >> v)
      fields.push_back(v);
    if (fields.size() < 20)
      return nullptr;
    std::ifstream boot("/proc/sys/kernel/random/boot_id");
    std::string boot_id;
    std::getline(boot, boot_id);
    // QEMU's synthesized self stat can report pgrp=0. Ask the kernel instead.
    pid_t group = getpgid(pid);
    if (group < 0)
      return nullptr;
    return {{"pid", pid}, {"start_ticks", std::stoll(fields[19])}, {"boot_id", boot_id}, {"pgid", group}, {"process_state", fields[0]}};
  }
  bool same(const Json& expected)
  {
    if (expected.is_null())
      return false;
    auto actual = identity(expected.at("pid"));
    return !actual.is_null()&&actual.at("start_ticks") == expected.at("start_ticks")&&actual.at("boot_id") == expected.at("boot_id")&&actual.at("pgid") == expected.at("pgid")&&actual.at("process_state") != "Z";
  }
  void verify(const std::filesystem::path& prefix)
  {
    auto path = prefix / "runtime_manifest.json";
    if (!std::filesystem::exists(path))
      throw std::runtime_error("Runtime manifest is required: " + path.string());
    auto m = read_json(path);
    if (m.at("runtime_language") != "C++17"||m.at("python_runtime") != false)
      throw std::runtime_error("Not a Python-free costmap runtime");
    for (auto it = m.at("files").begin(); it != m.at("files").end(); ++it)
    {
      auto relative = std::filesystem::path(it.key());
      if (relative.is_absolute()||it.key().find("..") != std::string::npos)
        throw std::runtime_error("Unsafe manifest path");
      if (sha256_file(prefix / relative) != it.value())
        throw std::runtime_error("Runtime hash mismatch: " + it.key());
    }
    for (auto name:{"nomad_mapper", "nomad_monitor", "nomad_runtime_manager"})
    {
      std::ifstream f(prefix / "lib/nomad_costmap_cpp" / name, std::ios::binary);
      char magic[4]{};
      f.read(magic, 4);
      if (std::string(magic, 4) != std::string("\177ELF", 4))
        throw std::runtime_error("Runtime executable is not ELF");
    }
  }
  void environment(const std::filesystem::path& prefix, const std::string& ros)
  {
    auto prepend = [](const char* key, const std::string& value){
                     const char* prior = getenv(key);
                     setenv(key, (value + (prior&&*prior?":" + std::string(prior):"")).c_str(), 1);
                   };
    prepend("AMENT_PREFIX_PATH", prefix.string() + ":" + ros);
    prepend("LD_LIBRARY_PATH", (prefix / "lib").string() + ":" + ros + "/lib");
    setenv("ROS_DOMAIN_ID", "42", 0);
    setenv("ROS_LOCALHOST_ONLY", "0", 0);
    setenv("ROS_VERSION", "2", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 0);
    setenv("OPENBLAS_NUM_THREADS", "1", 0);
  }
  struct Process {
    pid_t pid = -1;
    Json owner;
    std::string name;
    void spawn(std::vector<std::string> argv, const std::filesystem::path& log)
    {
      std::vector<char*> av;
      for (auto& s:argv)
        av.push_back(s.data());
      av.push_back(nullptr);
      int fd = open(log.c_str(), O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC, 0600);
      if (fd < 0)
        throw std::runtime_error("Cannot open child log");
      posix_spawn_file_actions_t actions;
      posix_spawn_file_actions_init(&actions);
      posix_spawn_file_actions_adddup2(&actions, fd, STDOUT_FILENO);
      posix_spawn_file_actions_adddup2(&actions, fd, STDERR_FILENO);
      posix_spawn_file_actions_addclose(&actions, fd);
      posix_spawnattr_t attr;
      posix_spawnattr_init(&attr);
      posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
      posix_spawnattr_setpgroup(&attr, 0);
      int r = posix_spawn(&pid, av[0], &actions, &attr, av.data(), environ);
      posix_spawnattr_destroy(&attr);
      posix_spawn_file_actions_destroy(&actions);
      close(fd);
      if (r)
        throw std::runtime_error("Cannot launch " + name + ": " + std::to_string(r));
      for (int i = 0; i < 200; ++i)
      {
        owner = identity(pid);
        if (!owner.is_null()&&owner.at("pgid")==pid)
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (owner.is_null()||owner.at("pgid")!=pid)
      {
        // This is still our unreaped direct child, so its PID cannot be reused.
        kill(pid,SIGTERM);
        while(waitpid(pid,nullptr,0)<0&&errno==EINTR){}
        pid=-1;
        throw std::runtime_error("Child process group ownership unavailable");
      }
    }
    bool alive()
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
    void stop()
    {
      if (pid <= 0)
        return;
      if (same(owner))
        kill(-pid, SIGTERM);
      for (int i = 0; i < 500; ++i)
      {
        int status;
        if (waitpid(pid, &status, WNOHANG) == pid)
        {
          pid = -1;
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (same(owner))
        kill(-pid, SIGKILL);
      while(waitpid(pid,nullptr,0)<0&&errno==EINTR){}
      pid = -1;
    }
    ~Process()
    {
      stop();
    }
  };
  int supervise(const Arguments& args, const std::filesystem::path& prefix, const std::filesystem::path& directory)
  {
    std::signal(SIGINT, signal_stop);
    std::signal(SIGTERM, signal_stop);
    int run_lock = open((directory.parent_path() / "runtime.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (run_lock < 0||flock(run_lock, LOCK_EX | LOCK_NB))
      throw std::runtime_error("Another runtime supervisor is active");
    verify(prefix);
    std::string ros = args.get("--ros-prefix", "/opt/ros/humble/install");
    if (!std::filesystem::is_directory(ros))
      throw std::runtime_error("Missing ROS C++ underlay " + ros);
    environment(prefix, ros);
    std::filesystem::create_directories(directory);
    auto exe = prefix / "lib/nomad_costmap_cpp";
    auto config = prefix / "share/nomad_costmap_cpp/config";
    auto manifest = read_json(prefix / "runtime_manifest.json");
    std::string run = args.get("--run-id");
    Json state = {{"run_id", run}, {"state", "STARTING"}, {"ready", false}, {"manager", identity(getpid())}, {"prefix", prefix.string()}, {"build_id", manifest.at("build_id")}, {"children", Json::object()}};
    atomic_json(directory / "state.json", state);
    std::vector<std::unique_ptr<Process> > children;
    std::string failure;
    auto launch = [&](const std::string& name, std::vector<std::string> command){
                    auto p = std::make_unique<Process>();
                    p->name = name;
                    p->spawn(command, directory / (name + ".log"));
                    state["children"][name] = p->owner;
                    children.push_back(std::move(p));
                    atomic_json(directory / "state.json", state);
                  };
    try{
      launch("monitor", {(exe / "nomad_monitor").string(), "--output", (directory / "health.json").string(), "--run-id", run, "--release-id", manifest.at("build_id")});
      if (!args.has("--external-lidar"))
      {
        auto driver = prefix / "lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node";
        if (!std::filesystem::is_regular_file(driver))
          throw std::runtime_error("Managed native LiDAR driver is missing");
        launch("lidar", {driver.string(), "--ros-args", "--params-file", (config / "g2b.yaml").string(), "-p", "port:=" + args.get("--lidar-port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")});
      }
      launch("mapper", {(exe / "nomad_mapper").string(), "--semantic-config", (config / "semantic.real.json").string(), "--slope-config", (config / "slope.json").string(), "--lidar-calibration", (config / "oak_g2b_selected_20261030.json").string(), "--health-file", (directory / "health.json").string(), "--run-id", run, "--output", directory.string()});
      int64_t started = steady_ns();
      bool ever_ready = false;
      while (!stopping)
      {
        for (auto& p:children)
          if (!p->alive())
            throw std::runtime_error(p->name + " exited");
        bool ready = false;
        std::string reason = "Waiting for native health report";
        try{
          auto h = read_json(directory / "health.json");
          int64_t age = steady_ns() - h.at("updated_monotonic_ns").get<int64_t>();
          if (h.at("run_id") != run||age < 0||age > 1000000000)
            reason = "Native health report expired";
          else{
            if (h.value("fatal", false))
              throw std::logic_error(h.at("reason").get<std::string>());
            ready = h.value("ready", false);
            reason = h.value("reason", reason);
          }
        }
        catch (const std::logic_error&){
          throw;
        }
        catch (const std::exception&){}
        ever_ready |= ready;
        if (!ever_ready&&steady_ns() - started > int64_t(std::stod(args.get("--startup-timeout", "120")) * 1e9))
          throw std::runtime_error("Startup deadline expired: " + reason);
        state["state"] = ready?"READY":"WAITING";
        state["ready"] = ready;
        state["reason"] = reason;
        state["updated_monotonic_ns"] = steady_ns();
        atomic_json(directory / "state.json", state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    catch (const std::exception& e){
      failure = e.what();
    }
    state["ready"] = false;
    state["state"] = "STOPPING";
    atomic_json(directory / "state.json", state);
    for (auto it = children.rbegin(); it != children.rend(); ++it)
      (*it)->stop();
    state["state"] = failure.empty()?"STOPPED":"FAILED";
    state["reason"] = failure.empty()?"All owned child processes stopped":failure;
    atomic_json(directory / "state.json", state);
    close(run_lock);
    std::cout << state.dump() << '\n';
    return failure.empty()?0:2;
  }
}
int main(int argc, char** argv)
{
  try{
    nomad::Arguments args(argc, argv);
    auto prefix = std::filesystem::absolute(args.get("--prefix", std::filesystem::path(nomad::executable_path()).parent_path().parent_path().parent_path().string()));
    std::string action = args.get("--action", "check");
    if (action == "check")
    {
      verify(prefix);
      std::cout << "Native runtime hashes and ELF entry points verified\n";
      return 0;
    }
    std::string run = args.get("--run-id");
    if (!std::regex_match(run, std::regex("[A-Za-z0-9][A-Za-z0-9_-]{0,47}")))
      throw std::runtime_error("Invalid run ID");
    auto base = std::filesystem::absolute(args.get("--state-root", (prefix / "runs").string()));
    auto directory = base / run;
    if (action == "supervise")
      return supervise(args, prefix, directory);
    if (action == "status")
    {
      auto s = nomad::read_json(directory / "state.json");
      if (!same(s.at("manager"))&&s.at("state") != "STOPPED"&&s.at("state") != "FAILED")
      {
        s["state"] = "FAILED";
        s["ready"] = false;
        s["reason"] = "Supervisor no longer running; owned cleanup required";
      }
      std::cout << s.dump() << '\n';
      return 0;
    }
    if (action == "stop")
    {
      auto s = nomad::read_json(directory / "state.json");
      if (s.at("state") == "STOPPED"||s.at("state") == "FAILED")
      {
        std::cout << s.dump() << '\n';
        return 0;
      }
      if (!same(s.at("manager")))
        throw std::runtime_error("Supervisor identity changed; no unowned process was signalled");
      kill(s.at("manager").at("pid").get<pid_t>(), SIGTERM);
      for (int i = 0; i < 600; ++i)
      {
        auto state = nomad::read_json(directory / "state.json");
        if (state.at("state") == "STOPPED"||state.at("state") == "FAILED")
        {
          std::cout << state.dump() << '\n';
          return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      throw std::runtime_error("Owned shutdown did not complete within 60s");
    }
    if (action != "start")
      throw std::runtime_error("Expected --action check|start|status|stop|supervise");
    verify(prefix);
    if (std::filesystem::exists(directory / "state.json"))
      throw std::runtime_error("Run ID already exists; preserve it and choose a new ID");
    std::filesystem::create_directories(directory);
    std::vector<std::string> command{nomad::executable_path(), "--action", "supervise", "--prefix", prefix.string(), "--state-root", base.string(), "--run-id", run, "--ros-prefix", args.get("--ros-prefix", "/opt/ros/humble/install"), "--lidar-port", args.get("--lidar-port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"), "--startup-timeout", args.get("--startup-timeout", "120")};
    if (args.has("--external-lidar"))
      command.push_back("--external-lidar");
    pid_t pid = fork();
    if (pid < 0)
      throw std::runtime_error("Cannot start native supervisor");
    if (pid == 0)
    {
      setsid();
      int fd = open((directory / "supervisor.log").c_str(), O_CREAT | O_WRONLY | O_APPEND, 0600);
      if (fd < 0)
        _exit(2);
      dup2(fd, STDOUT_FILENO);
      dup2(fd, STDERR_FILENO);
      close(fd);
      std::vector<char*> av;
      for (auto& s:command)
        av.push_back(s.data());
      av.push_back(nullptr);
      execv(av[0], av.data());
      _exit(2);
    }
    std::cout << nomad::Json{{"run_id", run}, {"state", "STARTING"}, {"supervisor_pid", pid}, {"directory", directory.string()}}.dump() << '\n';
    return 0;
  }
  catch (const std::exception& e){
    std::cerr << "Native runtime manager: " << e.what() << '\n';
    return 2;
  }
}

#include "nomad_planning_cpp/latest_json_writer.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

using Writer = nomad_planning_cpp::LatestJsonWriter;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

struct Pipe {
  int fd[2];
  Pipe() {
    if (::pipe(fd) != 0) throw std::runtime_error("pipe failed");
#ifdef F_SETPIPE_SZ
    ::fcntl(fd[1], F_SETPIPE_SZ, 4096);
#endif
  }
  ~Pipe() { for (int n : fd) if (n >= 0) ::close(n); }
};

template<class Predicate>
static bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 500ms) {
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

static std::vector<std::string> read_lines(int fd, std::size_t count) {
  std::string buffer;
  std::vector<std::string> lines;
  const auto deadline = Clock::now() + 2s;
  while (lines.size() < count && Clock::now() < deadline) {
    pollfd input{fd, POLLIN, 0};
    if (::poll(&input, 1, 20) <= 0) continue;
    char chunk[8192];
    const auto n = ::read(fd, chunk, sizeof(chunk));
    if (n <= 0) break;
    buffer.append(chunk, static_cast<std::size_t>(n));
    for (std::size_t end; (end = buffer.find('\n')) != std::string::npos;) {
      lines.push_back(buffer.substr(0, end)); buffer.erase(0, end + 1);
    }
  }
  return lines;
}

static void nonreading_peer_does_not_block_producer_or_shutdown() {
  Pipe pipe;
  std::atomic<int> snapshots{0}, failures{0};
  Writer::Options options; options.period = 10ms; options.write_timeout = 5s;
  const int before_flags = ::fcntl(pipe.fd[1], F_GETFL, 0);
  Writer writer(pipe.fd[1], [&] { ++snapshots; return std::string(128 * 1024, 'x'); },
                [&] { ++failures; }, options);
  require(wait_until([&] { return snapshots.load() > 0; }), "writer did not begin telemetry");
  std::this_thread::sleep_for(20ms); // No reader: first line cannot fit in the pipe.
  const auto start = Clock::now();
  for (int n = 0; n < 10000; ++n) writer.request_snapshot();
  const auto producer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
  require(producer_ms < 250, "telemetry requests blocked behind stdout");
  require(snapshots.load() == 1, "nonreading peer accumulated telemetry snapshots");
  const auto stop = Clock::now();
  writer.stop(100ms);
  const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - stop).count();
  require(stop_ms < 300, "shutdown waited for the nonreading peer");
  require(failures.load() == 0, "bounded shutdown falsely reported a transport failure");
  require(::fcntl(pipe.fd[1], F_GETFL, 0) == before_flags, "fd flags were not restored");
  std::cout << "nonreading peer: 10000 producer requests " << producer_ms
            << " ms; graceful shutdown " << stop_ms << " ms\n";
}

static void acknowledgements_are_prioritized_and_telemetry_is_latest_only() {
  Pipe pipe;
  std::atomic<int> version{1}, snapshots{0}, failures{0};
  Writer::Options options; options.period = 10ms; options.write_timeout = 5s;
  Writer writer(pipe.fd[1], [&] {
    const int current_version = version.load();
    ++snapshots;
    return "{\"version\":" + std::to_string(current_version) + ",\"pad\":\"" +
           std::string(64 * 1024, 'x') + "\"}";
  }, [&] { ++failures; }, options);
  require(wait_until([&] { return snapshots.load() == 1; }), "initial snapshot missing");
  std::this_thread::sleep_for(20ms);
  for (int n = 2; n <= 100; ++n) { version = n; writer.request_snapshot(); }
  require(writer.acknowledge("{\"ack\":1}"), "ACK 1 was rejected");
  require(writer.acknowledge("{\"ack\":2}"), "ACK 2 was rejected");
  const auto lines = read_lines(pipe.fd[0], 4);
  writer.stop();
  require(lines.size() >= 4, "missing or corrupted JSON lines after a partial write");
  require(lines[0].find("{\"version\":1,") == 0, "partial initial line was interleaved");
  require(lines[1] == "{\"ack\":1}" && lines[2] == "{\"ack\":2}", "ACK priority/FIFO order changed");
  require(lines[3].find("{\"version\":100,") == 0, "old telemetry was replayed instead of current state");
  require(failures.load() == 0, "healthy reader reported failure");
}

static void acknowledgement_overflow_fails_explicitly() {
  Pipe pipe;
  std::atomic<int> snapshots{0}, failures{0};
  Writer::Options options; options.max_ack_lines = 2; options.write_timeout = 5s;
  Writer writer(pipe.fd[1], [&] { ++snapshots; return std::string(128 * 1024, 'x'); },
                [&] { ++failures; }, options);
  require(wait_until([&] { return snapshots.load() > 0; }), "initial snapshot missing");
  std::this_thread::sleep_for(20ms);
  require(writer.acknowledge("one") && writer.acknowledge("two"), "bounded ACK capacity incorrect");
  require(!writer.acknowledge("three"), "ACK overflow was silently accepted");
  require(writer.failed() && failures.load() == 1, "ACK loss was not reported explicitly");
  writer.stop();
}

static void closed_or_stalled_peer_fails_without_sigpipe_or_hang() {
  {
    Pipe pipe; ::close(pipe.fd[0]); pipe.fd[0] = -1;
    std::atomic<int> failures{0};
    Writer writer(pipe.fd[1], [] { return "closed"; }, [&] { ++failures; });
    require(wait_until([&] { return failures.load() == 1; }), "EPIPE was not reported");
    writer.stop();
  }
  {
    Pipe pipe;
    std::atomic<int> failures{0};
    Writer::Options options; options.write_timeout = 80ms;
    Writer writer(pipe.fd[1], [] { return std::string(128 * 1024, 'x'); }, [&] { ++failures; }, options);
    require(wait_until([&] { return failures.load() == 1; }), "stalled output did not time out");
    writer.stop();
  }
  {
    Pipe pipe;
    std::atomic<int> snapshots{0}, failures{0};
    Writer writer(pipe.fd[1], [&] { ++snapshots; return std::string(128 * 1024, 'x'); }, [&] { ++failures; });
    require(wait_until([&] { return snapshots.load() == 1; }), "shutdown snapshot missing");
    std::this_thread::sleep_for(20ms);
    std::thread close_peer([&] { std::this_thread::sleep_for(10ms); ::close(pipe.fd[0]); pipe.fd[0] = -1; });
    writer.stop(100ms);
    close_peer.join();
    require(failures.load() == 0, "clean shutdown was reclassified as a transport fault");
  }
}

int main() {
  try {
    nonreading_peer_does_not_block_producer_or_shutdown();
    acknowledgements_are_prioritized_and_telemetry_is_latest_only();
    acknowledgement_overflow_fails_explicitly();
    closed_or_stalled_peer_fails_without_sigpipe_or_hang();
    std::cout << "latest_json_writer: PASS\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "latest_json_writer: FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}

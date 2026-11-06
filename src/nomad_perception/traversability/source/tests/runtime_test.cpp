#include "nomad_costmap/runtime.hpp"
#include <cassert>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>
int main()
{
  using namespace nomad;
  int fds[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0);
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<int64_t> received;
  Channel a(fds[0]), b(fds[1]);
  a.start([](Json){
  });
  b.start([&](Json j){
    std::lock_guard<std::mutex> l(mutex);
    received.push_back(j.at("stamp"));
    cv.notify_one();
  });
  const int64_t original = 1793870000000000001LL;
  for (int i = 0; i < 80; ++i)
    a.send({{"stamp", original + i}, {"points", {{.1, .2, .3}}}});
  {
    std::unique_lock<std::mutex> l(mutex);
    assert(cv.wait_for(l, std::chrono::seconds(3), [&]{
      return received.size() == 80;
    }));
    for (int i = 0; i < 80; ++i)
      assert(received[i] == original + i);
  }
  a.stop();
  b.stop();
  int queued[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM, 0, queued)==0);
  {
    Channel bounded(queued[0]);
    for(int i=0;i<200;++i) bounded.send({{"sequence",i}});
    bool rejected=false;
    try {bounded.send({{"sequence",200}});}catch(const std::exception&){rejected=true;}
    assert(rejected&&!bounded.fault().empty());
  }
  close(queued[1]);
  int malformed[2];
  assert(socketpair(AF_UNIX,SOCK_STREAM,0,malformed)==0);
  {
    Channel guarded(malformed[0]);guarded.start([](Json){});
    uint64_t zero_length=0;
    assert(write(malformed[1],&zero_length,sizeof zero_length)==sizeof zero_length);
    for(int i=0;i<100&&guarded.fault().empty();++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(guarded.fault()=="IPC packet exceeds bound");
  }
  close(malformed[1]);
  auto path = std::filesystem::temp_directory_path() / ("nomad_test_" + std::to_string(getpid()) + ".json");
  atomic_json(path, {{"stamp", original}});
  assert(read_json(path).at("stamp") == original);
  assert(sha256_text("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  assert(sha256_file(path).size() == 64);
  std::filesystem::remove(path);
  {
    PublisherLock lock("/test/nomad_" + std::to_string(getpid()));
    bool refused = false;
    try{
      PublisherLock second("/test/nomad_" + std::to_string(getpid()));
    }
    catch (const std::exception&){
      refused = true;
    }
    assert(refused);
  }
  std::cout << "IPC ordering, int64 capture stamps, atomic state, SHA256 and publisher ownership passed\n";
}

#include "nomad_control_cpp/gui_http.hpp"
#include <atomic>
#include <iostream>
#include <thread>

namespace {
// 브라우저의 사전 연결처럼 TCP만 열고 HTTP 첫 바이트는 보내지 않습니다.
class EmptyConnection {
public:
    EmptyConnection() {
        socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ == INVALID_SOCKET) throw std::runtime_error("test socket creation failed");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(18765);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::connect(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            httplib::detail::close_socket(socket_);
            throw std::runtime_error("test preconnection failed");
        }
    }
    ~EmptyConnection() { httplib::detail::close_socket(socket_); }
    EmptyConnection(const EmptyConnection&) = delete;
    EmptyConnection& operator=(const EmptyConnection&) = delete;
    bool closesWithin(std::chrono::milliseconds timeout) const {
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(socket_, &readable);
        timeval wait{static_cast<long>(timeout.count() / 1000),
                     static_cast<long>((timeout.count() % 1000) * 1000)};
        if (::select(static_cast<int>(socket_ + 1), &readable, nullptr, nullptr, &wait) <= 0) return false;
        char byte;
        return ::recv(socket_, &byte, 1, 0) == 0;
    }
private:
    socket_t socket_ = INVALID_SOCKET;
};

// 같은 클라이언트의 로컬 포트는 서버가 보는 원격 포트와 같아 TCP 연결 재사용을 확인할 수 있습니다.
int localPort(httplib::Client& client) {
    if (!client.is_socket_open()) return 0;
    sockaddr_in address{};
    socklen_t length = sizeof(address);
    if (::getsockname(client.socket(), reinterpret_cast<sockaddr*>(&address), &length) != 0) return 0;
    return ntohs(address.sin_port);
}
} // namespace

// 가짜 HTTP 처리기만 사용합니다. ROS나 실제 차량과 연결하지 않습니다.
int main() {
    int failures = 0, checks = 0;
    const auto expect = [&](bool ok, const char* name) {
        ++checks; if (!ok) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
    };
    std::atomic<int> command_count{0};
    nomad::GuiHttp server(NOMAD_TEST_WEB, 18765,
        [] { return nomad::Json{{"preview", true}, {"connected", false}}; },
        [&](const std::string&, const nomad::Json&) {
            ++command_count; return nomad::Json{{"accepted", false}, {"reason", "test only"}};
        });
    std::thread thread([&] { server.listen(); });
    for (int i = 0; i < 100 && !server.isRunning(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    try {
        if (!server.isRunning()) throw std::runtime_error("test HTTP port unavailable");
        httplib::Client client("127.0.0.1", 18765);
        client.set_read_timeout(2, 0);
        auto response = client.Get("/api/status");
        if (!response) throw std::runtime_error("test status connection failed");
        auto status = nomad::Json::parse(response->body);
        expect(status["preview"] == true && status["connected"] == false, "status endpoint");
        const httplib::Headers headers{{"Origin", "http://127.0.0.1:18765"}, {"X-Nomad-Token", status["token"].get<std::string>()}};
        response = client.Post("/api/stop", "{}", "application/json");
        expect(response && response->status == 403, "missing token rejected");
        auto foreign = headers; foreign.erase("Origin"); foreign.emplace("Origin", "http://example.com");
        response = client.Post("/api/stop", foreign, "{}", "application/json");
        expect(response && response->status == 403, "foreign origin rejected");
        auto bad_host = headers; bad_host.emplace("Host", "example.com:18765");
        response = client.Post("/api/stop", bad_host, "{}", "application/json");
        expect(response && response->status == 403, "foreign host rejected");
        expect(command_count == 0, "unauthorized requests never reach handler");
        response = client.Post("/api/steering-records", "{}", "application/json");
        expect(response && response->status == 403 && command_count==0,"measurement writes require origin and token");
        response = client.Post("/api/steering-range", "{}", "application/json");
        expect(response && response->status == 403 && command_count==0,"range changes require origin and token");
        response = client.Post("/api/manual", headers, "{}", "application/json");
        expect(response && response->status == 200 && command_count == 1, "authorized request reaches handler once");
        response = client.Post("/api/manual", headers, "bad-json", "application/json");
        expect(response && response->status == 400 && command_count == 1, "malformed JSON rejected");
        response = client.Get("/");
        expect(response && response->status == 200 && response->body.find("NOMAD") != std::string::npos, "page served");
        // 완료된 요청 뒤의 유휴 연결이 STOP을 막지 않는지 검사합니다.
        std::vector<std::unique_ptr<httplib::Client>> idle_clients;
        for (int i = 0; i < 4; ++i) {
            auto idle = std::make_unique<httplib::Client>("127.0.0.1", 18765);
            idle->set_keep_alive(true);
            idle->set_read_timeout(1, 0);
            const auto reply = idle->Get("/api/status");
            expect(reply && reply->status == 200, "independent status client served");
            idle_clients.push_back(std::move(idle));
        }
        client.set_read_timeout(1, 0);
        response = client.Post("/api/stop", headers, "{}", "application/json");
        expect(response && response->status == 200 && command_count == 2,
               "idle browser connections cannot starve an authorized STOP");
        idle_clients.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // 완료된 요청 뒤의 keep-alive와 달리, 첫 요청 전의 연결도 worker를 점유합니다.
        for (const int connections : {4, 6}) {
            std::vector<std::unique_ptr<EmptyConnection>> preconnections;
            for (int i = 0; i < connections; ++i) preconnections.push_back(std::make_unique<EmptyConnection>());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const int before_stop = command_count.load();
            client.set_read_timeout(0, 250000);
            const auto started = std::chrono::steady_clock::now();
            response = client.Post("/api/stop", headers, "{}", "application/json");
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            std::cout << connections << " empty preconnections: STOP " << elapsed_ms << " ms\n";
            expect(response && response->status == 200 && command_count == before_stop + 1 && elapsed_ms <= 300.0,
                   "browser preconnections leave an authorized STOP available within 300ms");
            preconnections.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        EmptyConnection unused;
        expect(unused.closesWithin(std::chrono::milliseconds(1500)),
               "unused initial connection expires within 1.5s rather than 5s");
        // 매 요청마다 새 TCP/SSH 연결을 만들지 않고 GET과 인증된 POST가 같은 소켓을 쓰는지 검사합니다.
        httplib::Client persistent("127.0.0.1", 18765);
        persistent.set_keep_alive(true);
        persistent.set_read_timeout(0, 250000);
        auto first = persistent.Get("/api/status");
        const int port = localPort(persistent);
        expect(first && first->status == 200 && port != 0,
               "keep-alive status leaves an established TCP connection");
        bool reused = port != 0;
        const int before_persistent = command_count.load();
        for (int i = 0; i < 3; ++i) {
            const auto read = persistent.Get("/api/status");
            reused = reused && read && read->status == 200 && localPort(persistent) == port;
            const auto post = persistent.Post("/api/stop", headers, "{}", "application/json");
            reused = reused && post && post->status == 200 && localPort(persistent) == port;
        }
        expect(reused && command_count == before_persistent + 3,
               "GET and authorized POST reuse the original TCP connection");
    } catch (const std::exception& e) { ++failures; std::cerr << e.what() << '\n'; }
    server.stop(); thread.join();
    std::cout << checks << " HTTP checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

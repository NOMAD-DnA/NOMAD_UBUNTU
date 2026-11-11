#pragma once
#include "httplib.h"
#include "json.hpp"
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <random>
#include <sstream>

namespace nomad {
using Json = nlohmann::json;
inline double unixSeconds() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// HTTP는 loopback에만 열고, 변경 요청에는 같은 출처와 임시 토큰을 모두 요구합니다.
class GuiHttp {
public:
    using Handler = std::function<Json(const std::string&, const Json&)>;
    GuiHttp(const std::string& page, int port, std::function<Json()> status, Handler command)
        : port_(port) {
        std::ifstream file(page, std::ios::binary);
        if (!file) throw std::runtime_error("GUI page not found: " + page);
        const std::string html((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        std::random_device random;
        std::ostringstream token;
        for (int i = 0; i < 8; ++i) token << std::hex << std::setw(8) << std::setfill('0') << random();
        token_ = token.str();
        // 브라우저 사전 연결 6개를 가정하고 상태 조회와 STOP 처리 자리를 남깁니다.
        // worker와 대기열을 모두 제한하며, 과부하 연결은 무한히 쌓지 않습니다.
        server_.new_task_queue = [] { return new httplib::ThreadPool(8, 8); };
        // 작은 응답을 즉시 보내고, 제한된 연결 재사용으로 TCP/SSH 연결 생성 빈도를 줄입니다.
        server_.set_tcp_nodelay(true);
        server_.set_keep_alive_max_count(32);
        // 이 제한은 HTTP 첫 바이트를 보내지 않은 사전 연결에도 적용됩니다.
        server_.set_keep_alive_timeout(1);
        server_.set_payload_max_length(4096);
        server_.set_read_timeout(1, 0);
        server_.set_write_timeout(1, 0);
        server_.set_pre_routing_handler([this](const httplib::Request& req, httplib::Response& res) {
            res.set_header("Cache-Control", "no-store");
            res.set_header("X-Content-Type-Options", "nosniff");
            res.set_header("X-Frame-Options", "DENY");
            res.set_header("Content-Security-Policy", "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'");
            const auto host = req.get_header_value("Host");
            const bool local = host == "127.0.0.1:" + std::to_string(port_) || host == "localhost:" + std::to_string(port_);
            const auto origin = req.get_header_value("Origin");
            if (req.method != "POST" && (!local || (!origin.empty() && origin != "http://" + host))) {
                res.status = 403; res.set_content("Forbidden", "text/plain");
                return httplib::Server::HandlerResponse::Handled;
            }
            return httplib::Server::HandlerResponse::Unhandled;
        });
        server_.Get("/", [html](const httplib::Request&, httplib::Response& res) { res.set_content(html, "text/html; charset=utf-8"); });
        server_.Get("/api/status", [this, status](const httplib::Request&, httplib::Response& res) {
            auto data = status(); data["token"] = token_; data["server_time_s"] = unixSeconds();
            res.set_content(data.dump(), "application/json");
        });
        server_.Post(R"(/api/(manual|commissioning|stop|reset|steering-records|steering-range))", [this, command](const httplib::Request& req, httplib::Response& res) {
            const auto host = req.get_header_value("Host");
            if ((host != "127.0.0.1:" + std::to_string(port_) && host != "localhost:" + std::to_string(port_)) ||
                req.get_header_value("Origin") != "http://" + host || req.get_header_value("X-Nomad-Token") != token_) {
                res.status = 403; res.set_content("Forbidden", "text/plain"); return;
            }
            try {
                const auto data = Json::parse(req.body);
                if (!data.is_object()) throw std::runtime_error("expected JSON object");
                res.set_content(command(req.matches[1], data).dump(), "application/json");
            } catch (const std::exception& e) {
                res.status = 400;
                res.set_content(Json{{"accepted", false}, {"reason", e.what()}}.dump(), "application/json");
            }
        });
    }
    bool listen() { return server_.listen("127.0.0.1", port_); }
    bool isRunning() const { return server_.is_running(); }
    void stop() { server_.stop(); }
private:
    int port_;
    std::string token_;
    httplib::Server server_;
};
} // namespace nomad

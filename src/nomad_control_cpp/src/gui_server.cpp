#include "nomad_control_cpp/gui_http.hpp"
#include "nomad_control_cpp/steering_records.hpp"
#include "nomad_control_cpp/control_limits.hpp"
#include "nomad_control_cpp/srv/manual_control.hpp"
#include "nomad_control_cpp/srv/commissioning_control.hpp"
#include "nomad_control_cpp/srv/set_steering_range.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <cmath>
#include <iostream>
#include <thread>
#include <limits>
#include <cstdlib>
#include <filesystem>

using namespace std;
// Exception: inside rclcpp::Node subclasses, Node::make_shared/make_unique (static members added by
// RCLCPP_SMART_PTR_DEFINITIONS) hide the free functions, so those calls stay std::-qualified.

namespace nomad {
using Commissioning = nomad_control_cpp::srv::CommissioningControl;
using Manual = nomad_control_cpp::srv::ManualControl;
using Trigger = std_srvs::srv::Trigger;
using SteeringRange = nomad_control_cpp::srv::SetSteeringRange;

// HTTP worker는 ROS 서비스를 요청하고, 별도 executor가 응답과 상태 구독을 처리합니다.
class GuiBridge : public rclcpp::Node {
public:
    explicit GuiBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("nomad_gui", options) {
        const auto controller = declare_parameter<string>("controller", "/nomad_vesc_trial");
        // 다른 로그인 계정에서도 사용자 홈 아래에 기록하며 ROS 매개변수로 재지정할 수 있습니다.
        const char* user_home=getenv("HOME");
        const auto record_path=(user_home ? filesystem::path(user_home) : filesystem::current_path()) /
            "nomad_cpp_ws/calibration/steering";
        records_=std::make_unique<SteeringRecords>(declare_parameter<string>(
            "steering_records_directory",record_path.string()));
        manual_ = create_client<Manual>(controller + "/manual_control");
        commissioning_ = create_client<Commissioning>(controller + "/commissioning_control");
        steering_range_=create_client<SteeringRange>(controller+"/steering_test_range");
        stop_ = create_client<Trigger>(controller + "/stop");
        reset_ = create_client<Trigger>(controller + "/reset");
        subscription_ = create_subscription<std_msgs::msg::String>(controller + "/status_json", 1,
            [this](std_msgs::msg::String::ConstSharedPtr message) {
                try {
                    auto data = Json::parse(message->data);
                    lock_guard<mutex> guard(status_mutex_);
                    status_ = move(data); received_ = chrono::steady_clock::now();
                } catch (const exception&) { /* 손상된 상태는 수신 시간을 갱신하지 않습니다. */ }
            });
    }
    Json status() {
        Json data;
        {
            // HTTP 부가 필드/기록 잠금을 처리하는 동안 telemetry 수신을 막지 않습니다.
            lock_guard<mutex> guard(status_mutex_);
            data=status_;
            data["connected"] = chrono::duration<double>(chrono::steady_clock::now() - received_).count() < 0.6;
        }
        data["preview"] = false;
        data["steering_records_supported"] = true;
        data["steering_command_recording_supported"] = true;
        {lock_guard<mutex> lock(record_mutex_);data["steering_record_error"]=record_error_;}
        return data;
    }
    Json command(const string& route, const Json& data) {
        if(route=="steering-records") return steeringRecords(data);
        if(route=="steering-range") {
            const auto s=status();const double span=data.at("span").get<double>(),sent=data.at("server_time_s").get<double>();
            const double age=unixSeconds()-sent;
            if(!s.value("connected",false)||!s.value("steering_range_config_supported",false)||
               !isfinite(span)||!isfinite(sent)||age<-limits::kFutureStampToleranceS||age>limits::kWifiManualCommandMaxAgeS||!steering_range_->service_is_ready())
                return refused("range request invalid, stale or controller unavailable");
            auto request=std::make_shared<SteeringRange::Request>();request->span=span;request->stamp=rclcpp::Time(static_cast<int64_t>(sent*1e9));
            auto future=steering_range_->async_send_request(request);
            if(future.wait_for(chrono::milliseconds(250))!=future_status::ready){steering_range_->remove_pending_request(future);return refused("range response timeout; check controller range");}
            const auto result=future.get();return {{"accepted",result->accepted},{"reason",result->reason},{"servo_min",result->servo_min},{"servo_max",result->servo_max}};
        }
        if (route == "stop" || route == "reset") {
            auto client = route == "stop" ? stop_ : reset_;
            if (!client->service_is_ready()) return refused("controller service unavailable");
            auto future = client->async_send_request(std::make_shared<Trigger::Request>());
            if (future.wait_for(chrono::milliseconds(250)) != future_status::ready) {
                client->remove_pending_request(future); return refused("service response timeout; observe state");
            }
            const auto result = future.get();
            return {{"accepted", result->success}, {"reason", result->message}};
        }
        if (route == "commissioning") return rawCommand(data);
        auto request = std::make_shared<Manual::Request>();
        const int action = data.at("action").get<int>();
        if (action < 1 || action > 3) return refused("invalid action");
        request->action = static_cast<uint8_t>(action);
        request->session = data.at("session").get<string>();
        request->sequence = data.at("sequence").get<uint64_t>();
        if (action != Manual::Request::STOP) {
            // 브라우저가 마지막으로 받은 서버 시각을 사용하여 지연된 HTTP 명령도 거절합니다.
            const double sent = data.at("server_time_s").get<double>();
            const double age = unixSeconds() - sent;
            if (!isfinite(sent) || age < -limits::kFutureStampToleranceS || age > limits::kWiredCommandMaxAgeS || !status().value("connected", false))
                return refused("stale browser command or controller status");
            request->stamp = rclcpp::Time(static_cast<int64_t>(sent * 1e9));
            request->speed_mps = data.at("speed_mps").get<double>();
            request->steering_rad = data.at("steering_rad").get<double>();
            request->duration_s = data.at("duration_s").get<double>();
        }
        if (!manual_->service_is_ready()) return refused("manual service unavailable");
        auto future = manual_->async_send_request(request);
        if (future.wait_for(chrono::milliseconds(250)) != future_status::ready) {
            manual_->remove_pending_request(future); return refused("manual response timeout; release hold");
        }
        const auto result = future.get();
        return {{"accepted", result->accepted}, {"reason", result->reason}};
    }
private:
    Json steeringRecords(const Json& data) {
        if(data.value("operation","")=="list")return records_->list();
        if(data.value("operation","")!="save")return refused("unknown measurement operation");
        const auto s=status();
        const auto state=s.value("state","");
        if(!s.value("connected",false)||!s.value("telemetry_fresh",false)||!s.value("stationary",false)||
           s.value("control_units","")!="raw"||(state!="IDLE"&&state!="STOPPED_OUTPUT")||
           !s.contains("commanded_motor_erpm")||s.at("commanded_motor_erpm").get<double>()!=0)
            return refused("stop the steering trial and wait for stationary telemetry before saving");
        const double servo=data.at("servo_command").get<double>();
        const double current=s.at("commanded_servo_position").get<double>();
        if(!isfinite(servo)||!isfinite(current)||abs(servo-current)>0.0001||
            servo<s.value("steering_test_servo_min",s.at("servo_min").get<double>())||servo>s.value("steering_test_servo_max",s.at("servo_max").get<double>()))
            return refused("servo command changed; measure again at the displayed command");
        const bool measured=data.value("kind",string("measured"))=="measured";
        if(!measured&&data.value("kind",string{})!="observation")return refused("unknown record kind");
        const double left=measured?data.at("left_deg").get<double>():0,right=measured?data.at("right_deg").get<double>():0;
        // 사용자가 입력한 양쪽 실제 각도와 실제 송신 명령을 짝지어 저장합니다. 목표 명령/입력 rad와 구분합니다.
        Json row={{"id",data.at("id")},{"servo_command",servo},{"left_deg",left},{"right_deg",right},
            {"left_rad",left*3.141592653589793/180},{"right_rad",right*3.141592653589793/180},
            {"note",data.value("note",string{})},{"measured_confirmed",data.value("measured_confirmed",false)},
            {"saved_at_s",unixSeconds()},{"applied",false},{"schema",1},
            {"angle_convention","body forward reference; left positive, right negative"},
            {"servo_source","controller commanded position, not measured servo feedback"}};
        row["kind"]=measured?"measured":"observation";
        if(!measured){for(const auto* key:{"left_deg","right_deg","left_rad","right_rad"})row[key]=nullptr;row["measured_confirmed"]=false;}
        return records_->save(row);
    }
    unique_ptr<SteeringRecords> records_;
    mutex record_mutex_;
    // 같은 시험의 동일 목표 heartbeat는 파일을 반복 생성하지 않습니다.
    string record_session_,record_error_;
    double record_target_=numeric_limits<double>::quiet_NaN();
    // 승인된 조향 요청만 기록합니다. 실제 바퀴가 목표에 도달했다는 뜻은 아닙니다.
    void recordSteeringCommand(const Commissioning::Request& request) {
        const auto snapshot=status();
        lock_guard<mutex> lock(record_mutex_);
        if(record_session_==request.session&&record_target_==request.servo_position)return;
        try {
            Json row={{"id",SteeringRecords::newId()},{"kind","command"},{"schema",2},
                {"servo_command",request.servo_position},{"observed_servo_command",snapshot.at("commanded_servo_position")},
                {"left_deg",nullptr},{"right_deg",nullptr},{"left_rad",nullptr},{"right_rad",nullptr},
                {"measured_confirmed",false},{"applied",false},{"saved_at_s",unixSeconds()},
                {"note","automatic: accepted steering request; wheel angle unmeasured"},
                {"servo_source","accepted target; observed_servo_command is controller command near acknowledgement, not position feedback"},
                {"session",request.session},{"sequence",request.sequence},{"motor_erpm",0}};
            records_->save(row);record_session_=request.session;record_target_=request.servo_position;record_error_.clear();
        } catch(const exception& e) {record_error_=e.what();}
    }
    // 미보정 시험은 ERPM/서보값 전용 서비스로 전달하며 m/s·rad 서비스와 섞지 않습니다.
    Json rawCommand(const Json& data) {
        const auto entered_at = chrono::steady_clock::now();
        auto request = std::make_shared<Commissioning::Request>();
        const int action = data.at("action").get<int>();
        double command_age_s = 0.0;
        bool has_stamp = false;
        const bool record_command=data.value("record_steering_command",false);
        // HTTP 처리 진입부터 ROS 응답까지 측정하며 요청 본문과 인증값은 기록하지 않습니다.
        const auto timed_result = [&](Json result, bool timed_out = false) {
            const double elapsed_ms = chrono::duration<double, milli>(
                chrono::steady_clock::now() - entered_at).count();
            if (elapsed_ms > 100.0 || timed_out || !result.value("accepted", false) ||
                (has_stamp && (command_age_s > 0.15 || command_age_s < -limits::kFutureStampToleranceS)))
                RCLCPP_INFO(get_logger(), "GUI timing action=%d handler_ms=%.1f has_stamp=%d command_age_ms=%.1f accepted=%d timeout=%d",
                            action, elapsed_ms, static_cast<int>(has_stamp), command_age_s * 1000.0,
                            static_cast<int>(result.value("accepted", false)), static_cast<int>(timed_out));
            return result;
        };
        if (action < 1 || action > 3) return timed_result(refused("invalid action"));
        request->action = static_cast<uint8_t>(action);
        request->session = data.at("session").get<string>();
        request->sequence = data.at("sequence").get<uint64_t>();
        if (action != Commissioning::Request::STOP) {
            const double sent = data.at("server_time_s").get<double>();
            const double age = unixSeconds() - sent;
            command_age_s = age;
            has_stamp = true;
            const auto current = status();
            const int axis = data.at("axis").get<int>();
            if (axis < 1 || axis > 4) return timed_result(refused("select motor, steering, manual or LPP"));
            if(axis==4&&!current.value("planning_supported",false))return timed_result(refused("planning interface is not installed on controller"));
            // GUI가 보낸 정책을 믿지 않고 제어기가 알린 Wi-Fi 연속 MANUAL 한계만 사용합니다.
            const bool wifi_manual = (axis == Commissioning::Request::MANUAL || axis == Commissioning::Request::AUTONOMOUS) &&
                current.value("wifi_manual_profile", false) && current.value("commissioning", false);
            const double max_age = wifi_manual ? current.value("command_max_age_s", limits::kWiredCommandMaxAgeS) : limits::kWiredCommandMaxAgeS;
            if (!isfinite(sent) || !isfinite(max_age) || max_age <= 0.0 || max_age > limits::kWifiManualCommandMaxAgeS ||
                age < -limits::kFutureStampToleranceS || age > max_age ||
                !current.value("connected", false) || current.value("control_units", "") != "raw")
                return timed_result(refused("stale command or not in raw commissioning mode"));
            request->axis = static_cast<uint8_t>(axis);
            request->stamp = rclcpp::Time(static_cast<int64_t>(sent * 1e9));
            request->motor_erpm = data.at("motor_erpm").get<double>();
            request->servo_position = data.at("servo_position").get<double>();
            request->duration_s = data.at("duration_s").get<double>();
        }
        if (!commissioning_->service_is_ready()) return timed_result(refused("commissioning service unavailable"));
        auto future = commissioning_->async_send_request(request);
        if (future.wait_for(chrono::milliseconds(250)) != future_status::ready) {
            commissioning_->remove_pending_request(future);
            return timed_result(refused("commissioning response timeout; release hold"), true);
        }
        const auto result = future.get();
        if(result->accepted&&request->action!=Commissioning::Request::STOP&&request->axis==Commissioning::Request::STEERING&&
           request->motor_erpm==0&&record_command)recordSteeringCommand(*request);
        return timed_result({{"accepted", result->accepted}, {"reason", result->reason}});
    }
    static Json refused(const string& reason) { return {{"accepted", false}, {"reason", reason}}; }
    mutex status_mutex_;
    Json status_ = {{"state", "DISCONNECTED"}, {"reason", "waiting for controller"}};
    chrono::steady_clock::time_point received_{};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<Manual>::SharedPtr manual_;
    rclcpp::Client<Commissioning>::SharedPtr commissioning_;
    rclcpp::Client<SteeringRange>::SharedPtr steering_range_;
    rclcpp::Client<Trigger>::SharedPtr stop_, reset_;
};
} // namespace nomad

#ifndef NOMAD_GUI_NO_MAIN
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto bridge = std::make_shared<nomad::GuiBridge>();
        const auto page = ament_index_cpp::get_package_share_directory("nomad_control_cpp") + "/web/index.html";
        nomad::GuiHttp server(page, 8765, [bridge] { return bridge->status(); },
            [bridge](const string& route, const nomad::Json& data) { return bridge->command(route, data); });
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(bridge);
        // 메시지가 도착할 때 깨워 처리합니다. 유휴 상태의 2ms 폴링을 없앱니다.
        thread ros_thread([&] { executor.spin(); });
        thread watcher([&] { while (rclcpp::ok()) this_thread::sleep_for(chrono::milliseconds(50)); server.stop(); });
        RCLCPP_INFO(bridge->get_logger(), "GUI: http://127.0.0.1:8765 (SSH tunnel from Windows)");
        if (!server.listen()) result = 1;
        rclcpp::shutdown();
        watcher.join(); ros_thread.join();
    } catch (const exception& e) { cerr << e.what() << '\n'; result = 1; rclcpp::shutdown(); }
    return result;
}

#endif

// 실제 노드 구현을 가짜 VESC와 같은 executor에서 시험합니다. 시험 토픽은 고유 namespace에 격리합니다.
#include "nomad_control_cpp/controller_node.hpp"
#include "json.hpp"
#ifdef NOMAD_ENABLE_PLANNING
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#endif
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcutils/logging.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <map>
#include <regex>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <mutex>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <functional>
#include <tuple>
#include <cctype>
#include <optional>
#include <limits>
#define NOMAD_GUI_NO_MAIN
#include "../src/gui_server.cpp"

using namespace std;

using nomad::steadySeconds;

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; cerr << "FAIL: " << name << '\n'; }
}

// 2026-11-19: the controller's trace lines, captured from the ROS log (still printed as before), to check which
// lines the team GUI would see. Only the lines starting with a tag in kept[] below are kept.
mutex log_mutex;
vector<pair<rcutils_time_point_value_t, string>> trace_lines;  // log time (ns), text
rcutils_logging_output_handler_t previous_log_handler = nullptr;
void captureTraceLine(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    va_list copy;
    va_copy(copy, *args);
    char text[1024];
    vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    const string line(text);
    // 2026-11-19: also the lines the team GUI parses and the [CONFIG] warning.
    // 2026-11-20: and [LPP_LOST] / [LPP_BACK] (lost LPP input, and the first accepted command after it).
    static const char* const kept[] = {"[STOP_RAMP]", "[RESTART_TRACE]", "[PLANNING_WAIT]", "[CONFIG]", "[SLIP]",
                                       "[LPP_LOST]", "[LPP_BACK]", "control_trace ", "vesc_trace ", "link_trace "};
    bool keep = false;
    for (const char* tag : kept) keep = keep || line.rfind(tag, 0) == 0;
    if (keep) {
        lock_guard<mutex> lock(log_mutex);
        trace_lines.emplace_back(timestamp, line);
    }
    if (previous_log_handler) previous_log_handler(location, severity, name, timestamp, format, args);
}
#ifdef NOMAD_ENABLE_PLANNING  // only the planning stop-ramp checks read them
size_t traceMark() { lock_guard<mutex> lock(log_mutex); return trace_lines.size(); }
// Captured lines since mark that start with prefix (e.g. "[STOP_RAMP] end"), with their log time.
vector<pair<rcutils_time_point_value_t, string>> traceLinesSince(size_t mark, const string& prefix) {
    lock_guard<mutex> lock(log_mutex);
    vector<pair<rcutils_time_point_value_t, string>> out;
    for (size_t i = mark; i < trace_lines.size(); ++i)
        if (trace_lines[i].second.rfind(prefix, 0) == 0) out.push_back(trace_lines[i]);
    return out;
}
vector<string> traceSince(size_t mark, const string& prefix) {
    vector<string> out;
    for (const auto& line : traceLinesSince(mark, prefix)) out.push_back(line.second);
    return out;
}
// The integer after "name=" in a log line, or -1 (also for a value that is not a number, e.g. "na").
long traceNumber(const string& line, const string& name) {
    const auto at = line.find(" " + name + "=");
    if (at == string::npos) return -1;
    const char* start = line.c_str() + at + name.size() + 2;
    char* end = nullptr;
    const long value = strtol(start, &end, 10);
    return end == start ? -1 : value;
}

// 2026-11-19: what the team GUI reads from this node. The GUI is the team-shared original
// (NOMAD_실물시험앱_20261106/laptop_monitor); update these lists together with it.
// - Log lines (autonomy): developer_control.py trace_from_line, vesc_status.py vesc_trace_from_line,
//   link_trace_from_line and slip_event_from_line. The board tail (board_tools/module_log_tail.cpp) cuts a line at
//   400 characters, counted with the rclcpp prefix "[INFO] [<sec>.<nsec>] [nomad_vesc_trial]: " (50). The GUI splits
//   values at " key=", so a value that contains "word=" would break the key before it.
struct GuiLogLine { string marker; set<string> keys, numeric, na_allowed; };
const size_t kGuiLinePrefix = 50, kGuiLineCut = 400;
const vector<GuiLogLine> kGuiLogLines = {
    {"control_trace ", {"source", "phase", "requested_erpm", "target_erpm", "output_erpm", "measured_erpm", "current_a",
                        "esc_temp_c", "thermal", "stop", "stop_a", "erpm_valid", "start_gate", "reset_required"},
     {"requested_erpm", "target_erpm", "output_erpm", "measured_erpm", "current_a", "esc_temp_c", "stop_a"}, {}},
    {"vesc_trace ", {"voltage_v", "input_current_a", "power_w", "iq_a", "duty", "fault_code", "fault", "servo_cmd",
                     "servo_target", "hold_basis", "telemetry_age_ms", "hold_s"},
     {"voltage_v", "input_current_a", "power_w", "iq_a", "duty", "fault_code", "servo_cmd", "servo_target",
      "telemetry_age_ms", "hold_s"}, {}},
    {"link_trace ", {"n", "gap_ms", "age_min_ms", "age_max_ms", "expired", "tick_max_ms"},
     {"n", "gap_ms", "age_min_ms", "age_max_ms", "expired", "tick_max_ms"}, {"age_min_ms", "age_max_ms"}},
};
// - HTTP /api/status (키보드 조종): keyboard_drive.py (control_timing, _start, _fresh, _targets, servo_target),
//   vesc_status.py (status_rows, diagnostic_rows, _slip_status_row), drive_record_dialog.py (refresh_drive: reason)
//   and scripts/ensure_gui.sh run by remote/pi4_manual_prepare.sh (ground_drive_supported). /api/status is this
//   node's status_json plus connected (gui_server) and token / server_time_s (gui_http), not listed here.
const vector<string> kGuiStatusKeys = {
    "browser_freshness_s", "browser_request_timeout_ms", "command_max_age_s", "commissioning",
    "commissioning_setup_acknowledged", "continuous_manual_supported", "control_units", "driver_ready",
    "manual_reverse_max_erpm", "manual_timeout_s", "motor_max_erpm", "servo_center", "servo_max", "servo_min", "state",
    "stationary", "telemetry_fresh", "wifi_manual_profile",
    "body_speed_mps", "charge_drawn_ah", "charge_regen_ah", "commanded_motor_erpm", "commanded_servo_position",
    "control_tick_max_ms", "current_a", "duty", "energy_drawn_wh", "energy_regen_wh", "erpm_valid", "esc_mos_temps_c",
    "esc_temp_c", "esc_temp_state", "estimated_speed_mps", "estimated_speed_verified", "fault_code", "fault_name", "id_a",
    "input_current_a", "input_power_w", "iq_a", "link_cmd_age_max_ms", "link_cmd_age_min_ms", "link_cmd_max_gap_ms",
    "link_commands", "link_expiries", "raw_erpm", "requested_servo_position", "slip_events", "slip_off_reason",
    "slip_state", "slip_state_s", "stop_hold_basis", "stop_hold_heat_w", "stop_hold_long", "stop_hold_s", "stop_output",
    "stop_output_a", "target_motor_erpm", "telemetry_age_s", "vd_v", "vesc_fault", "voltage_v", "vq_v",
    "tacho", "tacho_abs", "vesc_controller_id", "reason", "ground_drive_supported"};

// keyboard_drive.control_timing: the keyboard window drives only when this is "WIFI" or "STRICT".
string guiControlTiming(const nomad::Json& data) {
    const auto is_true = [&](const char* key) { return data.contains(key) && data[key].is_boolean() && data[key].get<bool>(); };
    const auto equals = [&](const char* key, double value) {
        return data.contains(key) && data[key].is_number() && data[key].get<double>() == value; };
    if (is_true("wifi_manual_profile")) {
        const bool ok = data.value("control_units", string()) == "raw" && is_true("commissioning") &&
            is_true("continuous_manual_supported") && equals("manual_timeout_s", 2) && equals("command_max_age_s", 1.5) &&
            equals("browser_freshness_s", 1.5) && equals("browser_request_timeout_ms", 1500);
        return ok ? "WIFI" : "NONE";
    }
    if (data.contains("wifi_manual_profile") && !data["wifi_manual_profile"].is_null() &&
        !(data["wifi_manual_profile"].is_boolean() && !data["wifi_manual_profile"].get<bool>())) return "NONE";
    for (const auto& [key, limit] : vector<pair<const char*, double>>{{"manual_timeout_s", .35}, {"command_max_age_s", .3},
                                                                    {"browser_freshness_s", .3}, {"browser_request_timeout_ms", 600}}) {
        if (!data.contains(key) || data[key].is_null()) continue;
        if (!data[key].is_number() || data[key].is_boolean()) return "NONE";
        const double value = data[key].get<double>();
        if (!(value > 0 && value <= limit)) return "NONE";
    }
    return "STRICT";
}

// The GUI's key=value split (vesc_status._KEY): each value runs up to the next " key=".
map<string, string> guiFields(const string& payload) {
    static const regex key(R"((?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=)");
    vector<tuple<string, size_t, size_t>> marks;  // name, match start, value start
    for (sregex_iterator it(payload.begin(), payload.end(), key), end; it != end; ++it)
        marks.emplace_back((*it)[1].str(), static_cast<size_t>(it->position(0)),
                           static_cast<size_t>(it->position(0) + it->length(0)));
    map<string, string> out;
    for (size_t i = 0; i < marks.size(); ++i) {
        const size_t from = get<2>(marks[i]), to = i + 1 < marks.size() ? get<1>(marks[i + 1]) : payload.size();
        string value = payload.substr(from, to - from);
        while (!value.empty() && isspace(static_cast<unsigned char>(value.back()))) value.pop_back();
        out[get<0>(marks[i])] = value;
    }
    return out;
}
// Empty when the GUI reads the line as intended; otherwise what is wrong.
string guiLogProblem(const GuiLogLine& kind, const string& line) {
    if (line.size() + kGuiLinePrefix > kGuiLineCut) return "longer than the GUI's 400-character cut: " + line;
    const auto fields = guiFields(line.substr(kind.marker.size()));
    set<string> found;
    for (const auto& field : fields) found.insert(field.first);
    if (found != kind.keys) return "keys differ from what the GUI parses: " + line;
    for (const auto& name : kind.numeric) {
        const string& value = fields.at(name);
        if (value == "na" && kind.na_allowed.count(name)) continue;
        char* end = nullptr;
        const double number = strtod(value.c_str(), &end);
        if (value.empty() || *end != '\0' || !isfinite(number)) return "not a number for " + name + ": " + line;
    }
    // developer_control.trace_from_line / vesc_status.trace_rows accept exactly "0" or "1" for these two.
    if (kind.marker == "control_trace ")
        for (const char* flag : {"reset_required", "erpm_valid"})
            if (fields.at(flag) != "0" && fields.at(flag) != "1") return string(flag) + " must be 0 or 1: " + line;
    return "";
}
// vesc_status.py _SLIP_START / _SLIP_END.
bool guiSlipLine(const string& line) {
    static const regex start(R"(\[SLIP\]\s+(stuck|backslide)\s+start:\s*(.*)$)");
    static const regex end(R"(\[SLIP\]\s+(stuck|backslide)\s+end after\s+([0-9.]+)\s*s(?:,\s*([0-9.]+)\s*m backward)?)");
    return line.size() + kGuiLinePrefix <= kGuiLineCut && (regex_search(line, start) || regex_search(line, end));
}

// The names in "[CONFIG] set but not used (...): a, b" lines.
set<string> configUnused(const vector<string>& lines) {
    set<string> names;
    for (const auto& line : lines) {
        const auto colon = line.find("): ");
        if (colon == string::npos) continue;
        size_t from = colon + 3;
        while (from < line.size()) {
            const auto comma = line.find(", ", from);
            names.insert(line.substr(from, comma == string::npos ? string::npos : comma - from));
            if (comma == string::npos) break;
            from = comma + 2;
        }
    }
    return names;
}

// The parameters the --params-file arguments would give node_fqn: the files in order (later wins), "/**" entries
// first. Read with rcl's own parser, so the YAML rules are the ones the node uses.
vector<rclcpp::Parameter> paramsFromFiles(const vector<string>& files, const string& node_fqn) {
    rcl_params_t* params = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
    if (!params) throw runtime_error("rcl_yaml_node_struct_init failed");
    for (const auto& file : files)
        if (!rcl_parse_yaml_file(file.c_str(), params)) {
            rcl_yaml_node_struct_fini(params);
            throw runtime_error("cannot parse " + file);
        }
    const auto map = rclcpp::parameter_map_from(params);
    rcl_yaml_node_struct_fini(params);
    std::map<string, rclcpp::Parameter> merged;
    for (const string& key : vector<string>{"/**", node_fqn}) {
        const auto found = map.find(key);
        if (found == map.end()) continue;
        for (const auto& parameter : found->second) merged.insert_or_assign(parameter.get_name(), parameter);
    }
    vector<rclcpp::Parameter> out;
    for (const auto& entry : merged) out.push_back(entry.second);
    return out;
}
#endif

class RosRig {
public:
    explicit RosRig(bool bench, bool commission = false, bool setup_acknowledged = true,
                    bool timeout_verified = true, bool stall_watchdog = false,
                    const vector<rclcpp::Parameter>& extra_parameters = {}) {
        const auto token = chrono::steady_clock::now().time_since_epoch().count();
        prefix_ = "/nomad_cpp_test_" + to_string(token);
        fake_ = make_shared<rclcpp::Node>("fake_vesc", prefix_);
        pub_ = fake_->create_publisher<nomad::VescState>(prefix_ + "/sensors/core", 1);
        for (const auto& suffix : {"speed", "current"}) {
            subs_.push_back(fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/motor/" + suffix, 10,
                [this, name = string(suffix)](nomad::Scalar::ConstSharedPtr msg) {
                    output_log_.emplace_back(steadySeconds(), name == "speed" ? 'S' : 'C', msg->data);
                    if (name == "speed") {
                        ++speed_messages_; if (msg->data != 0.0) ++moving_messages_;
                        observed_motor_erpm_.push_back(msg->data);
                        speed_log_.emplace_back(steadySeconds(), msg->data);
                        if (follow_ && msg->data != 0.0) telemetry_erpm_ = min(follow_cap_, msg->data + follow_lag_);
                    }
                    else { ++release_messages_; expect(msg->data == 0.0, "release is zero current"); }
                }));
        }
        subs_.push_back(fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/servo/position", 10,
            [this](nomad::Scalar::ConstSharedPtr) { ++servo_messages_; }));
        subs_.push_back(fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/motor/brake", 10,
            [this](nomad::Scalar::ConstSharedPtr msg) {
                ++brake_messages_; last_brake_ = msg->data; stop_output_times_.push_back(steadySeconds());
                output_log_.emplace_back(steadySeconds(), 'B', msg->data);
                if (follow_) telemetry_erpm_ = 0.0;
            }));
        subscribeHandbrake();
        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + prefix_,
            "-r","/nomad/planning/drive_command:="+prefix_+"/planning",
            "-r","/nomad/control/vehicle_state:="+prefix_+"/vehicle_state",
            "-r","/nomad/localization/odometry:="+prefix_+"/odometry",
            "-r","/nomad/localization/status:="+prefix_+"/localization_status"});
        options.parameter_overrides({
            {"planning.start_hold_s",0.0},{"planning.restart_cooldown_s",0.0},
            {"mode", commission ? "commission" : (bench ? "bench" : "monitor")},
            {"calibration_verified", !commission}, {"hardware_timeout_verified", timeout_verified},
            {"commissioning_setup_acknowledged", setup_acknowledged},
            {"commissioning.stationary_hold_s", 0.2}, {"commissioning.max_duration_s", 1.0},
            {"commissioning.motor_max_erpm", 1000.0}, {"commissioning.overspeed_erpm", 1500.0},
            {"commissioning.stall_watchdog_enabled", stall_watchdog}, {"commissioning.stall_timeout_s", 0.5},
            {"target_speed_mps", 0.1}, {"target_steering_rad", 0.0},
            {"trial_duration_s", 1.0}, {"stationary_hold_s", 0.2}, {"steering_settle_s", 0.1},
            {"calibration.speed_gain", 6000.0}, {"calibration.speed_offset", 0.0},
            {"calibration.steering_gain", -0.6}, {"calibration.steering_offset", 0.52},
            {"calibration.command_erpm_sign", 1}, {"calibration.feedback_erpm_sign", -1},
            {"calibration.speed_min_mps", 0.0}, {"calibration.speed_max_mps", 0.4},
            {"calibration.steering_min_rad", -0.2}, {"calibration.steering_max_rad", 0.2},
            {"calibration.servo_min", 0.4}, {"calibration.servo_max", 0.64},
            {"calibration.motor_erpm_min", -3000.0}, {"calibration.motor_erpm_max", 3000.0},
            {"telemetry_topic", prefix_ + "/sensors/core"},
            {"motor_speed_topic", prefix_ + "/commands/motor/speed"},
            {"motor_current_topic", prefix_ + "/commands/motor/current"},
            {"servo_topic", prefix_ + "/commands/servo/position"},
            {"motor_duty_topic", prefix_ + "/commands/motor/duty_cycle"},
            {"motor_brake_topic", prefix_ + "/commands/motor/brake"},
            {"motor_handbrake_topic", prefix_ + "/commands/motor/handbrake"},
            {"motor_position_topic", prefix_ + "/commands/motor/position"}});
        auto overrides = options.parameter_overrides();
        overrides.emplace_back("stability.imu_topic", prefix_+"/imu_body");
        overrides.insert(overrides.end(), extra_parameters.begin(), extra_parameters.end());
        options.parameter_overrides(overrides);
        controller_ = nomad::ControllerNode::create(options);
        status_sub_ = fake_->create_subscription<std_msgs::msg::String>(prefix_ + "/nomad_vesc_trial/status_json", 1,
            [this](std_msgs::msg::String::ConstSharedPtr message) { status_ = nomad::Json::parse(message->data); });
        executor_.add_node(fake_);
        executor_.add_node(controller_);
    }

    ~RosRig() { controller_->prepareShutdown(); }

    // The patched vesc_driver subscribes to the handbrake topic; tests drop and restore it.
    void subscribeHandbrake() {
        handbrake_sub_ = fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/motor/handbrake", 10,
            [this](nomad::Scalar::ConstSharedPtr msg) {
                ++handbrake_messages_; last_handbrake_ = msg->data; stop_output_times_.push_back(steadySeconds());
                output_log_.emplace_back(steadySeconds(), 'H', msg->data);
                if (follow_) telemetry_erpm_ = 0.0;
            });
    }

    void pump(double seconds) {
        const auto end = steadySeconds() + seconds;
        while (steadySeconds() < end) {
            if (send_telemetry_ && steadySeconds() - last_sent_ > 0.025) {
                nomad::VescState msg;
                msg.header.stamp = fake_->now();
                msg.state.speed = telemetry_erpm_;
                // 2026-11-17: lone sensorless (HFI) ERPM readings, as seen at 57 of 95 field ramp ends on 11-16.
                if (glitch_samples_ > 0) { msg.state.speed = glitch_erpm_; --glitch_samples_; }
                msg.state.current_motor = 0.0;
                msg.state.avg_iq = telemetry_iq_;
                msg.state.current_input = 1.5;
                msg.state.voltage_input = 12.0;
                // Display-only values (status_json / vesc_trace); no control decision uses them.
                msg.state.duty_cycle = 0.05; msg.state.avg_id = 0.25; msg.state.avg_vd = -0.5; msg.state.avg_vq = 1.5;
                msg.state.charge_drawn = 0.125; msg.state.charge_regen = 0.0625; msg.state.energy_drawn = 2.5;
                msg.state.energy_regen = 1.25; msg.state.displacement = 1234; msg.state.distance_traveled = 4321;
                msg.state.controller_id = 111;
                msg.state.fault_code = telemetry_fault_;
                msg.state.ntc_temp_mos1 = msg.state.ntc_temp_mos2 = msg.state.ntc_temp_mos3 = telemetry_temp_c_;
                pub_->publish(msg);
                if (imu_pub_ && send_imu_) {
                    sensor_msgs::msg::Imu imu;
                    imu.header.stamp=fake_->now(); imu.header.frame_id=imu_frame_;
                    imu.orientation.w=cos(imu_roll_/2); imu.orientation.x=sin(imu_roll_/2);
                    imu.orientation_covariance[0]=imu.orientation_covariance[4]=imu.orientation_covariance[8]=0.001;
                    imu.linear_acceleration.z=9.80665;
                    imu_pub_->publish(imu);
                }
                last_sent_ = steadySeconds();
#ifdef NOMAD_ENABLE_PLANNING
                if(planning_pub_&&send_planning_)publishPlanning();
                // 2026-11-17 (4a): fake fused pose (body along +x at body_mps_) and localization status.
                if(odom_pub_&&send_pose_){
                    const double now_s=steadySeconds();
                    if(last_pose_at_>0)pose_x_+=body_mps_*(now_s-last_pose_at_);
                    last_pose_at_=now_s;
                    nav_msgs::msg::Odometry odom;odom.header.stamp=fake_->now();odom.header.frame_id="odom";
                    odom.pose.pose.position.x=pose_x_;odom.pose.pose.orientation.w=1.0;odom_pub_->publish(odom);
                    nomad_interfaces::msg::ModuleStatus st;st.header.stamp=fake_->now();
                    st.state=localization_ready_?nomad_interfaces::msg::ModuleStatus::READY:nomad_interfaces::msg::ModuleStatus::WAITING;
                    loc_status_pub_->publish(st);
                }
#endif
            }
#ifdef NOMAD_ENABLE_PLANNING
            // Keeps the LPP stream alive while telemetry is withheld (telemetry-loss tests).
            else if(planning_without_telemetry_&&planning_pub_&&send_planning_&&steadySeconds()-last_sent_>0.025){
                publishPlanning();last_sent_=steadySeconds();
            }
#endif
            executor_.spin_some();
            this_thread::sleep_for(chrono::milliseconds(3));
        }
    }

#ifdef NOMAD_ENABLE_PLANNING
    void publishPlanning() {
        nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();cmd.header.frame_id="base_link";
        cmd.target_speed=planner_speed_;cmd.target_steering_angle=.01;cmd.valid_for.nanosec=300000000;
        planning_pub_->publish(cmd);
    }
#endif

    bool call(const string& action) {
        auto client = fake_->create_client<nomad::Trigger>(prefix_ + "/nomad_vesc_trial/" + action);
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.02);
        if (!client->service_is_ready()) throw runtime_error("test service discovery timed out");
        auto future = client->async_send_request(make_shared<nomad::Trigger::Request>());
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline)
            pump(0.01);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready)
            throw runtime_error("test service response timed out");
        return future.get()->success;
    }

    bool manual(int action, const string& session, uint64_t sequence = 1, double age = 0.0) {
        auto client = fake_->create_client<nomad::Manual>(prefix_ + "/nomad_vesc_trial/manual_control");
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.01);
        auto request = make_shared<nomad::Manual::Request>();
        request->action = static_cast<uint8_t>(action); request->session = session; request->sequence = sequence;
        request->stamp = rclcpp::Time(static_cast<int64_t>((fake_->now().seconds() - age) * 1e9));
        request->speed_mps = 0.1; request->steering_rad = 0.0; request->duration_s = 1.0;
        auto future = client->async_send_request(request);
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline) pump(0.005);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready) throw runtime_error("manual service timeout");
        return future.get()->accepted;
    }

    bool commission(int action, int axis, const string& session, uint64_t sequence = 1,
                    double erpm = 300.0, double servo = 0.50, double duration = 1.0, double age = 0.0) {
        auto client = fake_->create_client<nomad::Commissioning>(prefix_ + "/nomad_vesc_trial/commissioning_control");
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.01);
        auto request = make_shared<nomad::Commissioning::Request>();
        request->action = static_cast<uint8_t>(action); request->session = session; request->sequence = sequence;
        if (action != nomad::Commissioning::Request::STOP) {
            request->axis = static_cast<uint8_t>(axis);
            request->stamp = rclcpp::Time(static_cast<int64_t>((fake_->now().seconds() - age) * 1e9));
            request->motor_erpm = erpm; request->servo_position = servo; request->duration_s = duration;
        }
        auto future = client->async_send_request(request);
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline) pump(0.005);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready)
            throw runtime_error("commissioning service timeout");
        return future.get()->accepted;
    }

    // 2026-11-17 (5a): rules that hold for every motor command the fake VESC received in this rig (commission/LPP).
    void checkOutputInvariants(const string& label, double hold_min, double hold_max, bool fault_injected,
                               double max_erpm = 1000.0) {
        bool hold_ok = true, release_ok = true, forward_only = true, bounded = true;
        for (const auto& [time, kind, value] : output_log_) {
            (void)time;
            if (kind == 'H') hold_ok = hold_ok && value >= hold_min - 1e-9 && value <= hold_max + 1e-9;
            if (kind == 'C') release_ok = release_ok && fault_injected;
            if (kind == 'S') { forward_only = forward_only && value >= 0.0; bounded = bounded && value <= max_erpm + 1e-9; }
        }
        expect(hold_ok, (label + "every handbrake command is inside the hold range").c_str());
        expect(release_ok, (label + "0 A release only after a VESC fault").c_str());
        expect(forward_only && bounded, (label + "LPP speed commands stay forward and within the ERPM limit").c_str());
    }

    // Nullable numeric status key (json::value() throws on null and would end the whole suite).
    double statusNumber(const string& key, double fallback) const {
        return status_.contains(key) && status_.at(key).is_number() ? status_.at(key).get<double>() : fallback;
    }

    // 2026-11-17: ~/steering_test_range with an explicit wire stamp (a malformed one must be refused, not crash).
    bool steeringRange(int32_t sec, uint32_t nanosec, double span = 0.1) {
        auto client = fake_->create_client<nomad::SteeringRange>(prefix_ + "/nomad_vesc_trial/steering_test_range");
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.01);
        auto request = make_shared<nomad::SteeringRange::Request>();
        request->stamp.sec = sec; request->stamp.nanosec = nanosec; request->span = span;
        auto future = client->async_send_request(request);
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline) pump(0.005);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready) throw runtime_error("steering range service timeout");
        return future.get()->accepted;
    }

    void runCommissionGate() {
        pump(0.7);
        expect(!commission(1, 1, string(32, '1')), "raw setup/timeout gate refuses start");
        pump(0.1);
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "raw gate refusal has no output");
    }

#ifdef NOMAD_ENABLE_PLANNING
    void runMinimumRunningFloor() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75);
        const auto outputs_at_least=[this](size_t first,double floor) {
            if(first==observed_motor_erpm_.size())return false;
            for(auto i=first;i<observed_motor_erpm_.size();++i)
                if(observed_motor_erpm_[i]+1e-6<floor)return false;
            return true;
        };

        auto first=observed_motor_erpm_.size();
        planner_speed_=1.0/4000.0;send_planning_=true;pump(.20);
        expect(abs(status_.value("planning_requested_erpm",0.0)-1.0)<1e-6&&
               outputs_at_least(first,700.0),
               "positive 1 ERPM planner request publishes at least the configured 700 ERPM floor");
        first=observed_motor_erpm_.size();pump(.95);
        expect(outputs_at_least(first,700.0)&&status_.value("autonomous_active",false),
               "700 ERPM floor remains active beyond the former 0.8 second startup duration");

        first=observed_motor_erpm_.size();planner_speed_=206.0/4000.0;pump(.15);
        expect(abs(status_.value("planning_requested_erpm",0.0)-206.0)<1e-6&&
               outputs_at_least(first,700.0),
               "206 ERPM planner request continues publishing at least 700 ERPM");
        first=observed_motor_erpm_.size();planner_speed_=930.0/4000.0;pump(.40);
        expect(outputs_at_least(first,700.0)&&observed_motor_erpm_.back()+1e-6>=930.0&&
               abs(status_.value("target_motor_erpm",0.0)-930.0)<1e-6,
               "930 ERPM planner request ramps from the floor to its larger requested output");

        const int zero_releases=release_messages_;planner_speed_=0.0;pump(.06);
        expect(release_messages_>zero_releases,"zero planner request promptly publishes zero-current release");
        const int zero_moves=moving_messages_;pump(.10);
        expect(moving_messages_==zero_moves&&status_.value("commanded_motor_erpm",-1.0)==0.0&&
               status_.value("target_motor_erpm",-1.0)==0.0,
               "zero planner request stays released without a low-speed deceleration ramp");
        first=observed_motor_erpm_.size();planner_speed_=206.0/4000.0;pump(.15);
        expect(outputs_at_least(first,700.0),"fresh positive planning command resumes the running floor after zero");

        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;
        const int stop_releases=release_messages_;planning_pub_->publish(stop);pump(.06);
        expect(release_messages_>stop_releases,"LPP STOP promptly publishes zero-current release");
        const int stopped=moving_messages_;pump(.10);
        expect(moving_messages_==stopped&&status_.value("planning_soft_stopped",false)&&
               !status_.value("planning_reset_required",true),
               "LPP STOP remains released and keeps its existing resumable pause behavior");
        first=observed_motor_erpm_.size();planner_speed_=1.0/4000.0;send_planning_=true;pump(.16);
        expect(outputs_at_least(first,700.0)&&status_.value("autonomous_active",false)&&
               !status_.value("planning_reset_required",true),
               "fresh planning command resumes at the floor without a startup cooldown");
    }

    // 실제 드라이버 없이 가짜 telemetry/DriveCommand만으로 새 속도 피드백 분기를 검사합니다.
    void runMeasuredSpeedFeedback() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75); // 발견·초기 정지 확인은 이동 명령 없이 끝냅니다.
        planner_speed_=.0515;send_planning_=true;pump(.18); // 206 ERPM, 기존 550 입력 gate 미만
        expect(moving_messages_>0,"measured-speed mode starts low valid request without fixed 750ms dwell");
        expect(abs(status_.value("planning_requested_erpm",0.0)-206.0)<1e-6&&
               status_.value("target_motor_erpm",0.0)>0&&status_.value("target_motor_erpm",1000.0)<=206.000001,
               "measured-speed mode preserves 206 ERPM request without blocking or boosting it to 550");

        telemetry_erpm_=300.0;send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;planning_pub_->publish(stop);pump(.08);
        const int paused=moving_messages_;
        expect(!status_.value("planning_reset_required",true),"fresh LPP pause stays resumable with rolling feedback");
        send_planning_=true;pump(.14);
        expect(moving_messages_>paused&&status_.value("autonomous_active",false)&&
               !status_.value("planning_reset_required",true),
               "fresh LPP command resumes while rolling without 500ms stationary or one-second cooldown wait");

        planner_speed_=.232625;telemetry_erpm_=300.0;pump(.25); // requested 930.5 ERPM
        telemetry_erpm_=1438.0;pump(.36); // 과거 1200/0.2s latch 조건, hard 2500 미만
        const double reduced=status_.value("target_motor_erpm",-1.0);
        expect(!status_.value("planning_reset_required",true)&&status_.value("state","")!="FAULT"&&
               status_.value("autonomous_active",false),
               "sustained modest overspeed reduces output instead of latching reset or fault");
        expect(reduced>=0.0&&reduced<930.5&&status_.value("commanded_motor_erpm",1000.0)<930.5,
               "modest measured overspeed actually reduces fake VESC speed output below requested 930.5 ERPM");
        telemetry_erpm_=300.0;pump(.35);
        expect(status_.value("target_motor_erpm",0.0)>reduced&&
               status_.value("target_motor_erpm",1000.0)<=930.500001&&
               !status_.value("planning_reset_required",true)&&status_.value("autonomous_active",false),
               "speed recovery automatically restores bounded demand without operator reset");
    }

    void runMeasuredSpeedSafety(const string& failure) {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75);
        planner_speed_=.0515;send_planning_=true;pump(.18);
        expect(moving_messages_>0,"measured-speed safety fixture begins with fake output only");
        if(failure=="operator STOP") {
            expect(call("stop"),"measured-speed mode accepts explicit operator STOP");pump(.13);
        }
        else if(failure=="invalid command") {
            send_planning_=false;
            const size_t lost_mark=traceMark();
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();cmd.header.frame_id="base_link";
            cmd.valid_for.nanosec=300000000;cmd.target_speed=numeric_limits<double>::quiet_NaN();
            planning_pub_->publish(cmd);pump(.13);
            // 2026-11-20: the subscription path's [LPP_LOST] line (an invalid move while driving).
            const auto lost=traceSince(lost_mark,"[LPP_LOST]");
            const string line=lost.empty()?string():lost.front();
            const auto why=line.find(" why=");
            expect(lost.size()==1&&line.find(" valid_ms=300 ")!=string::npos&&traceNumber(line,"age_ms")>=0&&
                   why!=string::npos&&line.substr(why)==" why=invalid/stale planning command",
                   "measured-speed safety: an invalid move logs one [LPP_LOST] line with why=invalid/stale planning command");
            if(lost.size()!=1)for(const auto& l:lost)cerr<<"  "<<l<<'\n';
        } else if(failure=="command expiry") {send_planning_=false;pump(.45);}
        else if(failure=="telemetry timeout") {send_telemetry_=false;pump(.45);}
        else if(failure=="VESC fault") {
            telemetry_fault_=1;
            // observe()가 먼저 FAULT를 만들고 다음 제어 tick이 headless latch를 갱신합니다.
            // 수신 status는 10Hz이므로 두 번의 발행까지 진행한 뒤 두 상태를 함께 검사합니다.
            pump(.25);
        }
        else if(failure=="hard overspeed") {telemetry_erpm_=2501.0;pump(.25);} // same two-status window as VESC fault
        else throw runtime_error("unknown measured-speed safety case");
        // Current policy (09 auto-resume profile, 11-11 critical split): operator STOP and
        // physical faults latch until RESET; LPP input loss and telemetry loss only pause.
        const bool critical=failure=="operator STOP"||failure=="VESC fault"||failure=="hard overspeed";
        const int stopped=moving_messages_;
        if(critical){
            expect(status_.value("planning_reset_required",false)&&!status_.value("autonomous_active",true),
                   ("measured-speed mode preserves latched stop for "+failure).c_str());
            send_planning_=true;send_telemetry_=true;telemetry_erpm_=0;telemetry_fault_=0;pump(.25);
            expect(moving_messages_==stopped&&status_.value("planning_reset_required",false),
                   ("fresh commands cannot undo "+failure+" without reset").c_str());
        }else{
            expect(!status_.value("planning_reset_required",true)&&!status_.value("autonomous_active",true),
                   ("measured-speed mode pauses without latching for "+failure).c_str());
            // 2026-11-17: a pause from lost/invalid LPP input waits for 0.3 s of continuous commands
            // (planning.link_resume_stable_s) before it resumes; a telemetry loss resumes at once.
            const bool link_loss=failure=="command expiry"||failure=="invalid command";
            send_planning_=true;send_telemetry_=true;telemetry_erpm_=0;telemetry_fault_=0;pump(link_loss?.7:.35);
            expect(moving_messages_>stopped&&status_.value("autonomous_active",false)&&
                   !status_.value("planning_reset_required",true),
                   ("fresh commands resume after "+failure+" without reset").c_str());
        }
    }

    void runStableHeadlessStart() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        planner_speed_=.1;pump(.8);
        expect(moving_messages_==0,"autonomy never boosts unsupported low speed to minimum ERPM");
        expect(status_.value("planning_minimum_start_erpm",0.0)==550,"autonomy exposes startup threshold");
        planner_speed_=.2;pump(.1);expect(moving_messages_==0,"short valid burst cannot start motor");
        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;planning_pub_->publish(stop);pump(.1);
        send_planning_=true;pump(.1);expect(moving_messages_==0,"pause resets stability dwell");
        pump(.5);expect(moving_messages_>0,"continuous qualifying motion starts without GUI");
        planner_speed_=.1;pump(.15);
        expect(status_.value("target_motor_erpm",-1.0)==0,"low requested speed inhibits output instead of boosting");
        planner_speed_=.2;const int before=moving_messages_;pump(.1);
        expect(moving_messages_==before,"zero-to-motion restart observes cooldown and dwell");
        pump(.5);expect(moving_messages_>before,"motion resumes after continuous stable stream");
        telemetry_erpm_=1400;pump(.5); // 0.2 s rule + telemetry + 10 Hz status publication
        expect(status_.value("planning_reset_required",false),"autonomy sustained measured overspeed latches stop");
        expect(!status_.at("planning_requested_erpm").is_number()||status_.at("planning_requested_erpm").get<double>()!=1400,
               "requested and measured ERPM remain distinct");
    }
    void runHeadlessResume() {
        bool vehicle_ready=false;
        auto vehicle=fake_->create_subscription<nomad_interfaces::msg::VehicleState>(prefix_+"/vehicle_state",1,
            [&vehicle_ready](nomad_interfaces::msg::VehicleState::ConstSharedPtr msg){
                vehicle_ready=msg->ready;
                expect(!msg->steering_valid&&!msg->speed_valid,"initial mapping never pretends measured feedback");
            });
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.6);
        const auto stop=[&]{
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();
            cmd.header.frame_id="base_link";cmd.valid_for.nanosec=300000000;cmd.stop_requested=true;
            planning_pub_->publish(cmd);pump(.13);
        };
        const int initial_release=release_messages_;
        stop();
        expect(moving_messages_==0&&servo_messages_==0&&release_messages_==initial_release,
               "initial planner STOP has no motor/servo output");
        expect(!status_.value("planning_reset_required",true),"initial planner STOP keeps headless ready");
        expect(vehicle_ready,"initial planner STOP keeps VehicleState ready");
        send_planning_=true;pump(.4);
        expect(moving_messages_>0,"fresh command starts after initial planner STOP");
        send_planning_=false;telemetry_erpm_=100;stop();const int stopped=moving_messages_;
        expect(status_.value("planning_soft_stopped",false)&&!status_.value("planning_reset_required",true),
               "normal planner STOP is a resumable pause");
        expect(vehicle_ready,"soft pause retains VehicleState ready");
        send_planning_=true;pump(.2);
        expect(moving_messages_==stopped,"pause cannot resume before stationary hold");
        telemetry_erpm_=0;pump(.45);
        expect(moving_messages_>stopped,"fresh command resumes after stationary hold");
        send_planning_=false;pump(.45);const int expired=moving_messages_;
        pump(.1);
        expect(moving_messages_==expired&&!status_.value("planning_reset_required",true),
               "planner lease expiry releases output without a reset latch");
        expect(vehicle_ready,"module expiry retains VehicleState ready");
        send_planning_=true;pump(.7); // 2026-11-17: + the 0.3 s link-loss resume wait
        expect(moving_messages_>expired&&!status_.value("planning_reset_required",true),
               "fresh planner stream resumes after lease expiry without reset");
        send_planning_=false;pump(.45);stop();const int stopped_again=moving_messages_;
        send_planning_=true;pump(.4);
        expect(moving_messages_>stopped_again&&!status_.value("planning_reset_required",true),
               "STOP after expired command remains an automatically resumable module pause");
        expect(call("stop"),"operator STOP accepted with pause resume enabled");const int operator_stop=moving_messages_;
        send_planning_=false;stop();send_planning_=true;pump(.2);
        expect(moving_messages_==operator_stop&&status_.value("planning_reset_required",false),
               "planner pause cannot undo operator STOP");
        expect(!status_.value("planning_calibration_verified",true),"pause resume never fakes physical calibration");
    }
    // 2026-11-13: deployed stop profile (brake 20 A, handbrake 15 A at or below 200 ERPM) on the
    // headless planning path. Counts are taken right before each step so a check cannot pass on
    // output from an earlier step (the start tick itself may publish one hold before the first speed).
    void runPlanningStopOutput() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.6);
        const auto stop=[&]{
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();
            cmd.header.frame_id="base_link";cmd.valid_for.nanosec=300000000;cmd.stop_requested=true;
            planning_pub_->publish(cmd);pump(.13);
        };
        const auto stop_output=[&]{return status_.value("stop_output",string());};
        stop();
        expect(handbrake_messages_==0&&brake_messages_==0&&release_messages_==0,
               "stop profile: STOP before any drive sends no stop output");
        expect(stop_output()=="none"&&status_.value("erpm_valid",false)&&!status_.value("planning_reset_required",true),
               "stop profile: no stop output and still ready before ownership");
        int moves=moving_messages_;
        send_planning_=true;pump(.4);
        expect(moving_messages_>moves,"stop profile: LPP drive starts");
        // STOP at standstill: handbrake hold, never 0A.
        int holds=handbrake_messages_;
        send_planning_=false;telemetry_erpm_=0.0;stop();pump(.1);
        expect(handbrake_messages_>holds+3&&last_handbrake_==15.0&&release_messages_==0,
               "stop profile: LPP STOP at standstill holds with the 15 A handbrake");
        expect(stop_output()=="hold"&&!status_.value("erpm_valid",true)&&status_.value("stop_output_a",0.0)==15.0,
               "stop profile: status reports the hold and frozen ERPM");
        expect(abs(statusNumber("stop_hold_heat_w",-1.0)-1.5*0.0112*15.0*15.0)<1e-9&&!status_.value("stop_hold_long",true),
               "stop profile: status estimates the hold heat (15 A, short hold: no long-hold flag)");
        // Resume ends the hold.
        moves=moving_messages_;
        send_planning_=true;pump(.5);
        expect(moving_messages_>moves&&stop_output()=="none"&&status_.value("erpm_valid",false),
               "stop profile: resume ends the hold and ERPM is valid again");
        expect(status_.value("restart_trace_active",false),"stop profile: a restart after the hold starts the restart trace");
        // STOP while still turning: brake first, hold once slow.
        telemetry_erpm_=600.0;pump(.1); // the stop must see a fresh sample above the threshold
        holds=handbrake_messages_;int brakes=brake_messages_;
        send_planning_=false;stop();pump(.15);
        expect(brake_messages_>brakes&&last_brake_==20.0&&handbrake_messages_==holds,
               "stop profile: stop above 200 ERPM brakes with 20 A, no hold");
        expect(stop_output()=="brake"&&status_.value("erpm_valid",false),"stop profile: status reports braking with valid ERPM");
        telemetry_erpm_=0.0;pump(.2);
        expect(handbrake_messages_>holds,"stop profile: hold starts once telemetry is at or below 200 ERPM");
        // Latched up to the release threshold (400), given up above it.
        brakes=brake_messages_;
        telemetry_erpm_=300.0;pump(.25);
        expect(brake_messages_==brakes&&stop_output()=="hold","stop profile: the hold stays latched up to 400 ERPM");
        telemetry_erpm_=600.0;pump(.25);
        expect(brake_messages_>brakes&&stop_output()=="brake"&&status_.value("stop_holds_abandoned",0)>=1,
               "stop profile: a fresh reading above 400 ERPM means the hold is not working; brake instead");
        // Overspeed (not a VESC fault) latches but keeps braking, never 0A.
        int releases=release_messages_;brakes=brake_messages_;
        telemetry_erpm_=1600.0;pump(.3);
        expect(status_.value("planning_reset_required",false)&&brake_messages_>brakes&&release_messages_==releases&&
               !status_.value("vesc_fault",true)&&stop_output()=="brake",
               "stop profile: overspeed latch brakes instead of releasing with 0A");
        telemetry_erpm_=0.0;pump(.3);
        expect(stop_output()=="hold"&&release_messages_==releases,"stop profile: latched overspeed holds once slow");
        // VESC fault code: 0A, no brake or hold.
        releases=release_messages_;
        telemetry_fault_=7;pump(.3);
        expect(release_messages_>releases&&status_.value("vesc_fault",false)&&stop_output()=="coast",
               "stop profile: VESC fault code releases with 0A");
        holds=handbrake_messages_;brakes=brake_messages_;pump(.2);
        expect(handbrake_messages_==holds&&brake_messages_==brakes,"stop profile: no brake or hold to a faulted VESC");
        telemetry_fault_=0;pump(.4);
        expect(call("reset"),"stop profile: RESET accepted after the fault clears");
        holds=handbrake_messages_;pump(.3);
        expect(handbrake_messages_>holds&&!status_.value("vesc_fault",true)&&stop_output()=="hold",
               "stop profile: hold returns after RESET");
        // RESET cleared the VESC fault: a later non-VESC latch brakes again instead of 0A.
        releases=release_messages_;brakes=brake_messages_;
        telemetry_erpm_=1600.0;pump(.3);
        expect(brake_messages_>brakes&&release_messages_==releases&&!status_.value("vesc_fault",true),
               "stop profile: after RESET an overspeed latch brakes, the old VESC fault is gone");
        telemetry_erpm_=0.0;pump(.4);
        expect(call("reset"),"stop profile: RESET after the overspeed latch");
        // Telemetry loss with a stale 0 ERPM sample brakes and never enters the hold.
        moves=moving_messages_;
        send_planning_=true;pump(.6);
        expect(moving_messages_>moves,"stop profile: drive again after RESET");
        holds=handbrake_messages_;brakes=brake_messages_;
        planning_without_telemetry_=true;send_telemetry_=false;pump(.5);
        expect(brake_messages_>brakes&&handbrake_messages_==holds&&stop_output()=="brake",
               "stop profile: telemetry loss brakes and never holds on a stale sample");
        planning_without_telemetry_=false;send_telemetry_=true;send_planning_=false;pump(.2);
    }

    // 2026-11-13: load-sized handbrake (12.5..30 A from the drive's VESC avg_iq before a settled stop, gain
    // 1.45, rolling 6 A, margin 8 A) on the headless planning path. Current is set along the commanded direction;
    // "settled" telemetry follows the commanded ERPM (every 40 ms) after a 0.15 s start.
    void runPlanningLoadHold() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.6);
        const auto stop=[&]{
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();
            cmd.header.frame_id="base_link";cmd.valid_for.nanosec=300000000;cmd.stop_requested=true;
            planning_pub_->publish(cmd);pump(.13);
        };
        const auto basis=[&]{return status_.value("stop_hold_basis",string());};
        double sign=1.0;
        const auto drive_then_stop=[&](double start_iq,double iq,double seconds,bool settled){
            telemetry_iq_=start_iq*sign;telemetry_erpm_=0.0;const int moves=moving_messages_;
            send_planning_=true;pump(.15);
            expect(moving_messages_>moves&&!observed_motor_erpm_.empty()&&observed_motor_erpm_.back()!=0.0,"load hold: LPP drives");
            sign=observed_motor_erpm_.back()>0?1.0:-1.0;
            telemetry_iq_=iq*sign;
            // Settled: the reported ERPM follows the command (it may still be ramping up).
            for(double done=0.0;done<seconds;done+=.04){
                if(settled)telemetry_erpm_=observed_motor_erpm_.back();
                pump(.04);
            }
            send_planning_=false;stop();telemetry_erpm_=0.0;pump(.2);
        };
        // A first drive that never settles (stalled): not sized from its current, so the maximum.
        drive_then_stop(0.0,0.0,.25,false);
        expect(last_handbrake_==30.0&&basis()!="load"&&status_.value("stop_output",string())=="hold",
               "load hold: a drive that never settled holds with the maximum");
        expect(abs(statusNumber("stop_hold_heat_w",-1.0)-1.5*0.0112*30.0*30.0)<1e-9,
               "load hold: status estimates the 30 A hold heat (~15 W)");
        expect(status_.value("stop_handbrake_min_current_a",0.0)==12.5&&status_.value("stop_handbrake_current_a",0.0)==30.0&&
               status_.value("stop_handbrake_load_gain",0.0)==1.45&&status_.value("stop_handbrake_rolling_current_a",0.0)==6.0&&
               status_.value("stop_handbrake_margin_a",0.0)==8.0,"load hold: status reports the configuration");
        // Gentle climb: 16 A along the command = 10 A gravity over 6 A rolling -> 1.45 * 10 + 8 = 22.5 A.
        drive_then_stop(16.0,16.0,1.2,true);
        expect(abs(last_handbrake_-22.5)<1e-9&&basis()=="load"&&abs(status_.value("stop_output_a",0.0)-22.5)<1e-9&&
               abs(status_.value("stop_hold_load_a",0.0)-10.0)<1e-9,"load hold: settled climb current sizes the hold");
        // Downhill: the speed PID brakes, current against the command (4 A = 10 A gravity - 6 A rolling).
        drive_then_stop(-4.0,-4.0,1.2,true);
        expect(abs(last_handbrake_-22.5)<1e-9&&basis()=="load","load hold: downhill braking current sizes the same hold");
        // Flat ground after a 22.5 A hold: the minimum through the node path (the unit test covers ignoring the
        // restart samples that still carry the previous hold current).
        drive_then_stop(6.0,6.0,1.2,true);
        expect(last_handbrake_==12.5&&basis()=="load","load hold: flat ground after a load hold holds with the minimum");
        // A short stalled hop on the same spot keeps that hold current.
        drive_then_stop(6.0,6.0,.25,false);
        expect(last_handbrake_==12.5&&basis()=="same_place","load hold: a stalled hop keeps the previous hold current");
        // 14 deg climb (27 A) or steeper: the maximum; a stalled hill start from there keeps it.
        drive_then_stop(27.0,27.0,1.2,true);
        expect(last_handbrake_==30.0&&basis()=="load","load hold: steep climb holds with the maximum");
        drive_then_stop(15.0,15.0,1.2,false);
        expect(last_handbrake_==30.0&&basis()=="same_place","load hold: a stalled hill start keeps the previous hold current");
        expect(release_messages_==0,"load hold: never 0A without a VESC fault");
        send_planning_=true;pump(.3);
        expect(basis()=="none"&&status_.contains("stop_hold_load_a")&&status_.at("stop_hold_load_a").is_null(),
               "load hold: driving reports no hold basis");
        send_planning_=false;stop();pump(.2);
    }

    // 2026-11-13 stop ramp (planning.stop_ramp_s, running floor 1000 ERPM): an LPP STOP or lost LPP input ramps
    // the speed command down to the handbrake entry speed, keeps it there until the reported speed is there (at most
    // 0.6 s more), then the stop output; everything else still stops at once. ramp_s is 1.0 in the scenarios that act
    // during the ramp, so a slow machine still acts while it runs.
    // 2026-11-19: the node built from the deployed yaml files (topics moved to this rig). What the team GUI reads over
    // HTTP must be in status_json, with the deployed values.
    void runDeployedConfiguration(const vector<rclcpp::Parameter>& deployed) {
        const string label="deployed config: ";
        const auto check=[&](bool ok,const string& name){expect(ok,(label+name).c_str());};
        const size_t trace_mark=traceMark();
        const double trace_from=steadySeconds();
        for(const auto& parameter:deployed)
            check(controller_->has_parameter(parameter.get_name()),"the node reads "+parameter.get_name());
        const double deadline=steadySeconds()+2.0;
        while(steadySeconds()<deadline&&!(status_.contains("planning_stop_ramp_s")&&status_.contains("link_commands")))pump(.05);
        pump(.3);
        vector<string> missing;
        for(const auto& key:kGuiStatusKeys)if(!status_.contains(key))missing.push_back(key);
        string list;for(const auto& key:missing)list+=" "+key;
        check(missing.empty(),"status_json has every key the team GUI reads over HTTP (missing:"+list+")");
        const auto number=[&](const char* key){return status_.contains(key)&&status_[key].is_number()?status_[key].get<double>():NAN;};
        check(status_.value("control_units",string())=="raw"&&status_.value("commissioning",false)&&
              status_.value("commissioning_setup_acknowledged",false),"commission mode with the setup acknowledged");
        check(number("motor_max_erpm")==1850.0&&number("manual_reverse_max_erpm")==800.0&&number("servo_center")==0.5,
              "keyboard limits are the deployed ones (1850, reverse 800, servo center 0.5)");
        check(abs(number("planning_stop_ramp_s")-0.3)<1e-9&&abs(number("planning_link_resume_stable_s")-0.3)<1e-9&&
              status_.value("slip_enabled",false),"planning values are the deployed ones (ramp 0.3 s, link wait 0.3 s, slip on)");
        const string timing=guiControlTiming(status_);
        check(timing!="NONE","the keyboard window accepts the timing values (keyboard_drive.control_timing: "+timing+")");
        check(status_.contains("ground_drive_supported")&&status_["ground_drive_supported"].is_boolean()&&
              !status_["ground_drive_supported"].get<bool>(),"ground_drive_supported is false (scripts/ensure_gui.sh waits on it)");
        // control_trace / vesc_trace once a second (the GUI's autonomy rows), on the steady clock since 11-19.
        while(steadySeconds()-trace_from<2.6)pump(.05);
        const auto controls=traceSince(trace_mark,"control_trace ").size(),vescs=traceSince(trace_mark,"vesc_trace ").size();
        check(controls>=2&&controls<=3&&vescs==controls,"control_trace and vesc_trace once a second ("+
              to_string(controls)+" and "+to_string(vescs)+" lines in 2.6 s)");
    }

    void runStopRamp(const string& scenario,double ramp_s) {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        odom_pub_=fake_->create_publisher<nav_msgs::msg::Odometry>(prefix_+"/odometry",rclcpp::SensorDataQoS().keep_last(1));
        loc_status_pub_=fake_->create_publisher<nomad_interfaces::msg::ModuleStatus>(prefix_+"/localization_status",rclcpp::QoS(1).reliable());
        const auto lpp=[&](bool stop,double speed){
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();cmd.header.frame_id="base_link";
            cmd.valid_for.nanosec=300000000;cmd.stop_requested=stop;cmd.target_speed=speed;cmd.target_steering_angle=.01;
            planning_pub_->publish(cmd);
        };
        const string label="stop ramp ("+scenario+"): ";
        const auto check=[&](bool ok,const string& name){expect(ok,(label+name).c_str());};
        // status_json comes every 0.10-0.12 s: wait for it instead of reading it once after a fixed time.
        const auto wait_status=[&](const function<bool()>& ok,double seconds){
            const double deadline=steadySeconds()+seconds;
            while(!ok()&&steadySeconds()<deadline)pump(.01);
            return ok();
        };
        const auto ramp_running=[&]{return status_.value("stop_ramp_active",false)&&
            status_.value("planning_supervisor_state",string())=="ACTIVE"&&
            status_.value("planning_start_reason",string())=="speed ramp before stop";};
        const auto held=[&]{return status_.value("stop_output",string())=="hold"&&!status_.value("stop_ramp_active",true);};
        const bool climb=scenario=="climb hold"||scenario=="cancel then flat";
        send_planning_=false;pump(.75);
        follow_=true;telemetry_iq_=climb?16.0:6.0;planner_speed_=.25;send_planning_=true;pump(1.6);
        check(!observed_motor_erpm_.empty()&&observed_motor_erpm_.back()==1000.0&&telemetry_erpm_==1000.0,
              "the LPP drive runs at 1000 ERPM");
        // 2026-11-20: the ramp start uses the highest of the last 3 readings within 0.1 s (kStopRampStartWindow), so
        // all of them must be 0 here: 0.15 s of 0 ERPM telemetry (about 5 samples) pushes the 1000s out.
        if(scenario=="blocked"){follow_=false;telemetry_erpm_=0.0;pump(.15);}
        const size_t mark=speed_log_.size(),outputs=stop_output_times_.size(),ramp_log=traceMark();
        const int brakes=brake_messages_,handbrakes=handbrake_messages_,releases=release_messages_;
        const double t0=steadySeconds();
        const auto speeds_after=[&](size_t from){vector<double> v;for(size_t i=from;i<speed_log_.size();++i)v.push_back(speed_log_[i].second);return v;};
        const auto first_output_after=[&]{return stop_output_times_.size()>outputs?stop_output_times_[outputs]-t0:-1.0;};
        const auto falling_until_output=[&](size_t from,double low){
            vector<double> v;
            for(size_t i=from;i<speed_log_.size();++i)
                if((first_output_after()<0||speed_log_[i].first<t0+first_output_after())&&speed_log_[i].second<1000.0)
                    v.push_back(speed_log_[i].second);
            bool ok=v.size()>=8;
            for(size_t i=1;i<v.size();++i)ok=ok&&v[i]<=v[i-1]&&v[i]>=low;
            return ok&&!v.empty()&&v.back()<=200.0+1e-9;
        };
        send_planning_=false;
        if(scenario=="second publisher"){
            send_planning_=true; // the first planner keeps sending fresh commands
            auto second=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
            check(wait_status([&]{return held();},3.0),"a second planning publisher ramps down to the hold");
            check(falling_until_output(mark,200.0)&&handbrake_messages_>handbrakes,
                  "fresh commands from the first planner do not cancel the ramp while two publishers exist");
            check(status_.value("planning_supervisor_state",string())!="ACTIVE"&&!status_.value("planning_reset_required",true),
                  "it waits without a latch");
            return;
        }
        if(scenario!="expiry"&&scenario!="link resume"&&scenario!="shutdown while driving"&&scenario!="slip display")lpp(true,0.0);
        if(scenario=="climb hold"){telemetry_iq_=8.0;follow_lag_=150.0;}
        if(scenario=="LPP STOP"||scenario=="expiry"||scenario=="climb hold"){
            check(wait_status(ramp_running,scenario=="expiry"?.6:.25)&&status_.value("planning_stop_ramp_s",0.0)==ramp_s,
                  "status reports the running ramp");
            check(wait_status(held,2.0),"the ramp ends in a hold");
            pump(.15);
            const double until=first_output_after(),lead=scenario=="expiry"?.3:0.0;
            if(scenario=="climb hold"){
                // The reported speed stays 150 ERPM above the command: 0.6 s at 200 ERPM, then brake, then hold.
                check(until>=.8&&until<=1.3&&brake_messages_>brakes,
                      "a speed still above 200 after the ramp waits 0.6 s at 200 ERPM, then brakes before the hold");
            }else{
                check(falling_until_output(mark,200.0),"the speed command falls step by step from 1000 to 200 ERPM");
                check(until>=lead+.24&&until<=lead+.55,"the stop output follows about 0.3 s after the stop"+
                      string(scenario=="expiry"?" (after the 0.3 s command validity)":""));
                check(brake_messages_==brakes,"a speed that follows the ramp goes straight to the handbrake (no brake pulse)");
            }
            check(handbrake_messages_>handbrakes&&status_.value("stop_hold_basis",string())=="load"&&
                  status_.value("planning_supervisor_state",string())=="SOFT_STOPPED"&&
                  !status_.value("planning_reset_required",true),"the ramp ends in the load-sized hold and a planning pause");
            check(last_handbrake_==(climb?1.45*10.0+8.0:12.5),
                  climb?"a lagging ramp after a climb holds with the climb current (22.5 A)":
                        "a flat ramp holds with the minimum (12.5 A)");
            if(!climb)check(status_.value("planning_wait_reason",string()).find(
                scenario=="expiry"?"expired":"LPP pause")!=string::npos,"the wait reason names the stop");
            follow_lag_=0.0;telemetry_iq_=6.0;send_planning_=true;pump(scenario=="expiry"?.8:.5); // expiry: + 0.3 s link wait
            check(observed_motor_erpm_.back()==1000.0&&status_.value("planning_supervisor_state",string())=="ACTIVE",
                  "a fresh command resumes after the ramp stop");
        }else if(scenario=="cancel"||scenario=="cancel then flat"){
            pump(.1);
            // The drive after the cancel is flat; the cancelled ramp's record (climb, 10 A load) must not size its stop.
            if(scenario=="cancel then flat")telemetry_iq_=6.0;
            send_planning_=true;pump(.4);
            const auto v=speeds_after(mark);
            double lowest=1000.0;for(double x:v)lowest=min(lowest,x);
            check(stop_output_times_.size()==outputs&&lowest<1000.0&&lowest>200.0&&v.back()==1000.0&&
                  wait_status([&]{return !status_.value("stop_ramp_active",true)&&
                      status_.value("planning_supervisor_state",string())=="ACTIVE";},.3),
                  "a fresh command during the ramp cancels it: no brake/hold, back to 1000 ERPM");
            if(scenario=="cancel then flat"){
                // STOP ~0.75 s after the climb (inside kLoadWindowS: a kept record would still hold 22.5 A), with
                // ~0.5 s of settled flat driving in the new record even on a slow machine.
                pump(.25);
                send_planning_=false;lpp(true,0.0);
                check(wait_status(held,ramp_s+1.0)&&last_handbrake_==12.5&&status_.value("stop_hold_basis",string())=="load",
                      "after a cancelled ramp, a later flat stop holds with the minimum (new drive record)");
            }
        }else if(scenario=="speed 0"){
            pump(.06);
            lpp(false,0.0);
            const size_t before=speed_log_.size();pump(.12);
            check(speed_log_.size()>=before+3&&stop_output_times_.size()==outputs&&status_.value("stop_ramp_active",false),
                  "a speed-0 LPP command during the ramp lets the ramp run on (no brake from cruise speed)");
            check(wait_status(held,ramp_s+1.0)&&falling_until_output(mark,200.0),"the ramp still ends in the hold");
        }else if(scenario=="operator STOP"){
            check(wait_status(ramp_running,.3),"the ramp runs before the operator STOP");
            check(call("stop"),"operator STOP accepted during the ramp");
            const size_t after_call=speed_log_.size();pump(.25);
            check(speed_log_.size()==after_call&&stop_output_times_.size()>outputs&&
                  status_.value("planning_reset_required",false),"operator STOP during the ramp stops at once and latches");
        }else if(scenario=="VESC fault"){
            check(wait_status(ramp_running,.3),"the ramp runs before the VESC fault");
            telemetry_fault_=1;pump(.06);
            const size_t after=speed_log_.size();pump(.2);
            check(speed_log_.size()==after&&release_messages_>releases&&status_.value("planning_reset_required",false),
                  "a VESC fault during the ramp releases with 0 A at once and latches");
        }else if(scenario=="blocked"){
            pump(.3);
            check(traceSince(ramp_log,"[STOP_RAMP] begin").empty()&&speeds_after(mark).size()<=1&&
                  first_output_after()>=0.0&&first_output_after()<.15,
                  "a car already at 0 ERPM is not ramped: no [STOP_RAMP] begin, the stop output comes at once");
        }else if(scenario=="invalid message"){
            check(wait_status(ramp_running,.3),"the ramp runs before the invalid message");
            const size_t before=speed_log_.size();
            nomad_interfaces::msg::DriveCommand bad;bad.header.stamp=fake_->now();bad.header.frame_id="base_link";
            bad.valid_for.nanosec=300000000;bad.target_speed=numeric_limits<double>::quiet_NaN();planning_pub_->publish(bad);
            pump(.12);
            check(speed_log_.size()>=before+3&&stop_output_times_.size()==outputs,
                  "an invalid LPP message during the ramp does not cut it short");
            check(wait_status(held,ramp_s+1.0)&&!status_.value("planning_reset_required",true),"the ramp still ends in the hold");
        }else if(scenario=="noisy end"){
            // 2nd bundle item 1 (11-18 rule: 2 of the last 3): a lone low ERPM reading (sensorless HFI noise) must not
            // end the ramp; two in a row do. On 11-16, 57 of 95 field ramps ended on a lone drop while the car moved.
            check(wait_status(ramp_running,.3),"the ramp runs before the ERPM glitch");
            const size_t outputs_before=stop_output_times_.size();
            glitch_samples_=1;pump(.12);
            check(stop_output_times_.size()==outputs_before&&status_.value("stop_ramp_active",false),
                  "one lone low ERPM sample does not end the ramp");
            glitch_samples_=2;
            check(wait_status(held,.35)&&stop_output_times_.size()>outputs_before,
                  "two low ERPM samples in a row end the ramp (the speed really dropped)");
        }else if(scenario=="link resume"){
            // 2026-11-17 (2nd bundle, item 2): a stop from LPP input loss (Wi-Fi) resumes only after 0.3 s of
            // continuous fresh commands; one late command does not restart the car. A planner STOP is not a link loss.
            check(wait_status(held,2.0),"an expiry ends in a hold");
            const auto active=[&]{return status_.value("planning_supervisor_state",string())=="ACTIVE";};
            nomad_interfaces::msg::DriveCommand lone;lone.header.stamp=fake_->now();lone.header.frame_id="base_link";
            lone.valid_for.nanosec=200000000;lone.target_speed=.25;lone.target_steering_angle=.01;
            const int moves=moving_messages_;
            planning_pub_->publish(lone);
            // While that command is still valid (0.2 s < the 0.3 s check) the wait names the link check.
            check(wait_status([&]{return status_.value("planning_wait_reason",string()).find("link")!=string::npos;},.18),
                  "the wait reason names the link check");
            pump(.25);
            check(moving_messages_==moves&&!active(),"one late command after a link loss does not restart the car");
            // Arrivals count, not the lease: one command valid for 0.5 s does not pass the 0.3 s wait (review 11-17).
            lone.header.stamp=fake_->now();lone.valid_for.nanosec=500000000;planning_pub_->publish(lone);pump(.45);
            check(moving_messages_==moves&&!active(),"one late command with a 0.5 s lease does not restart the car");
            pump(.15);
            const double from=steadySeconds();send_planning_=true;
            check(wait_status(active,1.0),"continuous commands restart the car");
            check(steadySeconds()-from>=.28,"the restart waits for about 0.3 s of continuous commands");
            // Link loss again, then planner STOPs during the wait (late and on time): neither clears the link check.
            send_planning_=false;
            check(wait_status(held,2.0)&&wait_status([&]{return status_.value("planning_link_pause",false);},.5),
                  "a second expiry sets the link check");
            // 2026-11-20: a planner STOP no longer clears the link check. A late one (stamped 0.5 s ago, sent during
            // the outage and delivered in the burst after it) and an on-time one (the newest of such a burst, or a
            // VEHICLE_STATE_STALE STOP sent just after the link returns) both keep it.
            const size_t back_mark=traceMark();
            nomad_interfaces::msg::DriveCommand late;late.header.frame_id="base_link";
            late.header.stamp=fake_->now()-rclcpp::Duration::from_seconds(.5);
            late.valid_for.nanosec=300000000;late.stop_requested=true;planning_pub_->publish(late);
            pump(.25);
            check(status_.value("planning_link_pause",false),"a late planner STOP keeps the link check");
            lpp(true,0.0);pump(.3);
            check(status_.value("planning_link_pause",false),"an on-time planner STOP after a link loss keeps it too");
            // A move that arrives late (0.45 s old, valid for 0.3 s) is refused and counted for [LPP_BACK].
            nomad_interfaces::msg::DriveCommand old_move;old_move.header.frame_id="base_link";
            old_move.header.stamp=fake_->now()-rclcpp::Duration::from_seconds(.45);old_move.valid_for.nanosec=300000000;
            old_move.target_speed=.25;old_move.target_steering_angle=.01;planning_pub_->publish(old_move);pump(.05);
            const double t1=steadySeconds();send_planning_=true;
            check(wait_status(active,1.0)&&steadySeconds()-t1>=.28,
                  "after a link loss and planner STOPs, fresh commands still wait about 0.3 s");
            const auto backs=traceSince(back_mark,"[LPP_BACK]");
            const string back=backs.empty()?string():backs.front();
            const bool back_ok=backs.size()==1&&traceNumber(back,"refused")==1&&traceNumber(back,"refused_age_max_ms")>=400&&
                traceNumber(back,"stops")==2&&traceNumber(back,"stop_age_max_ms")>=450&&
                traceNumber(back,"stop_valid_ms")==300&&traceNumber(back,"gap_ms")>=900&&traceNumber(back,"age_ms")>=0&&
                traceNumber(back,"first_ms")>=0&&traceNumber(back,"first_ms")<traceNumber(back,"gap_ms");
            check(back_ok,"one [LPP_BACK] line when commands are accepted again: the late move, the two STOPs, the gap");
            if(!back_ok)for(const auto& b:backs)cerr<<"  "<<b<<'\n';
        }else if(scenario=="slip display"){
            // Reached with the drive stopped by expiry (send_planning_ false above): restart and drive on.
            send_planning_=true;
            check(wait_status([&]{return status_.value("planning_supervisor_state",string())=="ACTIVE";},1.5),
                  "the drive restarts for the slip check");
            body_mps_=.27;pump(1.0);
            check(status_.value("slip_state",string())=="ok"&&abs(statusNumber("body_speed_mps",-1.0)-.27)<.05,
                  "slip display: ok while the body follows the wheels at 0.27 m/s");
            const int brakes=brake_messages_,handbrakes=handbrake_messages_;
            body_mps_=0.0;telemetry_iq_=30.0;
            check(wait_status([&]{return status_.value("slip_state",string())=="stuck";},2.5),
                  "wheels at 1000 ERPM and 30 A with the body still: slip display shows stuck");
            check(brake_messages_==brakes&&handbrake_messages_==handbrakes&&observed_motor_erpm_.back()==1000.0,
                  "stuck is display only: the drive output is unchanged");
            check(statusNumber("slip_events",0)>=1,"the stuck event is counted");
            body_mps_=.27;telemetry_iq_=6.0;
            check(wait_status([&]{return status_.value("slip_state",string())=="ok";},2.0),"moving again clears stuck");
            localization_ready_=false;
            check(wait_status([&]{return status_.value("slip_state",string())=="off"&&
                status_.value("slip_off_reason",string())=="localization not READY";},1.0),
                  "localization not READY: the display is off with the reason");
            localization_ready_=true;send_pose_=false;
            check(wait_status([&]{return status_.value("slip_off_reason",string())=="localization pose stale";},1.0),
                  "no fresh pose (Wi-Fi): the display is off");
        }else if(scenario=="shutdown"){
            // 2026-11-17 (5a): node shutdown (SIGINT/SIGTERM path) while the commission LPP stop holds.
            check(wait_status(held,2.0),"the LPP STOP ends in a hold before the shutdown");
            // Only what prepareShutdown() itself publishes: the controller leaves the executor (no more timer ticks).
            const size_t mark=output_log_.size();
            controller_->prepareShutdown();executor_.remove_node(controller_);pump(.05);
            int holds=0;bool clean=true;
            for(size_t i=mark;i<output_log_.size();++i){
                const auto& [time,kind,value]=output_log_[i];(void)time;
                if(kind=='H'){++holds;clean=clean&&value>=12.5-1e-9&&value<=30.0+1e-9;}else clean=false;
            }
            check(holds>=1&&clean,"shutdown while holding sends the handbrake (12.5-30 A) and nothing else");
        }else if(scenario=="shutdown while driving"){
            const size_t mark=output_log_.size();
            controller_->prepareShutdown();executor_.remove_node(controller_);pump(.05);
            int brakes=0;bool clean=true;
            for(size_t i=mark;i<output_log_.size();++i){
                const auto& [time,kind,value]=output_log_[i];(void)time;
                if(kind=='B'){++brakes;clean=clean&&value==20.0;}else if(kind=='S')clean=clean&&value<=0.0;else if(kind!='H')clean=false;
            }
            check(brakes>=1&&clean,"shutdown at 1000 ERPM sends the 20 A brake, never a drive command");
        }else throw runtime_error("unknown stop ramp scenario");
    }

    // Without a handbrake subscriber (unpatched vesc_driver) the node names it, does not claim a
    // hold it cannot deliver, and resumes once the subscriber is back.
    void runStopOutputDriverMissing() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.6);
        send_planning_=true;pump(.4);
        expect(moving_messages_>0,"driver missing: drive starts while the handbrake subscriber exists");
        const int brakes=brake_messages_;
        handbrake_sub_.reset();pump(.8);
        const int moves=moving_messages_;pump(.3);
        expect(moving_messages_==moves,"driver missing: no drive output without a handbrake subscriber");
        expect(status_.value("planning_wait_reason",string()).find("handbrake topic has no subscriber")!=string::npos,
               "driver missing: wait reason names the missing handbrake subscriber");
        expect(brake_messages_>brakes&&status_.value("stop_output",string())=="brake",
               "driver missing: stop output falls back to the brake instead of an undelivered hold");
        subscribeHandbrake();pump(1.0);
        expect(moving_messages_>moves&&!status_.value("planning_reset_required",true),
               "driver missing: drive resumes without RESET once the handbrake subscriber is back");
    }

    void runHeadless() {
        pump(.6);expect(moving_messages_==0,"headless waits for planner");
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        pump(.8);expect(moving_messages_>0,"headless starts without GUI");
        expect(!commission(1,3,string(32,'c'),1,0,.5,0),"manual cannot steal headless output");
        const int first=moving_messages_;pump(2.2);
        expect(moving_messages_>first&&status_.value("autonomous_active",false),"headless continues beyond GUI heartbeat");
        send_planning_=false;pump(.45);const int stopped=moving_messages_;
        send_planning_=true;pump(.3);
        expect(moving_messages_==stopped&&status_.value("planning_reset_required",false),"headless expiry latches stop");
        expect(call("reset"),"headless reset re-arms without GUI");pump(.4);
        expect(moving_messages_>stopped,"fresh planning resumes after reset");
        expect(call("stop"),"global STOP works in headless");const int stopped2=moving_messages_;pump(.4);
        expect(moving_messages_==stopped2,"continuing planner cannot undo global STOP");
        expect(call("reset"),"reset before planner STOP");pump(.4);
        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.stop_requested=true;
        planning_pub_->publish(stop);pump(.15);const int stopped3=moving_messages_;
        send_planning_=true;pump(.3);
        expect(moving_messages_==stopped3&&status_.value("planning_reset_required",false),"LPP stop_requested latches headless stop");
        expect(call("reset"),"reset before session STOP");pump(.4);
        expect(commission(3,0,string(32,'d')),"session STOP accepted without GUI owner");
        const int stopped4=moving_messages_;pump(.3);
        expect(moving_messages_==stopped4,"session STOP halts headless output");
        expect(!status_.value("planning_calibration_verified",true),"estimated execution does not fake calibration verification");
    }
    void runPlanning() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        int vehicle_states=0;
        auto vehicle=fake_->create_subscription<nomad_interfaces::msg::VehicleState>(prefix_+"/vehicle_state",10,
            [&vehicle_states](nomad_interfaces::msg::VehicleState::ConstSharedPtr msg){
                ++vehicle_states;expect(!msg->steering_valid,"servo command never presented as measured angle");});
        pump(.8);
        {
            // 2026-11-17: vehicle_state every control tick (~50 Hz; was 10 Hz).
            const int before=vehicle_states;pump(.5);
            expect(vehicle_states-before>=15,"vehicle_state is published at the control rate, not 10 Hz");
            pump(.6);
            expect(statusNumber("link_commands",0)>=10&&statusNumber("link_cmd_age_max_ms",-1e9)>-1e9&&
                   statusNumber("link_cmd_max_gap_ms",1e9)<500.0,"status reports received LPP commands and their stamp age");
        }
        const string id(32,'a'),other(32,'b');
        expect(moving_messages_==0,"planner reception alone cannot actuate");
        expect(commission(1,4,id,1,0,.5,0),"explicit LPP session starts");
        pump(.15);expect(moving_messages_>0,"LPP converted ERPM reaches fake VESC");
        expect(!commission(1,3,other,1,0,.5,0),"manual cannot steal autonomous owner");
        expect(commission(2,4,id,2,0,.5,0),"operator LPP heartbeat accepted");
        send_planning_=false;pump(.45);
        expect(status_.value("state","")!="RUNNING"&&!status_.value("autonomous_active",true),"expired LPP stops despite GUI heartbeat");
        {
            // The expiry is counted in the window it happened in; that window closes within about a second.
            bool counted=statusNumber("link_expiries",0)>=1;
            for(int i=0;i<30&&!counted;++i){pump(.05);counted=statusNumber("link_expiries",0)>=1;}
            expect(counted,"an LPP expiry while driving is counted in the link window");
        }
        send_planning_=true;const int before=moving_messages_;pump(.2);
        expect(moving_messages_==before,"planner reconnect never restarts output");
        expect(call("reset"),"stationary reset after LPP expiry");
        expect(commission(1,3,other,1,0,.5,0),"manual available again after STOP/reset");
        expect(commission(3,0,other),"manual STOP after autonomous session");
    }
#endif
    void runManualBelowRunningFloor() {
        const string active(32,'f');
        pump(.70);
        expect(commission(1,3,active,1,0.0,.5,0.0),"independent manual session starts at zero motor target");
        const auto first=observed_motor_erpm_.size();
        expect(commission(2,3,active,2,200.0,.5,0.0),"manual session accepts requested 200 ERPM");
        pump(.23);
        bool bounded=observed_motor_erpm_.size()>first;
        for(auto i=first;i<observed_motor_erpm_.size();++i)
            bounded=bounded&&observed_motor_erpm_[i]>=0.0&&observed_motor_erpm_[i]<=200.000001;
        expect(bounded&&abs(observed_motor_erpm_.back()-200.0)<1e-6&&
               abs(status_.value("target_motor_erpm",0.0)-200.0)<1e-6,
               "manual 200 ERPM output preserves its ramp and is never raised to the autonomy floor");
        expect(commission(3,0,active),"manual floor-isolation fixture stops normally");
    }

    // 2026-11-15: ESC over-temperature ends a manual GUI session (autonomy already paused) and refuses START until
    // below 70 C.
    void runManualThermalStop() {
        const string first(32,'b'), second(32,'c'), third(32,'e');
        telemetry_temp_c_=40.0;
        pump(.70);
        expect(status_.value("esc_temp_state",string())=="ok","fixture reports the ESC temperature");
        expect(commission(1,3,first,1,0.0,.5,0.0),"thermal fixture: manual session starts");
        expect(commission(2,3,first,2,300.0,.5,0.0),"manual drive accepted");
        pump(.10);
        expect(commission(2,3,first,3,300.0,.5,0.0),"manual heartbeat");
        telemetry_temp_c_=85.0;
        bool stopped=false;
        for(int i=0;i<15&&!stopped;++i){pump(.02);stopped=status_.value("state",string())!="RUNNING";}
        expect(stopped&&status_.value("reason",string()).find("ESC over-temperature")!=string::npos,
               "ESC >= 80 C ends the manual session with its reason (before the GUI heartbeat timeout)");
        const int moves=moving_messages_;
        pump(.10);
        expect(moving_messages_==moves,"no drive after the over-temperature stop");
        expect(call("reset"),"reset after the over-temperature stop");
        expect(!commission(1,3,second,1,0.0,.5,0.0),"START refused while the ESC is still hot");
        telemetry_temp_c_=75.0;
        pump(.10);
        expect(!commission(1,3,second,2,0.0,.5,0.0),"START still refused between 70 and 80 C");
        telemetry_temp_c_=65.0;
        pump(.10);
        expect(commission(1,3,third,1,0.0,.5,0.0),"START allowed again below 70 C");
        expect(commission(3,0,third),"thermal fixture stops normally");
    }

    // 2026-11-15 GUI MANUAL reverse through the ROS commissioning service: the negative target is ramped and
    // published, the status reports the limit, and a forward target while the wheel still turns backward waits on
    // the stop output (manual_reversal_wait) instead of driving through zero.
    void runManualReverse() {
        const string active(32,'d'); // session ids are 32 hex digits
        pump(.70);
        expect(status_.value("manual_reverse_max_erpm",-1.0)==600.0&&!status_.value("manual_reversal_wait",true),
               "status reports the manual reverse limit and no reversal wait");
        expect(commission(1,3,active,1,0.0,.5,0.0),"reverse fixture: manual session starts at zero");
        const auto first=observed_motor_erpm_.size();
        expect(commission(2,3,active,2,-300.0,.5,0.0),"GUI MANUAL accepts a reverse target within the limit");
        pump(.20);
        expect(commission(2,3,active,3,-300.0,.5,0.0),"reverse heartbeat");
        bool bounded=observed_motor_erpm_.size()>first;
        double lowest=0.0;
        for(auto i=first;i<observed_motor_erpm_.size();++i){
            bounded=bounded&&observed_motor_erpm_[i]<=0.0&&observed_motor_erpm_[i]>=-300.000001;
            lowest=min(lowest,observed_motor_erpm_[i]);
        }
        expect(bounded&&lowest<=-200.0,"reverse speed is ramped toward the target and published negative");
        telemetry_erpm_=-250.0;
        pump(.08);
        const auto reversal=observed_motor_erpm_.size();
        const int stops=brake_messages_+release_messages_+handbrake_messages_;
        expect(commission(2,3,active,4,300.0,.5,0.0),"forward target accepted while rolling backward");
        bool saw_wait=false;
        for(int i=0;i<10&&!saw_wait;++i){pump(.02);saw_wait=status_.value("manual_reversal_wait",false);}
        bool no_forward=true;
        for(auto i=reversal;i<observed_motor_erpm_.size();++i) no_forward=no_forward&&observed_motor_erpm_[i]<=0.0;
        expect(saw_wait&&no_forward&&brake_messages_+release_messages_+handbrake_messages_>stops,
               "operator reversal while the wheel turns backward waits on the stop output, no forward drive");
        telemetry_erpm_=0.0;
        pump(.04);
        expect(commission(2,3,active,5,300.0,.5,0.0),"forward heartbeat once the wheel is still");
        bool forward=false;
        for(int i=0;i<10&&!forward;++i){
            pump(.02);
            forward=!observed_motor_erpm_.empty()&&observed_motor_erpm_.back()>0.0;
        }
        expect(forward,"the forward drive starts from 0 once the wheel is still");
        expect(commission(3,0,active),"reverse fixture stops normally");
    }

    void runCommission() {
        const string a(32, '1'), b(32, '2'), c(32, '3'), d(32, '4');
        pump(0.7);
        expect(status_.value("control_units", "") == "raw" && status_.value("commissioning", false), "raw status labels device units");
        expect(status_.at("speed_mps").is_null() && !status_.value("calibration_verified", true), "raw status never invents calibrated speed");
        // 2026-11-16 VESC display values for the team GUI (fake VESC: 12 V, input 1.5 A).
        expect(status_.value("input_current_a", 0.0) == 1.5 && status_.value("input_power_w", 0.0) == 18.0 &&
               status_.value("duty", 0.0) == 0.05 && status_.value("id_a", 0.0) == 0.25 &&
               status_.value("vd_v", 0.0) == -0.5 && status_.value("vq_v", 0.0) == 1.5 &&
               status_.at("iq_a").is_number(), "raw status reports VESC input current, power, duty and d/q values");
        expect(status_.value("charge_drawn_ah", 0.0) == 0.125 && status_.value("charge_regen_ah", 0.0) == 0.0625 &&
               status_.value("energy_drawn_wh", 0.0) == 2.5 && status_.value("energy_regen_wh", 0.0) == 1.25 &&
               status_.value("tacho", 0) == 1234 && status_.value("tacho_abs", 0) == 4321 &&
               status_.value("vesc_controller_id", 0) == 111, "raw status reports VESC counters and id");
        expect(status_.at("esc_mos_temps_c").is_array() && status_.at("esc_mos_temps_c").size() == 3 &&
               status_.value("fault_name", string()) == "NONE", "raw status reports per-MOSFET temperatures and the fault name");
        {
            const double age = status_.value("telemetry_age_s", -1.0);
            expect(age >= 0.0 && age < 0.2, "raw status reports a fresh telemetry age");
        }
#ifdef NOMAD_ENABLE_PLANNING
        expect(status_.contains("estimated_speed_mps") && !status_.value("estimated_speed_verified", true),
               "raw status labels the ERPM speed estimate as unverified");
#endif
        // 2026-11-17 link/loop window and long-hold estimate (closed once a second; no planner in this fixture).
        pump(0.5);
        for (int i = 0; i < 30 && statusNumber("control_tick_max_ms", -1.0) < 0.0; ++i) pump(0.05);
        expect(status_.at("control_tick_max_ms").is_number() && status_.value("control_tick_max_ms", 0.0) > 0.0 &&
               status_.value("link_commands", -1) == 0 && status_.value("link_cmd_max_gap_ms", -1.0) == 0.0 &&
               status_.at("link_cmd_age_max_ms").is_null() && status_.value("link_expiries", -1) == 0,
               "status reports the link/loop window (no planner: no commands, no gap, no age)");
        expect(status_.value("stop_hold_heat_w", -1.0) == 0.0 && !status_.value("stop_hold_long", true),
               "no hold: no estimated hold heat, no long-hold flag");
        // A malformed wire stamp (sec < 0) used to throw inside the service callback and end the controller.
        expect(!steeringRange(-5, 0), "steering range with a negative stamp is refused");
        expect(!steeringRange(1, 1000000000u), "steering range with nanosec >= 1e9 is refused");
        {
            status_ = nomad::Json::object();
            pump(0.25);
            expect(status_.value("commissioning", false), "controller keeps running after malformed stamps");
        }
        expect(!call("start_test") && !manual(1, a), "commission rejects both calibrated start paths");
        expect(commission(3, 0, a), "raw STOP with no axis/goals accepted");
        expect(!commission(1, 1, a), "raw delayed START after STOP rejected");
        expect(!commission(1, 1, b, 1, 300, 0.5, 1, 0.5), "raw stale START rejected");
        expect(commission(1, 1, b), "raw motor starts through ROS");
        for (uint64_t seq = 2; seq <= 5; ++seq) {
            pump(0.08);
            expect(commission(2, 1, b, seq), "raw motor heartbeat accepted");
        }
        expect(moving_messages_ > 0 && servo_messages_ == 0, "raw motor trial publishes no servo commands");
        expect(!commission(2, 1, b, 5), "raw duplicate sequence rejected");
        expect(!commission(2, 2, b, 6, 0, 0.51), "raw axis change rejected");
        pump(0.1);
        expect(call("reset"), "raw axis fault reset");
        const int moves = moving_messages_;
        expect(commission(1, 2, c, 1, 0, 0.51), "raw steering starts through ROS");
        pump(0.15);
        expect(commission(2, 2, c, 2, 0, 0.49), "raw steering heartbeat accepted");
        pump(0.1);
        expect(servo_messages_ > 0 && moving_messages_ == moves, "raw steering never drives motor");
        expect(commission(3, 0, c), "raw STOP accepts omitted targets while active");
        expect(call("reset"), "raw stop reset");
        expect(!commission(1, 2, c, 1, 0, 0.50), "closed raw session cannot restart after reset");
        expect(!commission(1, 2, d, 1, 0, 0.52), "raw unsafe servo target refused");
    }

    void runCommissionGuiBridge() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge); pump(0.8);
        auto invoke = [&](const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command("commissioning", data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        const string id(32, '5');
        auto request = nomad::Json{{"action", 1}, {"axis", 2}, {"session", id}, {"sequence", 1},
            {"motor_erpm", 0.0}, {"servo_position", 0.50}, {"duration_s", 1.0}, {"server_time_s", nomad::unixSeconds()}};
        expect(invoke(request), "GUI bridge raw START reaches commissioning service");
        pump(0.15);
        expect(servo_messages_ > 0 && moving_messages_ == 0, "GUI raw steering only drives servo");
        expect(invoke(nomad::Json{{"action", 3}, {"session", id}, {"sequence", 2}}), "GUI raw STOP needs no axis or targets");
        pump(0.3);
        expect(bridge->status().value("state", "") != "RUNNING", "GUI bridge observes raw stop");
        expect(call("reset"), "GUI continuous preparation resets stopped state");
        const string manual_id(32, '8');
        request["axis"] = 3; request["session"] = manual_id; request["duration_s"] = 0.0;
        request["server_time_s"] = nomad::unixSeconds();
        expect(invoke(request), "GUI bridge accepts continuous manual axis and explicit zero duration");
        request["action"] = 2; request["sequence"] = 2; request["motor_erpm"] = 200.0;
        request["servo_position"] = 0.51; request["server_time_s"] = nomad::unixSeconds();
        expect(invoke(request), "GUI bridge forwards both continuous slider targets");
        pump(0.15);
        expect(moving_messages_ > 0 && servo_messages_ > 0, "GUI continuous update drives both fake outputs");
        expect(invoke(nomad::Json{{"action", 3}, {"session", manual_id}, {"sequence", 3}}),
               "GUI continuous STOP accepts targetless payload");
        executor_.remove_node(bridge);
    }

    void runContinuousCommission() {
        const string rejected(32, '9'), active(32, 'a'), next(32, 'b');
        pump(0.7);
        expect(status_.value("continuous_manual_supported", false) &&
               !status_.value("continuous_manual_active", true) &&
               status_.at("commanded_servo_position").is_number(), "raw IDLE advertises continuous contract and candidate command");
        expect(!commission(1, 3, rejected, 1, 1, 0.5, 0), "continuous ROS START rejects nonzero motor");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "continuous ROS START at zero motor accepted");
        for (uint64_t sequence = 2; sequence <= 44; ++sequence) {
            pump(0.08);
            expect(commission(2, 3, active, sequence, 300, sequence % 2 ? 0.49 : 0.51, 0),
                   "continuous ROS heartbeat with two slider goals accepted");
        }
        expect(status_.value("state", "") == "RUNNING" && status_.value("continuous_manual_active", false) &&
               status_.value("remaining_s", -1.0) == 0.0,
               "continuous ROS control exceeds trial deadline while heartbeat remains fresh");
        expect(moving_messages_ > 0 && servo_messages_ > 0, "continuous ROS emits both command channels");
        expect(!commission(2, 3, active, 44, 0, 0.5, 0), "continuous duplicate sequence rejected");
        expect(!commission(2, 3, active, 45, 0, 0.5, 0, 0.5), "continuous stale UPDATE rejected");
        pump(0.4);
        expect(!commission(2, 3, active, 46, 300, 0.51, 0), "continuous heartbeat timeout cannot be revived");
        const int moves = moving_messages_, servos = servo_messages_;
        pump(0.15);
        expect(moving_messages_ == moves && servo_messages_ == servos &&
               status_.value("state", "") == "FAULT" && !status_.value("continuous_manual_active", true),
               "continuous timeout latches and stops both command channels");
        expect(call("reset"), "continuous fault allows explicit stationary reset");
        expect(!commission(1, 3, active, 47, 0, 0.5, 0), "closed continuous session stays closed after reset");
        expect(commission(3, 0, next), "continuous preemptive STOP accepted");
        expect(!commission(1, 3, next, 1, 0, 0.5, 0), "delayed continuous START cannot pass earlier STOP");
    }

    void runRawMotorOptimization() {
        const string active(32, '1'), blocked(32, '2'), resumed(32, '3');
        pump(0.7);
        expect(status_.value("motor_control_mode", "") == "vesc_speed_pid" &&
               status_.value("speed_pid_min_erpm", -1.0) == 50.0 &&
               !status_.value("speed_pid_min_erpm_verified", true),
               "PID minimum is exposed as unverified metadata until external readback confirmation");
        expect(status_.value("steering_input_limit_rad", 0.0) == 0.5 &&
               status_.value("control_tick_hz", 0) == 50 &&
               status_.value("motor_deceleration_erpm_s", 0.0) == 2400.0,
               "optimized command rates and candidate input range are advertised");
        expect(status_.value("observed_smooth_start_erpm", -1.0) == 0.0,
               "unconfigured operator observation is explicitly unset");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "optimized raw session starts explicitly");
        expect(commission(2, 3, active, 2, 600, 0.5, 0), "positive raw speed reaches internal VESC PID channel");
        pump(0.15);
        expect(moving_messages_ > 0, "positive command produces speed messages");
        expect(commission(2, 3, active, 3, 0, 0.5, 0), "zero target accepted without ending steering session");
        pump(0.08);
        const int moves = moving_messages_;
        pump(0.12);
        expect(moving_messages_ == moves && status_.value("commanded_motor_erpm", -1.0) == 0.0 &&
               status_.value("target_motor_erpm", -1.0) == 0.0,
               "zero target immediately stops speed messages rather than ramping through low RPM PID");
        expect(commission(3, 0, active), "optimized session STOP accepted");
        expect(call("reset"), "optimized zero session may reset when stationary");
        auto competing = fake_->create_publisher<nomad::Scalar>(prefix_ + "/commands/motor/speed", 1);
        const double deadline = steadySeconds() + 1.0;
        while (controller_->count_publishers(prefix_ + "/commands/motor/speed") < 2 && steadySeconds() < deadline)
            pump(0.005);
        expect(!commission(1, 3, blocked, 1, 0, 0.5, 0), "START refreshes graph and rejects a competing speed publisher");
        competing.reset(); pump(0.25);
        expect(commission(1, 3, resumed, 1, 0, 0.5, 0), "new session starts after competing publisher leaves");
        competing = fake_->create_publisher<nomad::Scalar>(prefix_ + "/commands/motor/speed", 1);
        pump(0.22);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("competing") != string::npos,
               "cached graph is refreshed within bounded time and faults an active competing publisher");
    }

    // Wi-Fi 유예는 발행 시각 기준이며 로컬 telemetry/timer 감시와 짧은 축 시험은 그대로입니다.
    void runWifiManualProfile() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge); pump(0.8);
        expect(status_.value("wifi_manual_profile", false) && status_.value("manual_timeout_s", 0.0) == 2.0 &&
               status_.value("command_max_age_s", 0.0) == 1.5 && status_.value("browser_freshness_s", 0.0) == 1.5 &&
               status_.value("browser_request_timeout_ms", 0) == 1500,
               "Wi-Fi commissioning advertises bounded command and browser timing policy");
        auto invoke = [&](const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command("commissioning", data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        const string active(32, 'a');
        auto request = nomad::Json{{"action", 1}, {"axis", 3}, {"session", active}, {"sequence", 1},
            {"motor_erpm", 0.0}, {"servo_position", 0.5}, {"duration_s", 0.0}, {"server_time_s", nomad::unixSeconds()}};
        expect(invoke(request), "Wi-Fi MANUAL starts explicitly at zero through GUI bridge");
        request["action"] = 2; request["sequence"] = 2; request["server_time_s"] = nomad::unixSeconds() - 1.6;
        expect(!invoke(request), "Wi-Fi GUI bridge rejects command older than 1.5 seconds");
        expect(!commission(2, 3, active, 2, 0, 0.5, 0, 1.6), "Wi-Fi ROS service independently rejects over-age command");
        pump(1.35);
        request["sequence"] = 3; request["server_time_s"] = nomad::unixSeconds() - 1.2;
        expect(invoke(request), "Wi-Fi GUI and ROS accept 1.2-second-old MANUAL update with fresh local telemetry");
        pump(0.45);
        expect(status_.value("state", "") == "RUNNING", "Wi-Fi issued-command lease tolerates bounded delivery delay");
        pump(0.65);
        expect(status_.value("state", "") == "FAULT" && status_.value("telemetry_fresh", false) &&
               status_.value("reason", "").find("GUI command timeout") != string::npos,
               "delayed update expires from original issue time instead of adding two seconds after receipt");
        expect(!commission(2, 3, active, 4, 0, 0.5, 0), "expired Wi-Fi session cannot be revived by fresh update");
        expect(call("reset") && !commission(1, 3, active, 5, 0, 0.5, 0), "Wi-Fi reset cannot reopen expired session ID");

        for (const int axis : {1, 2}) {
            const string id(32, axis == 1 ? 'b' : 'c');
            request["action"] = 1; request["axis"] = axis; request["session"] = id;
            request["sequence"] = 1; request["duration_s"] = 1.0;
            request["server_time_s"] = nomad::unixSeconds() - 1.2;
            expect(!invoke(request), "Wi-Fi profile does not extend short-axis GUI command age");
            expect(!commission(1, axis, id, 1, 0, 0.5, 1.0, 1.2), "Wi-Fi profile keeps short-axis ROS age strict");
            request["server_time_s"] = nomad::unixSeconds();
            expect(invoke(request), "short-axis trial accepts a fresh command under Wi-Fi profile");
            pump(0.5);
            expect(status_.value("state", "") == "FAULT" &&
                   status_.value("reason", "").find("GUI command timeout") != string::npos,
                   "short-axis trial retains 350ms heartbeat lease in Wi-Fi profile");
            expect(call("reset"), "short-axis Wi-Fi fixture resets only explicitly");
        }
        const string telemetry(32, 'd'), timer(32, 'e');
        expect(commission(1, 3, telemetry, 1, 0, 0.5, 0), "Wi-Fi telemetry guard fixture starts");
        send_telemetry_ = false; pump(0.45);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("telemetry") != string::npos,
               "Wi-Fi grace does not extend the local 300ms telemetry guard");
        send_telemetry_ = true; pump(0.4);
        expect(call("reset") && commission(1, 3, timer, 1, 0, 0.5, 0), "Wi-Fi timer guard fixture starts explicitly");
        this_thread::sleep_for(chrono::milliseconds(250));
        pump(0.15);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("control timer stalled") != string::npos,
               "Wi-Fi grace does not extend the local 200ms timer guard");
        expect(moving_messages_ == 0, "Wi-Fi timing integration tests never request positive motor output");
        executor_.remove_node(bridge);
    }

    void runRawStallWatchdog() {
        const string active(32, '4');
        pump(0.7);
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "stall watchdog fixture starts at zero");
        for (uint64_t sequence = 2; sequence < 11; ++sequence) {
            commission(2, 3, active, sequence, 200, 0.5, 0);
            pump(0.1);
        }
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("stalled") != string::npos,
               "ROS controller detects commanded speed without rotation while heartbeat is fresh");
        const int moves = moving_messages_, servos = servo_messages_;
        pump(0.15);
        expect(moving_messages_ == moves && servo_messages_ == servos && release_messages_ > 0,
               "ROS stall stop emits only motor release and no further servo commands");
    }

    void runExpandedSteeringContract() {
        const string active(32, '5');
        pump(0.7);
        expect(status_.value("steering_input_limit_rad", 0.0) == 0.55 &&
               status_.value("servo_min", 0.0) == 0.3625 && status_.value("servo_max", 0.0) == 0.6375,
               "configured next candidate angle and servo envelope are advertised together");
        expect(status_.value("observed_smooth_start_erpm", 0.0) == 550.0 &&
               status_.value("observed_smooth_start_note", "") == "공중 시험 · 사용자 관찰 · 실제 주행 최저속도 미보정",
               "operator startup observation carries its uncalibrated context");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "expanded candidate session still starts at zero motor");
        expect(commission(2, 3, active, 2, 200, 0.6375, 0), "target below observed startup threshold is accepted unchanged");
        pump(0.15);
        expect(status_.value("target_motor_erpm", -1.0) == 200.0 &&
               status_.value("commanded_motor_erpm", -1.0) > 0.0 &&
               status_.value("commanded_motor_erpm", -1.0) <= 200.0,
               "observed 550 ERPM metadata never raises the 200 ERPM control target or output");
        uint64_t sequence = 2;
        for (const double target : {0.6375, 0.3625}) {
            bool accepted = true;
            for (int step = 0; step < 16; ++step) {
                accepted = commission(2, 3, active, ++sequence, 0, target, 0) && accepted;
                pump(0.05);
            }
            expect(accepted && abs(status_.value("commanded_servo_position", 0.0) - target) < 1e-9 &&
                   status_.value("commanded_motor_erpm", -1.0) == 0.0,
                   "both configured candidate endpoints are reachable with motor released");
        }
        expect(!commission(2, 3, active, ++sequence, 0, 0.6376, 0),
               "ROS service refuses a target just outside the new candidate envelope");
        pump(0.15);
        expect(status_.value("state", "") == "FAULT" && status_.value("commanded_motor_erpm", -1.0) == 0.0,
               "out-of-range expanded steering request latches fault and motor release");
    }

    void runManual() {
        const string a(32, 'a'), b(32, 'b'), c(32, 'c');
        pump(0.7);
        expect(manual(3, a), "STOP before START accepted");
        expect(!manual(1, a), "delayed START after STOP refused");
        expect(!manual(1, b, 1, 0.5), "stale command refused");
        expect(manual(1, b), "manual ROS start succeeds");
        expect(manual(2, b, 2), "manual ROS update succeeds");
        expect(!manual(2, b, 2), "duplicate sequence refused");
        expect(!manual(2, c, 3), "wrong session refused");
        pump(0.5);
        expect(!manual(2, b, 3), "expired GUI lease cannot revive");
        const int moves = moving_messages_;
        pump(0.1);
        expect(moves == moving_messages_, "GUI timeout stops outputs while telemetry remains fresh");
        expect(call("reset"), "manual fault reset");
        expect(!manual(1, b), "closed session refused after reset");
        expect(manual(1, c), "new session accepted after reset");
        expect(manual(3, c), "manual release stop");
        expect(call("reset"), "manual stop reset");
        expect(!manual(1, c), "stopped session cannot start again");
    }

    void runGuiBridge() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge);
        pump(0.8);
        expect(bridge->status().value("connected", false), "GUI bridge receives controller status");
        const string id(32, 'e');
        auto request = nomad::Json{{"action", 1}, {"session", id}, {"sequence", 1},
            {"speed_mps", 0.1}, {"steering_rad", 0.0}, {"duration_s", 1.0}, {"server_time_s", nomad::unixSeconds()}};
        auto invoke = [&](const string& route, const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command(route, data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        expect(invoke("manual", request), "GUI bridge starts via real ROS service");
        pump(0.15);
        request["action"] = 2; request["sequence"] = 2; request["server_time_s"] = nomad::unixSeconds();
        expect(invoke("manual", request), "GUI bridge forwards heartbeat");
        request["server_time_s"] = nomad::unixSeconds() - 0.5;
        expect(!invoke("manual", request), "GUI bridge rejects delayed HTTP command");
        expect(invoke("stop", nomad::Json::object()), "GUI global stop reaches controller");
        pump(0.3);
        expect(bridge->status().value("state", "") != "RUNNING", "GUI bridge observes stopped state");
        expect(invoke("reset", nomad::Json::object()), "GUI reset reaches controller");
        executor_.remove_node(bridge);
    }

    void runMonitor() {
        pump(1.0);
        expect(fake_->count_publishers(prefix_ + "/commands/motor/speed") == 0, "monitor creates no motor publisher");
        expect(fake_->count_publishers(prefix_ + "/commands/servo/position") == 0, "monitor creates no servo publisher");
        expect(!call("start_test"), "monitor start refused");
        expect(!manual(1, string(32, 'd')), "monitor manual start refused");
        expect(!commission(1, 1, string(32, '6')), "monitor raw start refused");
        expect(call("stop"), "monitor stop service callable");
        pump(0.1);
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "monitor emits no actuator data");
    }

    // 2026-11-12: stop_brake_current_a > 0 -> releases are brake current, never 0A coast.
    void runBenchBrake(double amps) {
        pump(1.0);
        expect(brake_messages_ == 0 && release_messages_ == 0, "brake bench boot sends nothing");
        expect(call("start_test"), "brake bench start succeeds (own brake publisher is not a competitor)");
        pump(1.2);
        expect(moving_messages_ > 0, "brake bench drives");
        const int brakes = brake_messages_;
        pump(0.2);
        expect(brake_messages_ > brakes && last_brake_ == amps, "deadline release is brake current");
        pump(0.2);
        expect(brake_messages_ > brakes + 3, "brake is repeated while stopped (VESC command timeout)");
        expect(release_messages_ == 0, "no 0A coast when stop braking is enabled");
        expect(call("reset") && call("start_test"), "brake bench reset then start");
        pump(0.3);
        const int before_stop = brake_messages_;
        expect(call("stop"), "brake bench stop");
        pump(0.1);
        expect(brake_messages_ > before_stop && last_brake_ == amps && release_messages_ == 0, "operator stop brakes");
    }

    // 2026-11-12: standstill (fake telemetry 0 ERPM) -> handbrake hold, not brake current or 0A.
    void runBenchHandbrake(double hold) {
        pump(1.0);
        expect(handbrake_messages_ == 0, "handbrake bench boot sends nothing");
        expect(call("start_test"), "handbrake bench start succeeds (own handbrake publisher is not a competitor)");
        pump(1.2);
        expect(moving_messages_ > 0, "handbrake bench drives");
        const int holds = handbrake_messages_;
        pump(0.3);
        expect(handbrake_messages_ > holds + 3 && last_handbrake_ == hold, "stopped at standstill repeats handbrake hold");
        expect(release_messages_ == 0 && brake_messages_ == 0, "standstill release is handbrake only");
        const int moves_before_restart = moving_messages_;
        expect(call("reset") && call("start_test"), "handbrake bench reset then start");
        pump(0.3);
        expect(moving_messages_ > moves_before_restart, "handbrake bench drives after hold");
        const int before_stop = handbrake_messages_;
        expect(call("stop"), "handbrake bench stop");
        pump(0.1);
        expect(handbrake_messages_ > before_stop && last_handbrake_ == hold && release_messages_ == 0 && brake_messages_ == 0,
               "operator stop at standstill holds with the handbrake");
    }

    // A VESC fault while a stopped bench trial holds: 0A, latched until RESET, then hold again.
    void runBenchHandbrakeFault(double hold) {
        pump(1.0);
        expect(call("start_test"),"bench fault: start");
        pump(1.4);
        expect(status_.value("stop_output",string())=="hold"&&last_handbrake_==hold,"bench fault: stopped trial holds");
        const int releases=release_messages_;
        telemetry_fault_=7;pump(.3);
        expect(release_messages_>releases&&status_.value("vesc_fault",false)&&status_.value("state",string())=="FAULT",
               "bench fault: VESC fault on a stopped trial latches and releases with 0A");
        const int holds=handbrake_messages_;pump(.2);
        expect(handbrake_messages_==holds,"bench fault: no hold to a faulted VESC");
        telemetry_fault_=0;pump(.5);
        expect(call("reset"),"bench fault: RESET after the fault clears");
        pump(.3);
        expect(handbrake_messages_>holds&&!status_.value("vesc_fault",true),"bench fault: hold returns after RESET");
    }

    void runBench() {
        pump(1.0);
        expect(!commission(1, 1, string(32, '7')), "calibrated bench rejects raw start service");
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "bench boot never auto-starts");
        expect(call("start_test"), "explicit bench start succeeds");
        expect(!call("start_test"), "duplicate start rejected");
        pump(1.2);
        expect(moving_messages_ > 0 && servo_messages_ > 0, "fake driver receives trial output");
        const int moves_at_stop = moving_messages_;
        const int releases_at_stop = release_messages_;
        pump(0.2);
        expect(moving_messages_ == moves_at_stop && release_messages_ > releases_at_stop, "deadline releases and stays stopped");
        expect(!call("start_test"), "deadline requires explicit reset");
        expect(call("reset") && call("start_test"), "reset then start");
        pump(0.3);
        expect(call("stop"), "stop service succeeds");
        pump(0.1);
        const int stopped_moves = moving_messages_;
        pump(0.2);
        expect(moving_messages_ == stopped_moves, "service stop stays stopped");
        expect(call("reset") && call("start_test"), "start telemetry timeout trial");
        pump(0.2);
        send_telemetry_ = false;
        pump(0.5);
        const int timeout_moves = moving_messages_;
        pump(0.2);
        expect(moving_messages_ == timeout_moves, "telemetry loss stops outputs");
        expect(!call("reset"), "stale telemetry cannot reset");
    }

    void runSteeringRecords() {
        const auto directory=filesystem::temp_directory_path()/prefix_.substr(1);
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller",prefix_+"/nomad_vesc_trial"},{"steering_records_directory",directory.string()}});
        auto bridge=make_shared<nomad::GuiBridge>(options);executor_.add_node(bridge);pump(.8);
        auto setRange=[&](double span){
            const nomad::Json data={{"span",span},{"server_time_s",nomad::unixSeconds()}};
            auto future=async(launch::async,[&]{return bridge->command("steering-range",data);});
            while(future.wait_for(chrono::milliseconds(0))!=future_status::ready)pump(.005);
            return future.get().value("accepted",false);
        };
        const auto servo_before=servo_messages_;
        expect(setRange(.3),"GUI range service accepts stationary steering-only limit");pump(.15);
        expect(servo_messages_==servo_before&&moving_messages_==0,"range change publishes neither steering nor motor movement");
        expect(bridge->status().at("steering_test_servo_max")==.8&&bridge->status().at("servo_max")!=.8,"test range is separate from drive range");
        expect(!setRange(.6),"GUI range request above protocol bounds rejected");
        nomad::Json row={{"operation","save"},{"id",string(32,'a')},{"servo_command",.5},
            {"left_deg",0.0},{"right_deg",0.0},{"note","integration test only"},{"measured_confirmed",true}};
        expect(bridge->command("steering-records",row).value("accepted",false),"stationary measurement stored through GUI bridge");
        auto invoke=[&](const nomad::Json& data){
            auto future=async(launch::async,[&]{return bridge->command("commissioning",data);});
            while(future.wait_for(chrono::milliseconds(0))!=future_status::ready)pump(.005);
            return future.get().value("accepted",false);
        };
        nomad::Json step={{"action",1},{"axis",2},{"session",string(32,'b')},{"sequence",1},
            {"motor_erpm",0},{"servo_position",.505},{"duration_s",bridge->status().at("max_duration_s")},{"server_time_s",nomad::unixSeconds()},
            {"record_steering_command",true}};
        expect(invoke(step),"calibration uses steering-only trial and automatic recording");pump(.12);
        expect(!setRange(.4),"range service rejects changes during steering motion");
        step["action"]=2;step["sequence"]=2;step["server_time_s"]=nomad::unixSeconds();
        expect(invoke(step),"recorded steering heartbeat accepted");
        auto automatic=bridge->command("steering-records",{{"operation","list"}});
        expect(automatic.at("records").size()==2,"unchanged heartbeat does not duplicate command record");
        for(const auto& record:automatic.at("records"))if(record.value("kind",string{})=="command")
            expect(record.at("left_deg").is_null()&&!record.at("measured_confirmed").get<bool>()&&record.at("servo_command")==.505,
                   "automatic record contains accepted servo target but no fabricated angle");
        expect(!bridge->command("steering-records",row).value("accepted",true),"cannot save while steering trial running");
        expect(call("stop"),"calibration explicit stop");pump(.25);
        row["id"]=string(32,'c');row["servo_command"]=.6;
        expect(!bridge->command("steering-records",row).value("accepted",true),"stale servo pairing refused");
        row["servo_command"]=bridge->status().at("commanded_servo_position");row["left_deg"]=1.2;row["right_deg"]=1.1;
        expect(bridge->command("steering-records",row).value("accepted",false),"settled command pairs with user measurement");
        auto note=row;note["id"]=string(32,'e');note["kind"]="observation";
        note.erase("left_deg");note.erase("right_deg");note.erase("measured_confirmed");
        auto noted=bridge->command("steering-records",note);
        expect(noted.value("accepted",false)&&noted.at("record").at("left_rad").is_null(),"memo needs no measured angle");
        auto listed=bridge->command("steering-records",{{"operation","list"}});
        expect(listed.at("records").size()==4&&!listed.value("applied",true),"command, observation and measurements remain distinct without applying calibration");
        // 0.6 s: telemetry timeout (0.3 s) + status period (0.1 s) + margin; 0.4 s failed once under load (11-15).
        send_telemetry_=false;pump(.6);row["id"]=string(32,'d');
        expect(!bridge->command("steering-records",row).value("accepted",true),"stale telemetry cannot save measurement");
        expect(moving_messages_==0,"calibration emitted no nonzero motor speed command");
        executor_.remove_node(bridge);
        if(directory.parent_path()==filesystem::temp_directory_path())filesystem::remove_all(directory);
    }

    void runStability() {
        pump(.8);
        expect(!call("start_test"),"stability rejects missing IMU before drive");
        expect(!manual(1,"77777777777777777777777777777777"),"GUI start also rejects missing IMU");
        expect(manual(3,"77777777777777777777777777777777",2),"GUI STOP remains available without IMU");
        expect(moving_messages_==0,"missing IMU produces no drive");
        imu_pub_=fake_->create_publisher<sensor_msgs::msg::Imu>(prefix_+"/imu_body",1);
        send_imu_=true; imu_frame_="wrong_frame"; pump(.3);
        expect(!call("start_test"),"wrong IMU frame rejected");
        imu_frame_="base_link";pump(.3);
        expect(call("start_test"),"qualified simulated IMU starts bench");
        // 2026-11-19: status_json comes every 0.10-0.12 s; wait for it instead of reading it once after a fixed time
        // (this check failed now and then on a busy PC).
        const auto stopped=[&](double seconds){
            const double deadline=steadySeconds()+seconds;
            while(status_.value("state","")!="STOPPED_OUTPUT"&&steadySeconds()<deadline)pump(.02);
            return status_.value("state","")=="STOPPED_OUTPUT";
        };
        pump(.2); send_imu_=false; pump(.3);
        expect(stopped(1.0),"stale IMU stops owned bench output");
        const auto moves=moving_messages_; send_imu_=true; pump(.3);
        expect(moving_messages_==moves,"IMU recovery cannot restart motion");
        expect(call("reset"),"explicit reset with stationary telemetry");
        expect(call("start_test"),"explicit restart after IMU recovery");
        imu_roll_=.35; pump(.15);
        expect(stopped(1.0),"excessive tilt stops bench output");
        expect(status_.value("ground_drive_supported",true)==false,"never claims ground support");
    }

private:
#ifdef NOMAD_ENABLE_PLANNING
    rclcpp::Publisher<nomad_interfaces::msg::DriveCommand>::SharedPtr planning_pub_;
    bool send_planning_=true,planning_without_telemetry_=false;
    double planner_speed_=.2;
#endif
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    bool send_imu_=false;
    string imu_frame_="base_link";
    double imu_roll_=0;
    string prefix_;
    rclcpp::Node::SharedPtr fake_;
    shared_ptr<nomad::ControllerNode> controller_;
    rclcpp::Publisher<nomad::VescState>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    nomad::Json status_ = nomad::Json::object();
    vector<rclcpp::Subscription<nomad::Scalar>::SharedPtr> subs_;
    rclcpp::Subscription<nomad::Scalar>::SharedPtr handbrake_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool send_telemetry_ = true;
    double last_sent_ = 0.0;
    double telemetry_erpm_ = 0.0;
    double telemetry_iq_ = 0.0; // VESC avg_iq (load-sized hold)
    double telemetry_temp_c_ = 0.0; // ESC MOSFET temperature (0 = no reading, thermal guard 'unknown')
    int telemetry_fault_ = 0;
    int speed_messages_ = 0, moving_messages_ = 0, servo_messages_ = 0, release_messages_ = 0;
    int brake_messages_ = 0, handbrake_messages_ = 0;
    double last_brake_ = -1.0, last_handbrake_ = -1.0;
    vector<tuple<double, char, double>> output_log_; // every command the fake VESC got: S speed, C 0 A, B brake, H handbrake
    int glitch_samples_ = 0; double glitch_erpm_ = 0.0;   // next telemetry samples report glitch_erpm_
#ifdef NOMAD_ENABLE_PLANNING
    // 2026-11-17 (4a): fake localization for the slip/stuck display
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nomad_interfaces::msg::ModuleStatus>::SharedPtr loc_status_pub_;
    bool send_pose_ = true, localization_ready_ = true;
    double body_mps_ = 0.0, pose_x_ = 0.0, last_pose_at_ = 0.0;
#endif
    vector<double> observed_motor_erpm_;
    // Stop ramp tests: the reported ERPM follows each speed command (trailing by follow_lag_, capped at follow_cap_)
    // and drops to 0 on a brake/handbrake command. Times of the speed commands and of the brake/handbrake commands.
    bool follow_ = false;
    double follow_lag_ = 0.0, follow_cap_ = 1000.0;
    vector<pair<double, double>> speed_log_;
    vector<double> stop_output_times_;
};

#ifdef NOMAD_ENABLE_PLANNING
void testPlanningStopOutput() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},
        {"planning.resume_after_stop",true},{"planning.erpm_per_mps",1000.0},
        {"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"stop_brake_current_a",20.0},{"stop_handbrake_current_a",15.0},{"stop_handbrake_below_erpm",200.0}};
    {RosRig stop_output(true,true,true,true,false,parameters);stop_output.runPlanningStopOutput();}
    {RosRig missing(true,true,true,true,false,parameters);missing.runStopOutputDriverMissing();}
    vector<rclcpp::Parameter> load=parameters;
    for(auto& parameter:load)if(parameter.get_name()=="stop_handbrake_current_a")parameter=rclcpp::Parameter("stop_handbrake_current_a",30.0);
    load.insert(load.end(),{{"stop_handbrake_min_current_a",12.5},{"stop_handbrake_load_gain",1.45},
                              {"stop_handbrake_rolling_current_a",6.0},{"stop_handbrake_margin_a",8.0}});
    {RosRig load_hold(true,true,true,true,false,load);load_hold.runPlanningLoadHold();}
}

void testStopRamp() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.minimum_running_erpm",1000.0},{"planning.feedback_limit_erpm",1200.0},
        {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"planning.stop_ramp_s",.3},{"commissioning.overspeed_erpm",2500.0},
        {"stop_brake_current_a",20.0},{"stop_handbrake_current_a",30.0},{"stop_handbrake_below_erpm",200.0},
        {"stop_handbrake_min_current_a",12.5},{"stop_handbrake_load_gain",1.45},
        {"stop_handbrake_rolling_current_a",6.0},{"stop_handbrake_margin_a",8.0}};
    const size_t gui_mark=traceMark();
    for(const string scenario:{"LPP STOP","expiry","climb hold","second publisher","blocked",
                               "cancel","cancel then flat","speed 0","operator STOP","VESC fault","invalid message",
                               "noisy end","link resume","shutdown","shutdown while driving","slip display"}){
        // Scenarios that act during the ramp use a 1 s ramp so they still act while it runs on a slow machine.
        const bool acts_during=scenario=="cancel"||scenario=="cancel then flat"||scenario=="speed 0"||
                               scenario=="operator STOP"||scenario=="VESC fault"||scenario=="invalid message"||
                               scenario=="noisy end";
        auto with_ramp=parameters;
        for(auto& parameter:with_ramp)
            if(parameter.get_name()=="planning.stop_ramp_s")parameter=rclcpp::Parameter("planning.stop_ramp_s",acts_during?1.0:.3);
        const size_t log_mark=traceMark();
        RosRig rig(true,true,true,true,false,with_ramp);rig.runStopRamp(scenario,acts_during?1.0:.3);
        // 2026-11-17 (5a): the commission output rules hold in every scenario.
        rig.checkOutputInvariants("stop ramp ("+scenario+") output: ",12.5,30.0,scenario=="VESC fault");
        // 2026-11-19: no per-tick trace lines by default (trace_every_tick false); summaries at the end instead.
        const string label="stop ramp ("+scenario+") log: ";
        expect(traceSince(log_mark,"[STOP_RAMP] t=").empty()&&traceSince(log_mark,"[RESTART_TRACE] t=").empty(),
               (label+"no per-tick [STOP_RAMP]/[RESTART_TRACE] lines").c_str());
        if(scenario=="LPP STOP"||scenario=="expiry"||scenario=="noisy end"){
            // These ramps end on samples ("2 of the last 3" at or below 200, noisy end before the command reaches 200):
            // the end line's erpm_last includes the deciding sample.
            const auto ends=traceLinesSince(log_mark,"[STOP_RAMP] end");
            const string end=ends.empty()?string():ends.back().second;
            vector<long> last;
            const auto at=end.find(" erpm_last=");
            if(at!=string::npos){
                const char* text=end.c_str()+at+11;char* next=nullptr;
                for(long value=strtol(text,&next,10);next!=text;value=strtol(text,&next,10)){
                    last.push_back(value);text=next;if(*text!='/')break;++text;}
            }
            int low=0;
            for(size_t i=last.size()>=3?last.size()-3:0;i<last.size();++i)low+=last[i]<=200;
            const bool summary=!ends.empty()&&end.find(" wait_s=")!=string::npos&&end.find(" end_samples=")!=string::npos&&
                traceNumber(end,"end_above")>=0&&low>=2;
            expect(summary,(label+"the ramp end line carries the summary with the deciding samples").c_str());
            if(!summary&&!ends.empty())cerr<<"  "<<end<<'\n';
            // No ~420-byte wait line in the burst of the end line (it pushed the end out of the GUI's 1 KiB read).
            bool near_wait=false;
            if(!ends.empty())
                for(const auto& wait:traceLinesSince(log_mark,"[PLANNING_WAIT]"))
                    near_wait=near_wait||llabs(static_cast<long long>(wait.first-ends.back().first))<50000000LL;
            expect(!ends.empty()&&!near_wait,(label+"no [PLANNING_WAIT] in the tick of the ramp end line").c_str());
        }
        if(scenario=="LPP STOP"){
            // The drive start was a restart: its trace ends when the stop ramp begins, with the 1000 ERPM reached.
            const auto ends=traceSince(log_mark,"[RESTART_TRACE] end");
            const bool summary=!ends.empty()&&ends.back().find("reason=stop_ramp")!=string::npos&&
                ends.back().find(" reach_s=")!=string::npos&&ends.back().find("reach_s=na")==string::npos&&
                ends.back().find("iq_peak_a=na")==string::npos;
            expect(summary,(label+"the restart trace ends at the ramp with one summary line (reached, iq peak)").c_str());
            if(!summary&&!ends.empty())cerr<<"  "<<ends.back()<<'\n';
        }
        if(scenario=="expiry"){
            // 2026-11-20: an [LPP_LOST] line on why and how old the last accepted command is (valid_for 0.3 s).
            const auto lost=traceSince(log_mark,"[LPP_LOST]");
            const string line=lost.empty()?string():lost.back();
            const long since=traceNumber(line,"since_ms");
            const auto why=line.find(" why=");
            const bool ok=!lost.empty()&&line.find(" valid_ms=300 ")!=string::npos&&since>=250&&since<=450&&
                traceNumber(line,"age_ms")>=0&&why!=string::npos&&line.substr(why)==" why=planning command expired";
            expect(ok,(label+"an [LPP_LOST] line: since/age/valid fields, why=planning command expired").c_str());
            if(!ok)for(const auto& l:lost)cerr<<"  "<<l<<'\n';
        }
        if(scenario=="cancel"){
            const auto cancels=traceSince(log_mark,"[STOP_RAMP] cancelled");
            const bool ok=!cancels.empty()&&traceNumber(cancels.front(),"cmd_age_ms")>=0;
            expect(ok,(label+"the cancel line carries the fresh command's receipt age (cmd_age_ms)").c_str());
            if(!ok&&!cancels.empty())cerr<<"  "<<cancels.front()<<'\n';
        }
        if(scenario=="VESC fault"){
            // A latched stop during the ramp keeps its wait line (endpoint counts, fault fields): nothing repeats it.
            bool fault_wait=false;
            for(const auto& wait:traceSince(log_mark,"[PLANNING_WAIT]"))fault_wait=fault_wait||wait.find("state=FAULT")!=string::npos;
            expect(fault_wait,(label+"a VESC fault during the ramp still logs its [PLANNING_WAIT] line").c_str());
        }
        if(scenario=="shutdown while driving"){
            const auto ends=traceSince(log_mark,"[RESTART_TRACE] end");
            expect(!ends.empty()&&ends.back().find("reason=shutdown ticks=")!=string::npos,
                   (label+"a shutdown still prints the restart summary").c_str());
        }
    }
    {
        // trace_every_tick true: the per-tick lines come back (bench analysis), the summaries stay.
        auto every=parameters;
        every.emplace_back("trace_every_tick",true);
        const size_t log_mark=traceMark();
        RosRig rig(true,true,true,true,false,every);rig.runStopRamp("LPP STOP",.3);
        expect(!traceSince(log_mark,"[STOP_RAMP] t=").empty()&&!traceSince(log_mark,"[RESTART_TRACE] t=").empty()&&
               !traceSince(log_mark,"[STOP_RAMP] end").empty(),
               "stop ramp (trace_every_tick): per-tick lines and the end line");
    }
    // 2026-11-19: every control_trace, vesc_trace, link_trace and [SLIP] line of these scenarios reads in the team GUI.
    for(const auto& kind:kGuiLogLines){
        const auto lines=traceSince(gui_mark,kind.marker);
        string problem;
        for(const auto& line:lines)if(problem.empty())problem=guiLogProblem(kind,line);
        expect(!lines.empty()&&problem.empty(),("team GUI log contract: "+kind.marker+"lines parse as the GUI expects").c_str());
        if(!problem.empty())cerr<<"  "<<problem<<'\n';
    }
    {
        const auto slips=traceSince(gui_mark,"[SLIP]");
        bool all=!slips.empty();
        for(const auto& line:slips)if(!guiSlipLine(line)){all=false;cerr<<"  [SLIP] the GUI cannot read: "<<line<<'\n';}
        expect(all,"team GUI log contract: [SLIP] lines match the GUI's start/end patterns");
    }
    const vector<rclcpp::Parameter> base={{"mode","commission"},{"planning.headless",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.feedback_limit_erpm",1200.0}};
    const auto refused=[&](const vector<rclcpp::Parameter>& extra){
        auto all=base;all.insert(all.end(),extra.begin(),extra.end());
        rclcpp::NodeOptions options;options.parameter_overrides(all);
        try{auto node=nomad::ControllerNode::create(options);}
        catch(const invalid_argument& error){return string(error.what()).find("stop_ramp_s")!=string::npos;}
        return false;
    };
    expect(!refused({{"planning.stop_ramp_s",.3}})&&!refused({{"planning.stop_ramp_s",0.0}})&&!refused({{"planning.stop_ramp_s",1.0}}),
           "stop ramp: 0..1 s accepted with headless measured planning");
    expect(refused({{"planning.stop_ramp_s",-.1}})&&refused({{"planning.stop_ramp_s",1.01}})&&
           refused({{"planning.stop_ramp_s",numeric_limits<double>::quiet_NaN()}}),"stop ramp: out-of-range time refused");
    expect(refused({{"planning.stop_ramp_s",.3},{"planning.resume_after_stop",false}})&&
           refused({{"planning.stop_ramp_s",.3},{"planning.measured_speed_control",false},{"planning.minimum_running_erpm",0.0}}),
           "stop ramp: refused without headless measured planning that resumes after a stop");
}

// 2026-11-19: the node as the Pi4 starts it (pi4_output_only.sh: commissioning, planning, planning_headless,
// nomad_vehicle_ready, in this order) and the driver steering file it starts the VESC driver with.
void testDeployedConfiguration() {
    const string dir=NOMAD_SOURCE_CONFIG_DIR;
    vector<rclcpp::Parameter> deployed;
    try{
        deployed=paramsFromFiles({dir+"/commissioning.yaml",dir+"/planning.yaml",dir+"/planning_headless.yaml",
                                  dir+"/nomad_vehicle_ready.yaml"},"/nomad_vesc_trial");
    }catch(const exception& error){cerr<<error.what()<<'\n';}
    expect(deployed.size()>50,"deployed config: the four yaml files parse for /nomad_vesc_trial");
    const auto value=[&](const string& name)->optional<rclcpp::ParameterValue>{
        for(const auto& parameter:deployed)if(parameter.get_name()==name)return parameter.get_parameter_value();
        return nullopt;};
    expect(value("mode")&&value("mode")->get<string>()=="commission"&&value("planning.headless")&&
           value("planning.headless")->get<bool>(),"deployed config: commission mode with the headless planner");
    // The VESC driver runs in /nomad_bench (pi4_output_only.sh -r __ns:=/nomad_bench) with the f1tenth topic names.
    // Name and value together: a swapped pair (speed <-> duty) or a misspelt name is a failure (review 11-19).
    const map<string,string> driver_topics={{"telemetry_topic","/nomad_bench/sensors/core"},
        {"motor_speed_topic","/nomad_bench/commands/motor/speed"},{"motor_current_topic","/nomad_bench/commands/motor/current"},
        {"servo_topic","/nomad_bench/commands/servo/position"},{"motor_duty_topic","/nomad_bench/commands/motor/duty_cycle"},
        {"motor_brake_topic","/nomad_bench/commands/motor/brake"},{"motor_handbrake_topic","/nomad_bench/commands/motor/handbrake"},
        {"motor_position_topic","/nomad_bench/commands/motor/position"}};
    vector<rclcpp::Parameter> rig_params;
    map<string,string> topics;
    for(const auto& parameter:deployed){
        const string& name=parameter.get_name();
        if(name.size()>6&&name.compare(name.size()-6,6,"_topic")==0){
            topics[name]=parameter.get_type()==rclcpp::ParameterType::PARAMETER_STRING?parameter.as_string():string("?");
            continue;
        }
        rig_params.push_back(parameter);  // topics stay the rig's own: this test never uses the driver's topics
    }
    expect(topics==driver_topics,"deployed config: each VESC topic setting names its driver topic under /nomad_bench");
    if(topics!=driver_topics)for(const auto& [name,topic]:topics)cerr<<"  "<<name<<" = "<<topic<<'\n';
    // The driver passes servo positions only inside its own servo_min..servo_max.
    try{
        const auto driver=paramsFromFiles({dir+"/vesc_steering_test.yaml"},"/nomad_bench/vesc_driver_lpp");
        double driver_min=NAN,driver_max=NAN;
        for(const auto& parameter:driver){
            if(parameter.get_name()=="servo_min")driver_min=parameter.as_double();
            if(parameter.get_name()=="servo_max")driver_max=parameter.as_double();
        }
        expect(value("commissioning.servo_min")&&value("commissioning.servo_max")&&
               driver_min<=value("commissioning.servo_min")->get<double>()&&
               driver_max>=value("commissioning.servo_max")->get<double>(),
               "deployed config: the driver's servo range covers the controller's");
    }catch(const exception& error){cerr<<error.what()<<'\n';expect(false,"deployed config: the driver steering file parses");}
    const size_t log_mark=traceMark();
    try{
        RosRig rig(true,true,true,true,false,rig_params);
        rig.runDeployedConfiguration(rig_params);
    }catch(const exception& error){cerr<<error.what()<<'\n';expect(false,"deployed config: the node starts with the deployed values");}
    // [CONFIG] lists names the node never read; none of them may come from the deployed files.
    bool clean=true;
    for(const auto& name:configUnused(traceSince(log_mark,"[CONFIG]")))
        for(const auto& parameter:rig_params)
            if(parameter.get_name()==name){clean=false;cerr<<"  unused deployed setting: "<<name<<'\n';}
    expect(clean,"deployed config: no deployed setting is reported unused");
    // A typo is reported (and the node still starts with the default).
    const size_t typo_mark=traceMark();
    try{
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"mode","commission"},{"planning.headless",true},{"planning.resume_after_stop",true},
            {"planning.measured_speed_control",true},{"planning.feedback_limit_erpm",1200.0},{"planning.stop_ramp_sec",.5},
            {"current_limit_a",25.0},{"commissioning.current_limit_a",30.0}});
        auto node=nomad::ControllerNode::create(options);
    }catch(const exception& error){cerr<<error.what()<<'\n';}
    const auto unused=configUnused(traceSince(typo_mark,"[CONFIG]"));
    expect(unused.count("planning.stop_ramp_sec")==1,"deployed config: a misspelt setting is reported by name");
    expect(unused.count("current_limit_a")==1&&unused.count("commissioning.current_limit_a")==0,
           "deployed config: a calibrated-mode setting is reported in commission mode, its commissioning.* twin is not");
}

void testMinimumRunningFloor() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.minimum_running_erpm",700.0},
        {"planning.start_hold_s",.75},{"planning.restart_cooldown_s",1.0},
        {"planning.minimum_start_erpm",550.0},{"planning.feedback_limit_erpm",1200.0},
        {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"commissioning.overspeed_erpm",2500.0},{"commissioning.stationary_hold_s",.5}};
    {RosRig floor(true,true,true,true,false,parameters);floor.runMinimumRunningFloor();}
    {RosRig manual(true,true);manual.runManualBelowRunningFloor();}
}

void testMeasuredSpeedControl() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.start_hold_s",.75},{"planning.restart_cooldown_s",1.0},
        {"planning.minimum_start_erpm",550.0},{"planning.feedback_limit_erpm",1200.0},
        {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"commissioning.overspeed_erpm",2500.0},{"commissioning.stationary_hold_s",.5}};
    auto feedback_parameters=parameters;
    feedback_parameters.emplace_back("planning.minimum_running_erpm",0.0);
    {RosRig feedback(true,true,true,true,false,feedback_parameters);feedback.runMeasuredSpeedFeedback();}
    for(const string failure:{"operator STOP","invalid command","command expiry","telemetry timeout","VESC fault","hard overspeed"}) {
        RosRig guarded(true,true,true,true,false,parameters);guarded.runMeasuredSpeedSafety(failure);
    }
}
#endif

void testRawMetadataConfiguration() {
    const double nan = numeric_limits<double>::quiet_NaN();
    const double infinity = numeric_limits<double>::infinity();
    const vector<rclcpp::Parameter> invalid = {
        {"steering_input_limit_rad", 0.0}, {"steering_input_limit_rad", -0.01},
        {"steering_input_limit_rad", 0.5501}, {"steering_input_limit_rad", nan},
        {"steering_input_limit_rad", infinity}, {"observed_smooth_start_erpm", -1.0},
        {"observed_smooth_start_erpm", nan}, {"observed_smooth_start_erpm", infinity},
        {"stop_brake_current_a", -1.0}, {"stop_brake_current_a", 30.5}, {"stop_brake_current_a", nan},
        {"stop_handbrake_current_a", -1.0}, {"stop_handbrake_current_a", 30.5}, {"stop_handbrake_below_erpm", 2500.0}};
    for (const auto& parameter : invalid) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"mode", "commission"}, parameter});
        bool rejected = false;
        try { auto node = nomad::ControllerNode::create(options); }
        catch (const invalid_argument&) { rejected = true; }
        expect(rejected, "invalid raw angle or observation metadata is rejected before output creation");
    }
}
} // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    previous_log_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(captureTraceLine);
    try {
#ifdef NOMAD_ENABLE_PLANNING
        if(argc>1&&string(argv[1])=="--running-floor-only"){
            testMinimumRunningFloor();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--measured-speed-only"){
            testMeasuredSpeedControl();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--deployed-config-only"){
            testDeployedConfiguration();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--stop-ramp-only"){
            testStopRamp();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--stop-output-only"){
            testPlanningStopOutput();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--headless-resume-only"){
            {
            RosRig resume(true,true,true,true,false,{
                {"planning.headless",true},{"planning.allow_estimated_calibration",true},
                {"planning.resume_after_stop",true},{"planning.erpm_per_mps",1000.0},
                {"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3}});
            resume.runHeadlessResume();
            }
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
#endif
        testRawMetadataConfiguration();
        for (const auto& params : vector<vector<rclcpp::Parameter>>{
                {{"operation_environment","ground"}},{{"mode","commission"},{"stability.enabled",true}}}) {
            rclcpp::NodeOptions options;options.parameter_overrides(params);
            bool refused=false;
            try { auto node=nomad::ControllerNode::create(options); }
            catch(const invalid_argument&) { refused=true; }
            expect(refused,"unvalidated ground/raw stability modes refused");
        }
#ifdef NOMAD_ENABLE_PLANNING
        for (const string mode : {"bench","monitor"}) {
            rclcpp::NodeOptions options;
            options.parameter_overrides({{"mode",mode},{"planning.headless",true},
                {"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true}});
            bool refused=false;
            try { auto node=nomad::ControllerNode::create(options); }
            catch(const invalid_argument& error) {
                refused=string(error.what()).find("planning.headless")!=string::npos;
            }
            expect(refused,"headless planning outside commission mode refused at startup");
        }
#endif
        { RosRig stability(true,false,true,true,false,{
            {"stability.enabled",true},{"stability.geometry_verified",true},{"stability.frame_verified",true},
            {"stability.track_m",.25},{"stability.wheelbase_m",.335},{"stability.cg_height_m",.15},
            {"stability.cg_from_rear_m",.17}});stability.runStability(); }
        { RosRig monitor(false); monitor.runMonitor(); }
#ifdef NOMAD_ENABLE_PLANNING
        testMeasuredSpeedControl();
        testPlanningStopOutput();
        // testStopRamp() runs as its own ctest (planning_stop_ramp_ros, --stop-ramp-only).
        { RosRig stable(true,true,true,true,false,{
            {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
            {"planning.start_hold_s",.2},{"planning.restart_cooldown_s",.3},{"planning.minimum_start_erpm",550.0},
            {"planning.feedback_limit_erpm",1200.0},{"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},
            {"planning.max_mps",.25},{"planning.max_rad",.3}});stable.runStableHeadlessStart(); }
        { RosRig headless(true,true,true,true,false,{
            {"planning.headless",true},{"planning.allow_estimated_calibration",true},
            {"planning.erpm_per_mps",1000.0},{"planning.servo_per_rad",.5},
            {"planning.max_mps",.25},{"planning.max_rad",.3}});headless.runHeadless(); }
        { RosRig planning(true,true,true,true,false,{
            {"planning.calibration_verified",true},{"planning.erpm_per_mps",1000.0},
            {"planning.feedback_erpm_sign",1},{"planning.servo_per_rad",-.5},
            {"planning.max_mps",.6},{"planning.max_rad",.1}});planning.runPlanning(); }
        { RosRig table(true,true,true,true,false,{
            {"planning.calibration_verified",true},{"planning.erpm_per_mps",1000.0},
            {"planning.feedback_erpm_sign",1},{"planning.max_mps",.6},{"planning.max_rad",.1},
            {"planning.steering_rads",vector<double>{-.1,0.0,.1}},
            {"planning.steering_servos",vector<double>{.55,.5,.45}}});table.runPlanning(); }
#endif
        { RosRig bench(true); bench.runBench(); }
        { RosRig braked(true, false, true, true, false, {{"stop_brake_current_a", 8.0}}); braked.runBenchBrake(8.0); }
        { RosRig held(true, false, true, true, false, {{"stop_brake_current_a", 8.0}, {"stop_handbrake_current_a", 12.0}});
          held.runBenchHandbrake(12.0); }
        { RosRig faulted(true, false, true, true, false, {{"stop_brake_current_a", 8.0}, {"stop_handbrake_current_a", 12.0}});
          faulted.runBenchHandbrakeFault(12.0); }
        { RosRig manual(true); manual.runManual(); }
        { RosRig gui(true); gui.runGuiBridge(); }
        { RosRig raw(true, true); raw.runCommission(); }
        { RosRig raw_gui(true, true); raw_gui.runCommissionGuiBridge(); }
        { RosRig calibration(true,true);calibration.runSteeringRecords(); }
        { RosRig continuous(true, true); continuous.runContinuousCommission(); }
        { RosRig reverse(true, true, true, true, false, {{"commissioning.manual_reverse_max_erpm", 600.0}});
          reverse.runManualReverse(); }
        { RosRig thermal(true, true); thermal.runManualThermalStop(); }
        { RosRig wifi(true, true, true, true, false, {{"commissioning.manual_timeout_s", 2.0}});
          wifi.runWifiManualProfile(); }
        { RosRig optimized(true, true); optimized.runRawMotorOptimization(); }
        { RosRig expanded(true, true, true, true, false, {
              {"steering_input_limit_rad", 0.55}, {"observed_smooth_start_erpm", 550.0},
              {"commissioning.servo_min", 0.3625}, {"commissioning.servo_max", 0.6375}});
          expanded.runExpandedSteeringContract(); }
        { RosRig stalled(true, true, true, true, true); stalled.runRawStallWatchdog(); }
        { RosRig no_setup(true, true, false, true); no_setup.runCommissionGate(); }
        { RosRig no_timeout(true, true, true, false); no_timeout.runCommissionGate(); }
    } catch (const exception& error) {
        ++failures;
        cerr << "ROS integration test error: " << error.what() << '\n';
    }
    rclcpp::shutdown();
    cout << checks << " ROS checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

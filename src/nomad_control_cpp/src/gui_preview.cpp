#include "nomad_control_cpp/gui_http.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace std;

namespace {
// 11-16: a simulated servo for the steering gauge: the target alternates every 2.5 s and the output follows it at
// the controller's manual servo rate (1.0/s), so the preview shows the movement. No device is involved.
struct PreviewServo { double target, output; bool moving; };
PreviewServo previewServo() {
    static const auto start = chrono::steady_clock::now();
    const double t = chrono::duration<double>(chrono::steady_clock::now() - start).count();
    const double period = 2.5, rate = 1.0;
    const int step = static_cast<int>(t / period);
    const double target = step % 2 ? 0.80 : 0.25, previous = step % 2 ? 0.25 : 0.80;
    const double since = t - step * period;
    const double output = step == 0 ? 0.50 + copysign(min(abs(target - 0.50), rate * since), target - 0.50)
                                    : previous + copysign(min(abs(target - previous), rate * since), target - previous);
    return {target, output, abs(target - output) > 1e-4};
}
double previewDeg(double servo) { return (servo - 0.5) / 0.5 * 35.0; } // the car's LPP table: servo 0..1 = -35..+35 deg
} // namespace

// Windows에서도 화면을 확인할 수 있는 전용 실행 파일. ROS·장치 출력 코드는 링크하지 않습니다.
int main(int argc, char** argv) {
    try {
        const string page = argc > 1 ? argv[1] : "web/index.html";
        // 미리보기는 실제 SSH 제어 포트(8765)와 분리합니다.
        int port=8766;
        if(argc>2){
            char* end=nullptr;const auto value=strtol(argv[2],&end,10);
            if(end==argv[2]||*end!='\0'||value<1024||value>65535)
                throw invalid_argument("preview port must be 1024..65535");
            port=static_cast<int>(value);
        }
        nomad::GuiHttp server(page, port, [] {
            const auto servo = previewServo();
            return nomad::Json{{"preview", true}, {"connected", false}, {"state", "PREVIEW"},
                {"reason", "화면 미리보기: 실제 ROS/VESC에 연결하지 않습니다."},
                {"control_units", "raw"}, {"continuous_manual_supported", true},
                {"continuous_manual_active", false}, {"motor_max_erpm", 1850.0},
                {"servo_min", 0.0}, {"servo_max", 1.0}, {"servo_center", 0.50},
                {"steering_input_limit_rad", 0.50}, {"motor_control_mode", "vesc_speed_pid"},
                {"speed_pid_min_erpm", 50.0}, {"speed_pid_min_erpm_verified", false},
                {"commanded_servo_position", servo.output}, {"requested_servo_position", servo.target},
                {"servo_moving", servo.moving}, {"steering_estimate_deg", previewDeg(servo.output)},
                {"steering_target_estimate_deg", previewDeg(servo.target)}, {"commanded_motor_erpm", 0.0},
                {"servo_rate_s", 1.0}, {"max_duration_s", 3.0}};
        }, [](const string&, const nomad::Json&) {
            return nomad::Json{{"accepted", false}, {"reason", "PREVIEW: actuator commands are unavailable"}};
        });
        cout << "Preview only: http://127.0.0.1:" << port << '\n' << flush;
        return server.listen() ? 0 : 1;
    } catch (const exception& e) { cerr << e.what() << '\n'; return 1; }
}

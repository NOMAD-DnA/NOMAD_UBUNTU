#include "nomad_control_cpp/gui_http.hpp"
#include <iostream>
#include <cstdlib>

// Windows에서도 화면을 확인할 수 있는 전용 실행 파일. ROS·장치 출력 코드는 링크하지 않습니다.
int main(int argc, char** argv) {
    try {
        const std::string page = argc > 1 ? argv[1] : "web/index.html";
        // 미리보기는 실제 SSH 제어 포트(8765)와 분리합니다.
        int port=8766;
        if(argc>2){
            char* end=nullptr;const auto value=std::strtol(argv[2],&end,10);
            if(end==argv[2]||*end!='\0'||value<1024||value>65535)
                throw std::invalid_argument("preview port must be 1024..65535");
            port=static_cast<int>(value);
        }
        nomad::GuiHttp server(page, port, [] {
            return nomad::Json{{"preview", true}, {"connected", false}, {"state", "PREVIEW"},
                {"reason", "화면 미리보기: 실제 ROS/VESC에 연결하지 않습니다."},
                {"control_units", "raw"}, {"continuous_manual_supported", true},
                {"continuous_manual_active", false}, {"motor_max_erpm", 1850.0},
                {"servo_min", 0.0}, {"servo_max", 1.0}, {"servo_center", 0.50},
                {"steering_input_limit_rad", 0.50}, {"motor_control_mode", "vesc_speed_pid"},
                {"speed_pid_min_erpm", 50.0}, {"speed_pid_min_erpm_verified", false},
                {"commanded_servo_position", 0.50}, {"commanded_motor_erpm", 0.0},
                {"servo_rate_s", 0.40}, {"max_duration_s", 3.0}};
        }, [](const std::string&, const nomad::Json&) {
            return nomad::Json{{"accepted", false}, {"reason", "PREVIEW: actuator commands are unavailable"}};
        });
        std::cout << "Preview only: http://127.0.0.1:" << port << '\n' << std::flush;
        return server.listen() ? 0 : 1;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

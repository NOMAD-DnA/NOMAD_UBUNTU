#include "nomad_control_cpp/vesc_mapping.hpp"
#include "calibration.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// 명령행 숫자를 읽습니다. 숫자 뒤에 문자가 남으면 입력 오류로 처리합니다.
double parseNumber(const char* input) {
    const std::string text(input);
    std::size_t consumed = 0; // 0.3oops처럼 일부만 숫자인 입력을 검출하기 위한 길이
    const double value = std::stod(text, &consumed);
    if (consumed != text.size()) throw std::invalid_argument("Trailing characters in number");
    return value;
}

// 오프라인 계산 예제. 인자: 목표 m/s, 등가각 rad, 선택적 raw 측정 ERPM입니다.
int main(int argc, char* argv[]) {
    if (argc != 1 && argc != 3 && argc != 4) {
        std::cerr << "Usage: nomad_convert [speed_mps steering_rad [raw_feedback_erpm]]\n";
        return 2;
    }
    try {
        // 인자가 없으면 가상 예시를 사용합니다. feedback도 실시간 센서가 아닌 입력 숫자입니다.
        const double speed = argc >= 3 ? parseNumber(argv[1]) : 0.30;
        const double steering = argc >= 3 ? parseNumber(argv[2]) : 0.15;
        const double feedback = argc == 4 ? parseNumber(argv[3]) : -1740.0;
        const auto& cfg = nomad::config::kCalibration;
        const auto target = nomad::convertTarget(speed, steering, cfg);
        const auto measured = nomad::convertFeedback(feedback, cfg);
        std::cout << "OFFLINE ONLY: no ROS, USB, motor or servo output\n";
        std::cout << "calibration_measured=" << std::boolalpha << nomad::config::kMeasuredOnVehicle << '\n';
        std::cout << nomad::config::kCalibrationNote << '\n';
        // optional 결과가 있는지 확인한 뒤에만 계산값에 접근합니다.
        if (!target.value) {
            std::cerr << "Target error: " << nomad::errorMessage(target.error) << '\n';
            return 1;
        }
        if (!measured.value) {
            std::cerr << "Feedback error: " << nomad::errorMessage(measured.error) << '\n';
            return 1;
        }
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "motor_erpm=" << target.value->motor_erpm
                  << " servo_position=" << target.value->servo_position << '\n';
        std::cout << "feedback_speed_mps=" << *measured.value << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Input error: " << error.what() << '\n';
        return 1;
    }
}

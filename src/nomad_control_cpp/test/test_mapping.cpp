#include "nomad_control_cpp/vesc_mapping.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int checks = 0;
int failures = 0;

// assert와 달리 Release 빌드에서도 유지되는 검사입니다. 실패 항목을 모아 보고합니다.
void expect(bool condition, const std::string& name) {
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

// 반올림 오차를 허용해 실수를 비교합니다. 허용 절대 오차는 1e-9입니다.
bool near(double actual, double expected) {
    return std::abs(actual - expected) <= 1e-9;
}

// 실차 설정과 독립적인 고정 시험 데이터입니다. 보정값 변경에 맞춰 이 기준값을 바꾸지 않습니다.
nomad::Calibration fixture() {
    return {6000.0, 0.0, -0.60, 0.52, 1, -1,
            0.0, 0.40, -0.20, 0.20, 0.40, 0.64, -3000.0, 3000.0};
}

// 독립적으로 계산한 속도·조향 기준값과 허용 범위의 끝점을 확인합니다.
void testKnownValues() {
    const auto cfg = fixture();
    const auto result = nomad::convertTarget(0.30, 0.15, cfg);
    expect(result.value.has_value(), "known target accepted");
    if (result.value) {
        expect(near(result.value->motor_erpm, 1800.0), "0.30 m/s -> 1800 ERPM");
        expect(near(result.value->servo_position, 0.43), "0.15 rad -> servo 0.43");
    }
    const auto feedback = nomad::convertFeedback(-1740.0, cfg);
    expect(feedback.value && near(*feedback.value, 0.29), "raw -1740 -> +0.29 m/s");
    for (double angle : {-0.20, 0.0, 0.20}) {
        expect(nomad::convertTarget(0.40, angle, cfg).value.has_value(), "inclusive angle endpoints");
    }
    const auto stopped = nomad::convertTarget(0.0, 0.0, cfg);
    expect(stopped.value && near(stopped.value->motor_erpm, 0.0) &&
           near(stopped.value->servo_position, 0.52), "zero target and neutral");
}

// 비정상 입력·보정 범위 초과·출력 포화가 실행 가능한 결과로 바뀌지 않는지 검사합니다.
void testRejections() {
    using nomad::Error;
    auto cfg = fixture();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    expect(nomad::convertTarget(nan, 0.0, cfg).error == Error::NonFiniteInput, "NaN speed");
    expect(nomad::convertTarget(0.2, inf, cfg).error == Error::NonFiniteInput, "infinite steering");
    expect(nomad::convertFeedback(inf, cfg).error == Error::NonFiniteInput, "infinite telemetry");
    expect(nomad::convertTarget(-0.1, 0.0, cfg).error == Error::SpeedOutOfRange, "uncalibrated reverse");
    expect(nomad::convertTarget(0.4001, 0.0, cfg).error == Error::SpeedOutOfRange, "overspeed target");
    expect(nomad::convertTarget(0.2, 0.2001, cfg).error == Error::SteeringOutOfRange, "steering outside range");
    cfg.servo_max = 0.60;
    expect(nomad::convertTarget(0.2, -0.2, cfg).error == Error::ServoOutOfRange, "servo saturation rejected");
    cfg = fixture();
    cfg.motor_erpm_max = 1000.0;
    expect(nomad::convertTarget(0.3, 0.0, cfg).error == Error::MotorOutOfRange, "motor saturation rejected");
    cfg = fixture();
    cfg.speed_offset = -1000.0; // 작은 전진 목표를 역방향 ERPM으로 만드는 오류 재현
    expect(nomad::convertTarget(0.1, 0.0, cfg).error == Error::DirectionMismatch, "offset cannot reverse target");
    const auto rejected = nomad::convertTarget(nan, 0.0, fixture());
    expect(!rejected.value.has_value(), "failure has no usable command");
}

// 각 설정 필드의 NaN, 잘못된 gain·부호·범위를 검사합니다.
void testInvalidSettings() {
    double nomad::Calibration::* const fields[] = { // NaN 검사를 위해 각 실수 멤버를 선택하는 포인터 목록
        &nomad::Calibration::speed_gain, &nomad::Calibration::speed_offset,
        &nomad::Calibration::steering_gain, &nomad::Calibration::steering_offset,
        &nomad::Calibration::speed_min_mps, &nomad::Calibration::speed_max_mps,
        &nomad::Calibration::steering_min_rad, &nomad::Calibration::steering_max_rad,
        &nomad::Calibration::servo_min, &nomad::Calibration::servo_max,
        &nomad::Calibration::motor_erpm_min, &nomad::Calibration::motor_erpm_max
    };
    for (auto field : fields) {
        auto cfg = fixture();
        cfg.*field = std::numeric_limits<double>::quiet_NaN();
        expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "NaN config rejected");
        expect(!nomad::convertFeedback(1000.0, cfg).value, "feedback rejects invalid config");
    }
    auto cfg = fixture();
    cfg.speed_gain = 0.0;
    expect(!nomad::convertTarget(0.2, 0.0, cfg).value, "zero speed gain");
    cfg = fixture(); cfg.steering_gain = 0.0;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "zero steering gain");
    cfg = fixture(); cfg.command_erpm_sign = 0;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "invalid command sign");
    cfg = fixture(); cfg.feedback_erpm_sign = 2;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "invalid feedback sign");
    cfg = fixture(); cfg.servo_max = 1.1;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "invalid servo bounds");
    cfg = fixture(); cfg.speed_min_mps = 0.5;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "reversed speed bounds");
    cfg = fixture(); cfg.steering_offset = 0.8;
    expect(nomad::validateCalibration(cfg) == nomad::Error::InvalidCalibration, "neutral outside servo range");
}

// 실제 측정값을 숨기지 않는지, 영점 보정과 정지 예외 처리가 일관적인지 검사합니다.
void testFeedbackAndOffset() {
    auto cfg = fixture();
    const auto fast = nomad::convertFeedback(-3600.0, cfg);
    expect(fast.value && near(*fast.value, 0.6), "overspeed feedback not clipped");
    const auto reverse = nomad::convertFeedback(1200.0, cfg);
    expect(reverse.value && near(*reverse.value, -0.2), "reverse feedback remains visible");
    cfg.speed_offset = 100.0;
    const auto stopped = nomad::convertTarget(0.0, 0.0, cfg);
    expect(stopped.value && stopped.value->motor_erpm == 0.0, "zero target bypasses offset");
    const auto moving = nomad::convertTarget(0.3, 0.0, cfg);
    expect(moving.value && near(moving.value->motor_erpm, 1900.0), "nonzero offset applied to motion");
    const auto measured = nomad::convertFeedback(-1900.0, cfg);
    expect(measured.value && near(*measured.value, 0.3), "offset inverse conversion");
}

// 명령·측정 부호 4개 조합의 전후진 왕복 변환과 계산 overflow를 검사합니다.
void testRoundTripsAndOverflow() {
    for (int command_sign : {-1, 1}) {
        for (int feedback_sign : {-1, 1}) {
            auto cfg = fixture();
            cfg.speed_min_mps = -0.4;
            cfg.command_erpm_sign = command_sign;
            cfg.feedback_erpm_sign = feedback_sign;
            for (int i = -40; i <= 40; ++i) {
                const double speed = i / 100.0;
                const auto output = nomad::convertTarget(speed, 0.1, cfg);
                expect(output.value.has_value(), "signed target accepted");
                if (!output.value) continue;
                const double raw = output.value->motor_erpm * command_sign * feedback_sign; // 같은 물리 회전을 telemetry 부호로 표현
                const auto measured = nomad::convertFeedback(raw, cfg);
                expect(measured.value && near(*measured.value, speed), "signed round trip");
            }
        }
    }
    auto cfg = fixture();
    cfg.speed_gain = std::numeric_limits<double>::max();
    cfg.speed_max_mps = 2.0;
    expect(nomad::convertTarget(2.0, 0.0, cfg).error == nomad::Error::NonFiniteOutput, "target overflow");
    cfg = fixture(); cfg.speed_gain = std::numeric_limits<double>::min();
    expect(nomad::convertFeedback(-1e300, cfg).error == nomad::Error::NonFiniteOutput, "feedback overflow");
}
}

// 모든 시험을 수행하고 실패가 있으면 종료코드 1로 CTest에 알립니다.
int main() {
    testKnownValues();
    testRejections();
    testInvalidSettings();
    testFeedbackAndOffset();
    testRoundTripsAndOverflow();
    std::cout << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

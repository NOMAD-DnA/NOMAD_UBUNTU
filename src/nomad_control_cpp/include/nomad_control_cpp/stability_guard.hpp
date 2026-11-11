#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace nomad {
// 실측 전에는 0으로 둡니다. 차량 외폭은 윤거가 아니며, 차체 높이는 CG 높이가 아닙니다.
struct StabilityConfig {
    bool geometry_verified = false, frame_verified = false;
    double track_m = 0, wheelbase_m = 0, cg_height_m = 0;
    double cg_from_rear_m = 0, cg_lateral_m = 0;
    double margin = 0.5, max_tilt_rad = 0.20, max_rate_radps = 0.4;
    double timeout_s = 0.20, max_speed_mps = 0.3, max_accel_mps2 = 0.15;
    bool valid() const {
        for (double v : {track_m,wheelbase_m,cg_height_m,cg_from_rear_m,cg_lateral_m,
                         margin,max_tilt_rad,max_rate_radps,timeout_s,max_speed_mps,max_accel_mps2})
            if (!std::isfinite(v)) return false;
        return geometry_verified && frame_verified && track_m>0 && wheelbase_m>0 && cg_height_m>0 &&
            cg_from_rear_m>0 && cg_from_rear_m<wheelbase_m && std::abs(cg_lateral_m)<track_m/2 &&
            margin>0 && margin<=0.5 && max_tilt_rad>0 && max_tilt_rad<0.6 && max_rate_radps>0 &&
            timeout_s>0 && timeout_s<=0.3 && max_speed_mps>0 && max_accel_mps2>0;
    }
};
struct AttitudeSample {
    double stamp_s=0, roll=0, pitch=0, roll_rate=0, pitch_rate=0;
    // 차체 좌표의 specific force: 수평 정지 시 (0,0,+g). 중력을 제거한 가속도가 아닙니다.
    double force_x=0, force_y=0, force_z=0;
};
struct StabilityDecision {
    bool allowed=false;
    double speed_cap_mps=0, accel_cap_mps2=0, lateral_usage=0, longitudinal_usage=0;
    std::string reason="stability data unavailable";
};

// 준정적 지지영역 모델입니다. 턱 충돌·점프·노면 마찰·서스펜션 전복을 보장하는 모델이 아닙니다.
class StabilityGuard {
public:
    explicit StabilityGuard(const StabilityConfig& config) : config_(config) {}
    void invalidate() { sample_.reset(); }
    bool observe(const AttitudeSample& sample, double steady, double ros) {
        for (double v : {sample.stamp_s,sample.roll,sample.pitch,sample.roll_rate,sample.pitch_rate,
                         sample.force_x,sample.force_y,sample.force_z,steady,ros})
            if (!std::isfinite(v)) { invalidate(); return false; }
        if (sample.stamp_s<=0 || ros-sample.stamp_s < -0.02 || ros-sample.stamp_s>config_.timeout_s ||
            (last_stamp_ && sample.stamp_s<=*last_stamp_) ||
            (last_receive_ && steady<*last_receive_)) { invalidate(); return false; }
        sample_=sample; last_stamp_=sample.stamp_s; last_receive_=steady;
        return true;
    }
    StabilityDecision evaluate(double steady, double ros, double speed, double steering) const {
        StabilityDecision d;
        if (!config_.valid()) { d.reason="stability geometry/frame unverified"; return d; }
        if (!std::isfinite(steady)||!std::isfinite(ros)||!std::isfinite(speed)||
            !std::isfinite(steering)||std::abs(steering)>=1.0) { d.reason="invalid stability input"; return d; }
        if (std::abs(speed)>config_.max_speed_mps) { d.reason="stability speed limit exceeded"; return d; }
        if (!sample_ || !last_receive_ || steady-*last_receive_<0 || steady-*last_receive_>config_.timeout_s ||
            ros-sample_->stamp_s < -0.02 || ros-sample_->stamp_s>config_.timeout_s) {
            d.reason="IMU missing, stale or invalid"; return d;
        }
        const auto& s=*sample_;
        if (std::abs(s.roll)>=config_.max_tilt_rad || std::abs(s.pitch)>=config_.max_tilt_rad ||
            std::abs(s.roll_rate)>=config_.max_rate_radps || std::abs(s.pitch_rate)>=config_.max_rate_radps) {
            d.reason="tilt or tilt-rate limit"; return d;
        }
        constexpr double g=9.80665;
        const double normal=g*std::cos(s.roll)*std::cos(s.pitch);
        // 접지 이탈/과격한 수직 충격을 제한합니다. 실측 센서 진동 특성에 대한 추가 검증이 필요합니다.
        if (s.force_z<0.5*g || s.force_z>1.5*g) { d.reason="abnormal vertical specific force"; return d; }
        const double support=std::min(normal,s.force_z);
        const double lat=config_.margin*(config_.track_m/2-std::abs(config_.cg_lateral_m))*support/config_.cg_height_m;
        const double lon=config_.margin*std::min(config_.cg_from_rear_m,
            config_.wheelbase_m-config_.cg_from_rear_m)*support/config_.cg_height_m;
        const double gravity_lat=g*std::abs(std::sin(s.roll));
        const double gravity_lon=g*std::abs(std::sin(s.pitch));
        const double curvature=std::abs(std::tan(steering))/config_.wheelbase_m;
        // 기울기와 원심 가속도가 같은 방향인 최악 조합을 사용합니다.
        d.lateral_usage=std::max(gravity_lat+speed*speed*curvature,std::abs(s.force_y))/lat;
        d.longitudinal_usage=std::max(gravity_lon,std::abs(s.force_x))/lon;
        if (d.lateral_usage>=1 || d.longitudinal_usage>=1) { d.reason="support margin exceeded"; return d; }
        d.speed_cap_mps=curvature>1e-9?std::min(config_.max_speed_mps,std::sqrt((lat-gravity_lat)/curvature)):config_.max_speed_mps;
        d.accel_cap_mps2=std::min(config_.max_accel_mps2,lon-gravity_lon);
        d.allowed=true; d.reason="experimental envelope available; not ground-drive approval";
        return d;
    }
private:
    StabilityConfig config_;
    std::optional<AttitudeSample> sample_;
    std::optional<double> last_stamp_,last_receive_;
};
} // namespace nomad

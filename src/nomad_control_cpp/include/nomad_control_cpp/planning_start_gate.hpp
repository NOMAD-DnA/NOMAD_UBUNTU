#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include "nomad_control_cpp/control_limits.hpp"

namespace nomad {

// 출발 허용만 계산합니다. 목표값을 변경하거나 하드웨어 명령을 만들지 않습니다.
// update는 guard가 새로 승인한 명령에만 호출하고 ready는 제어 tick에서 조회합니다.
class PlanningStartGate {
public:
    explicit PlanningStartGate(double hold_s = .75, double cooldown_s = 1.,
                               double minimum_start_erpm = 550.)
        : hold_s_(hold_s), cooldown_s_(cooldown_s), minimum_start_erpm_(minimum_start_erpm) {
        if (!finiteNonnegative(hold_s_) || !finiteNonnegative(cooldown_s_) ||
            !finiteNonnegative(minimum_start_erpm_))
            throw std::invalid_argument("invalid planning start gate configuration");
    }

    // STOP/PAUSE/무효 입력에서 호출합니다. hard STOP의 재허용 여부는 상위 상태 머신이 소유합니다.
    void reset(double steady_now, bool apply_cooldown = true) {
        since_.reset(); last_command_.reset();
        reason_ = "waiting for stable planning commands";
        if (!finiteNonnegative(steady_now)) {
            cooldown_since_.reset(); last_observed_.reset();
            reason_ = "invalid planning start gate clock";
            return;
        }
        last_observed_ = steady_now;
        if (apply_cooldown) cooldown_since_ = steady_now;
    }

    // valid_for는 수신 시점에 남은 유효기간입니다. 기존 guard 검사도 계속 유지해야 합니다.
    // 0 ERPM은 조향 전용 준비를 허용하지만, 0 스트림은 다음 구동 명령을 미리 승인하지 않습니다.
    bool update(double steady_now, double target_erpm, double valid_for) {
        if (!finiteNonnegative(steady_now) || !std::isfinite(target_erpm) ||
            !std::isfinite(valid_for) || valid_for <= 0 || valid_for > .5 || target_erpm < 0 ||
            (last_observed_ && steady_now < *last_observed_)) {
            reset(steady_now, true);
            reason_ = "invalid planning start gate input";
            return false;
        }
        last_observed_ = steady_now;
        if (target_erpm > 0 && target_erpm < minimum_start_erpm_) {
            since_.reset(); last_command_.reset();
            reason_ = "planning target below minimum start ERPM; target not increased";
            return false;
        }
        const bool motion = target_erpm > 0;
        const bool gap = last_command_ && steady_now - *last_command_ > lease_s_;
        if (!since_ || gap || motion != motion_) since_ = steady_now;
        motion_ = motion;
        last_command_ = steady_now;
        lease_s_ = std::min(valid_for, limits::kPlanningReceiveLeaseS);
        reason_ = "waiting for stable planning commands";
        return true;
    }

    bool ready(double steady_now) const { return reason(steady_now) == "ready"; }

    std::string reason(double steady_now) const {
        if (!finiteNonnegative(steady_now) || (last_observed_ && steady_now < *last_observed_))
            return "invalid planning start gate clock";
        if (!last_command_ || !since_) return reason_;
        if (steady_now - *last_command_ > lease_s_)
            return "planning command gap; waiting for stable stream";
        if (cooldown_since_ && steady_now - *cooldown_since_ < cooldown_s_)
            return "planning restart cooldown";
        if (steady_now - *since_ < hold_s_) return "waiting for stable planning commands";
        return "ready";
    }

private:
    static bool finiteNonnegative(double value) { return std::isfinite(value) && value >= 0; }
    double hold_s_, cooldown_s_, minimum_start_erpm_, lease_s_ = 0;
    bool motion_ = false;
    std::optional<double> since_, last_command_, cooldown_since_, last_observed_;
    std::string reason_ = "waiting for stable planning commands";
};

} // namespace nomad

#include "nomad_thermal/thermal_guard.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace std;
using nomad_thermal::ThermalGuard;
static int checks = 0;
static void expect(bool v, const char* why) { ++checks; if (!v) throw runtime_error(why); }
int main() {
  try {
    ThermalGuard g;
    g.update({40, 42, 41}); expect(g.valid() && g.state() == "ok" && g.scale() == 1. && !g.stopped(), "ok below warn");
    g.update({66, 0, 0}); expect(g.state() == "warn" && g.scale() == 1., "warn only at 65..70");
    g.update({75}); expect(g.state() == "derate" && abs(g.scale() - .65) < 1e-9, "linear derate 75C -> 0.65");
    expect(g.update({80.1, 30, 30}) && g.stopped() && g.scale() == 0. && g.state() == "stop", "stop at >=80C");
    g.update({75}); expect(g.stopped(), "hysteresis: still stopped at 75C");
    expect(g.update({69.9}) && !g.stopped(), "resume below 70C");
    ThermalGuard u; u.update({0, -5, numeric_limits<double>::quiet_NaN(), 300});
    expect(!u.valid() && u.state() == "unknown" && u.scale() == 1. && !u.stopped(), "invalid readings never block");
    g.update({85}); expect(g.stopped(), "stop again"); g.update({0, 0, 0});
    expect(g.stopped() && !g.valid(), "lost sensor while stopped keeps stop until a valid cool reading");
    cout << checks << " thermal guard checks PASS\n"; return 0;
  } catch (const exception& e) { cerr << e.what() << "\n"; return 1; }
}

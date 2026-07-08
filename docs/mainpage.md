Rosmaster C++ Driver — API Reference {#mainpage}
================================================

Single-header, fully-inline **C++17** driver for the **Yahboom Rosmaster** motion
controller driving a 4-wheel mecanum robot ("MecaMate"). It ports Yahboom's Python
driver (V3.3.9) and adds a per-motor **software velocity PID**, a **kS/kV motor
feedforward**, **automatic calibration**, and a **power-loss-resilient POSIX serial
lifecycle**. Intended as the backend for a `ros2_control` hardware interface, but it
also runs standalone.

This site is the **generated API reference**. For the full narrative guide —
protocol details, PID theory, calibration procedure, `ros2_control` integration,
tuning, and troubleshooting — see **README.md** in the repository. Developer/build
conventions are in **CONTRIBUTING.md**.

Where to start
--------------

- \ref Rosmaster — the main driver class: construction, motor control, the software
  PID lifecycle, calibration, feedforward, sensors, and configuration. Every method,
  member, and constant is documented inline.
- \ref SerialPort — the hardened POSIX/Win32 serial wrapper (the FIX-1..FIX-7
  power-cycle lifecycle).

Key features
------------

- **Per-motor software velocity PID** (25 Hz) with a timestamp-based sliding-window
  velocity estimate, sign-correct derivative, and sign-aware anti-windup.
- **kS/kV feedforward** compensating brushed-DC static friction (dead zone) and the
  PWM→speed slope, so the PID only trims the residual.
- **Automatic calibration**: `Rosmaster::calibrate_motor_scales()` (multi-run,
  ratio-based, per-motor ticks/s scales) and `Rosmaster::calibrate_feedforward()`
  (PWM sweep → kS dead zone + kV least-squares slope).
- **Robust serial lifecycle**: survives CH340/CP210x USB power-cycles and VHANGUP so
  the host never needs a reboot after an e-stop.
- **Dependency-free**: C++17 + POSIX only (no Boost, no libserial).

Minimal usage
-------------

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "Rosmaster.hpp"
#include <thread>
#include <chrono>

int main() {
    Rosmaster bot(1, "/dev/ttyUSB0");        // car_type 1 = X3
    bot.create_receive_threading();          // start the UART receive thread
    bot.set_auto_report_state(true, true);   // stream sensors/encoders

    while (bot.get_battery_voltage() == 0.0) // wait for the first packet
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const double scale = bot.calibrate_motor_scales();   // wheels must spin freely
    bot.enable_pid_control(0.6, 0.1, 0.0, scale);        // start the PID loop

    bot.set_motor(40, 40, 40, 40);           // 40 % on all four wheels
    std::this_thread::sleep_for(std::chrono::seconds(3));
    bot.set_motor(0, 0, 0, 0);
    bot.disable_pid_control();
    return 0;
}
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Build a consumer with `g++ -std=c++17 -O2 -pthread your_program.cpp`.

Regenerating this documentation
-------------------------------

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
sudo apt-get install doxygen graphviz     # one-time
./scripts/build_docs.sh --open            # or simply: doxygen Doxyfile
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The result is written to `docs/html/index.html`.

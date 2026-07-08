# Rosmaster C++ Driver — MecaMate

A single-header C++17 driver for the **Yahboom Rosmaster** motion controller
(X3 / X3+ / X1 / R2), built for a 4-wheel mecanum robot. It ports Yahboom's
Python `Rosmaster` library (V3.3.9) to C++ and extends it with a **per-motor
software velocity PID**, a **motor feedforward** model (kS/kV), **automatic
calibration**, and a **power-loss-resilient serial port lifecycle**.

The driver targets a `ros2_control` hardware interface, but works standalone in
any C++ program (see the [Test bench](#13-test-bench-standalone-example)).

> **Language note.** This README is the single, authoritative document for the
> project. It supersedes the earlier French notes (`README_2.md`, `ok.txt`) and
> the standalone test file (`Rosmaster.cpp`), all of which were removed.

---

## Table of contents

1. [Overview](#1-overview)
2. [Repository layout](#2-repository-layout)
3. [Requirements & quick start](#3-requirements--quick-start)
4. [Hardware setup & serial permissions](#4-hardware-setup--serial-permissions)
5. [Thread architecture](#5-thread-architecture)
6. [Yahboom UART protocol](#6-yahboom-uart-protocol)
7. [The software PID loop](#7-the-software-pid-loop)
8. [Motor feedforward (kS/kV)](#8-motor-feedforward-kskv)
9. [Calibration](#9-calibration)
10. [The development story — PID v2 → v8](#10-the-development-story--pid-v2--v8)
11. [Serial-port lifecycle hardening (FIX-1 → FIX-7)](#11-serial-port-lifecycle-hardening-fix-1--fix-7)
12. [Motor model & physical limits](#12-motor-model--physical-limits)
13. [Test bench (standalone example)](#13-test-bench-standalone-example)
14. [ros2_control integration](#14-ros2_control-integration)
15. [Tuning the PID gains](#15-tuning-the-pid-gains)
16. [Public API reference](#16-public-api-reference)
17. [Troubleshooting](#17-troubleshooting)
18. [Known limitations & future work](#18-known-limitations--future-work)
19. [Parameters to adjust](#19-parameters-to-adjust)

---

## 1. Overview

`Rosmaster.hpp` exposes a single class, `Rosmaster`. It is `#pragma once`,
fully `inline`, and dependency-free (no Boost, no libserial — raw POSIX
`termios`). Include it directly in a `ros2_control` system interface or in a
standalone program.

The driver handles:

- **POSIX serial I/O** with the Yahboom microcontroller (CH340 / CP210x USB
  bridge, 115200 baud).
- **Parsing** of the auto-report frames (encoders, IMU, battery, odometry).
- A **per-motor software velocity PID** running in a dedicated thread, with a
  sliding-window anti-aliasing velocity estimate.
- A **feedforward** term (kS static friction + kV velocity slope) so the PID
  only has to correct the residual.
- **Automatic calibration** of per-motor speed scales and feedforward gains.
- A **thread-safe getter** `get_pid_measured()` exposing measured wheel speed
  to higher layers (e.g. a ROS 2 hardware interface).

### Why a software PID?

The Yahboom firmware has its own on-board PID, but its gains are opaque and its
behaviour is not finely controllable from ROS 2. A software loop gives full
control of gains, rate, measurement window, and anti-windup, and drops straight
into a `ros2_control` hardware interface with no firmware dependency.

### Dependencies

- **C++17**, POSIX (Linux). A Windows serial stub exists but is untested.
- No third-party libraries.

---

## 2. Repository layout

| File                    | Purpose                                                        |
| ----------------------- | -------------------------------------------------------------- |
| `Rosmaster.hpp`         | The entire driver — single header, ~2500 lines, header-only. Fully documented with Doxygen comments. |
| `README.md`             | This document — the user guide.                                |
| `CONTRIBUTING.md`       | Developer guide: build, compile-check, docs toolchain, house style. |
| `Doxyfile`              | Doxygen configuration for the HTML API reference.              |
| `scripts/build_docs.sh` | One-command wrapper to generate the API docs.                  |
| `LICENSE`               | License.                                                       |

There is intentionally **no build system** (no CMake / Makefile). The header is
compiled directly by whatever consumes it — a one-line `g++` invocation for the
test bench, or your `ament_cmake` package for ROS 2.

---

## 3. Requirements & quick start

### Build the header into a program

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pthread your_program.cpp -o your_program
```

`-pthread` is required — the driver spawns a receive thread and a PID thread.

### Minimal program

```cpp
#include "Rosmaster.hpp"
#include <thread>
#include <chrono>

int main() {
    // car_type: 1=X3, 2=X3_PLUS, 4=X1, 5=R2
    Rosmaster bot(1, "/dev/ttyUSB0", /*cmd_delay_s=*/0.002, /*debug=*/false);

    // 1. Start the UART receive thread.
    bot.create_receive_threading();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2. Ask the firmware to stream sensor/encoder auto-reports.
    bot.set_auto_report_state(true, /*forever=*/true);

    // 3. Wait for the first valid packet (battery voltage becomes non-zero).
    while (bot.get_battery_voltage() == 0.0)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 4. Calibrate speed scales (wheels must spin freely).
    const double scale = bot.calibrate_motor_scales(/*throttle_pct=*/60,
                                                     /*duration_ms=*/800,
                                                     /*n_runs=*/5,
                                                     /*warmup_ms=*/3000,
                                                     /*use_per_motor=*/true);

    // 5. Enable the PID loop.
    bot.enable_pid_control(/*kp=*/0.6, /*ki=*/0.1, /*kd=*/0.0, /*ticks_per_sec=*/scale);

    // 6. Command 40% forward speed on all four wheels.
    bot.set_motor(40.0, 40.0, 40.0, 40.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 7. Clean stop.
    bot.set_motor(0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // ~2 PID cycles
    bot.disable_pid_control();
    // The destructor joins both threads and closes the port.
    return 0;
}
```

### Browsing the API reference (Doxygen)

Every class, method, member, struct, and constant in `Rosmaster.hpp` is documented
in-source with Doxygen comments. The generated HTML site gives you a searchable API
reference, clickable annotated source, and call/collaboration graphs.

**Step 1 — install the tools** (one time). `doxygen` is required; `graphviz` is
optional (it enables the diagrams):

| Platform            | Command                                          |
| ------------------- | ------------------------------------------------ |
| Debian / Ubuntu / WSL | `sudo apt-get install doxygen graphviz`        |
| Fedora / RHEL       | `sudo dnf install doxygen graphviz`              |
| Arch                | `sudo pacman -S doxygen graphviz`                |
| macOS (Homebrew)    | `brew install doxygen graphviz`                  |

**Step 2 — generate and view.** The helper script does everything in one command:

```bash
./scripts/build_docs.sh --serve      # generate, serve on http://localhost:8000, open your browser
# variants:
./scripts/build_docs.sh --serve 9000 # use a different port
./scripts/build_docs.sh --open       # generate + open the static file directly
./scripts/build_docs.sh              # just generate into docs/html/
doxygen Doxyfile                     # equivalent raw command (no serving)
```

Then browse to **http://localhost:8000** and open **Classes → `Rosmaster`**. Use
*Files → Rosmaster.hpp → “Go to the source code”* for the clickable annotated source.
Press `Ctrl+C` in the terminal to stop the server.

**Viewing on WSL 2.** `--serve` is the most reliable path: WSL forwards `localhost`
to Windows, so `http://localhost:8000` just works in your Windows browser. If it does
not, use the WSL IP printed by the script (e.g. `http://172.x.x.x:8000`). To open the
file without a server instead:

```bash
explorer.exe "$(wslpath -w docs/html/index.html)"
# or paste this into the Windows Explorer address bar:
#   \\wsl.localhost\Ubuntu\home\<you>\rosmater_cpp\docs\html\index.html
```

The Doxygen config is [`Doxyfile`](Doxyfile); the landing page is
[`docs/mainpage.md`](docs/mainpage.md); developer/build conventions are in
[CONTRIBUTING.md](CONTRIBUTING.md). The generated `docs/html/` is git-ignored — it is
a build artifact you regenerate any time with the command above.

---

## 4. Hardware setup & serial permissions

- Connect the Yahboom controller by USB. It enumerates as `/dev/ttyUSB0`
  (CH340) or `/dev/ttyUSB0`/`/dev/ttyACM0` (CP210x). Confirm with `dmesg | tail`.
- The port opens at **115200 baud**, hard-coded (this is the Yahboom firmware
  rate).
- If `open()` fails with `Permission denied`, add your user to the `dialout`
  group and re-log:
  ```bash
  sudo usermod -aG dialout "$USER"
  ```
- For a stable device name across reboots, add a udev rule mapping the adapter's
  serial to e.g. `/dev/myserial`, and pass that path to the constructor.
- **ModemManager** can steal the port on plug-in. The driver claims it
  exclusively (`TIOCEXCL`), but if you see intermittent open failures, mask
  ModemManager: `sudo systemctl mask ModemManager`.

**Calibration precondition:** the wheels must be **free to spin** (robot up on a
stand or on blocks). Calibration drives the motors open-loop and measures ticks.

---

## 5. Thread architecture

Two background threads run alongside the application thread. All cross-thread
data is lock-free via `std::atomic` (except the PID gains, which use a small
mutex).

```
┌─────────────────────────────────────────────────────────────┐
│  Application thread (ros2_control write()/read(), or main()) │
│                                                              │
│  bot.set_motor(s1,s2,s3,s4)  ──writes──▶  target_[i]         │
│  bot.get_pid_measured()      ──reads───▶  pid_measured_[i]   │
│  bot.get_motor_encoder(...)  ──reads───▶  encoder_mX_        │
└──────────────────────────┬───────────────────────────────────┘
                           │  atomics (lock-free)
          ┌────────────────┴────────────────┐
          │                                  │
┌─────────▼──────────┐            ┌──────────▼───────────┐
│  receiveLoop()      │            │  pidLoop()            │
│  (recv_thread_)     │            │  (pid_thread_)        │
│                     │            │                       │
│  UART → parse →     │──atomic──▶ │  read encoder_mX_ →   │
│  encoder_mX_,       │            │  velocity → PID →     │
│  battery, IMU, ...  │            │  writeMotorRaw() →    │
│                     │            │  pid_measured_[i]     │
└─────────────────────┘            └───────────────────────┘
```

| Thread         | Role                                                          | Rate                          |
| -------------- | ------------------------------------------------------------- | ----------------------------- |
| `recv_thread_` | Read UART, parse frames, update sensor atomics.               | Event-driven (~24.4 Hz encoder) |
| `pid_thread_`  | Velocity control loop; write PWM commands.                    | Fixed 25 Hz (`kPidHz`)        |

The PID thread only exists after `enable_pid_control()`. Without it, `set_motor()`
writes raw PWM directly (legacy mode, identical to the original Python driver).

### Concurrency rules

| Data              | Producer               | Consumer          | Protection                    |
| ----------------- | ---------------------- | ----------------- | ----------------------------- |
| `encoder_mX_`     | `receiveLoop`          | `pidLoop`, app    | `std::atomic<int>`            |
| `target_[i]`      | app (`set_motor`)      | `pidLoop`         | `std::atomic<double>`         |
| `pid_measured_[i]`| `pidLoop`              | app               | `std::atomic<double>`         |
| `pid_gains_`      | app (`set_pid_gains`)  | `pidLoop`         | `std::mutex pid_gains_mutex_` |
| `motor_state_[i]` | `pidLoop`              | `pidLoop`         | thread-exclusive, no lock     |
| `enc_history_`    | `pidLoop`              | `pidLoop`         | thread-exclusive, no lock     |

### Guaranteed startup order

```
Rosmaster()                  → open serial port
create_receive_threading()   → start recv_thread_
set_auto_report_state(true)  → enable Yahboom auto-report
[wait: encoder_received_ == true]
calibrate_motor_scales()     → measure scales (PID off)
enable_pid_control()         → start pid_thread_
```

### Guaranteed shutdown order (destructor)

```
disable_pid_control()  → pid_running_=false → join(pid_thread_)
uart_running_=false    → join(recv_thread_)
ser_.close()           → release the fd
```

The PID thread is joined **before** the receive thread, because `pidLoop()`
reads encoder atomics populated by `receiveLoop()`. Reversing the order would
let the PID thread read stale values or a closed fd.

---

## 6. Yahboom UART protocol

### Incoming frame (auto-report)

```
Byte 0     : 0xFF            HEAD
Byte 1     : 0xFB            DEVICE_ID − 1  (0xFC − 1)
Byte 2     : ext_len         length from ext_type onward, checksum included
Byte 3     : ext_type        function id (encoder=0x0D, IMU=0x0C, battery in 0x0A, …)
Byte 4..N-2: data
Byte N-1   : checksum        = (ext_len + ext_type + Σ data) % 256
```

### Outgoing frame (command)

```
Byte 0     : 0xFF            HEAD
Byte 1     : 0xFC            DEVICE_ID
Byte 2     : length          = total size − 1 (checksum excluded)
Byte 3     : function
Byte 4..N-2: parameters
Byte N-1   : checksum        = (COMPLEMENT + Σ bytes[0..N-2]) % 256
                              COMPLEMENT = 257 − DEVICE_ID = 5
```

### Encoder frame (0x0D)

16 data bytes: four little-endian `int32_t`, one per motor (FL, FR, RL, RR).
The counters are **cumulative** and never reset while powered. Wrap-around is
handled by modulo-2³² arithmetic on the delta:

```cpp
const int32_t delta = static_cast<int32_t>(
    static_cast<uint32_t>(enc_now) - static_cast<uint32_t>(enc_old));
```

This is exact as long as real travel between two reads stays below 2³¹ ticks —
never a concern at ~10 000 ticks/s.

### Auto-report rate

Measured on a Pi 4B with the X3: **~24.4 Hz** (median interval 41 ms). The rate
is fixed by firmware and not configurable. The 24.4 vs 25 Hz mismatch is why the
PID uses a sliding window and real timestamps (see §7).

---

## 7. The software PID loop

### Constants (`Rosmaster.hpp`)

```cpp
static constexpr double kDerivAlpha = 0.8;   // derivative EMA factor
static constexpr int    kPidHz      = 25;    // PID rate
static constexpr int    kVelWindow  = 10;    // velocity ring-buffer depth (~400 ms)
```

### Control law (v8)

Per motor `i`, each PID tick:

```
measured[i] = signed(delta_ticks[i]) / window_dt / scale[i] × 100     [%]
error       = target[i] − measured[i]                                 [signed %]

err_filt[i] = α·err_filt_prev + (1−α)·error                           [EMA of error]
d_term      = kd · (err_filt − err_filt_prev) / dt

ff_term     = kS[i]·sign(target) + kV[i]·target      (feedforward, §8)
raw_cmd     = ff_term + kp·error + ki·∫ + d_term

anti-windup : integrate if |raw_cmd| < 100  OR  sign(error) ≠ sign(∫)
cmd[i]      = clamp(raw_cmd, −100, 100)
```

Key points, each of which cost a debugging cycle to get right (see §10):

- **Feedforward carries the steady-state command.** At zero error the output
  equals the feedforward prediction; the integral only trims residual bias
  (friction, inter-motor imbalance), never the bulk of the command.
- **The derivative filters `error`, not `measured`.** Filtering `error` makes
  `d_term` sign-correct in both rotation directions and additive — a step in
  `target` yields a positive D impulse that *helps* acceleration. Filtering
  `measured` (as an earlier version did) inverted the sign at direction reversal.
- **`d_term` divides by `dt`, but velocity divides by `window_dt`.** The
  measurement is a windowed average (~400 ms); the derivative is a per-tick
  difference of the filtered error (~40 ms). Dividing the derivative by `dt` and
  the velocity by `window_dt` keeps both dimensionally correct.

### Sliding-window velocity estimate

Encoder packets arrive at ~24.4 Hz while the PID runs at 25 Hz. Estimating
velocity over a single `dt` would beat badly (ratio 24.4/25 ≈ 0.976, non-integer)
because an iteration captures either 0 or 1 new packet.

The loop keeps the last `kVelWindow=10` encoder snapshots in a ring buffer and
computes velocity from the delta between *now* and the *oldest* snapshot, over
`window_dt` derived from **real timestamps** (not `n·dt_nominal`). Real
timestamps remove the ~2.5% systematic bias the nominal-`dt` method had.

```
oldest_idx = enc_history_full_ ? (enc_history_idx_ + 1) % kVelWindow : 0
delta[i]   = enc_now[i] − enc_history_[oldest_idx][i]
```

> **Note on `oldest_idx`.** The write happens at `idx` *before* the increment,
> so after the increment `idx` points at the slot written on the previous full
> lap — the oldest. Using `idx` without `+1` regresses `delta` to zero on the
> first wrap, collapsing `measured` to 0 exactly one window (`kVelWindow·dt`)
> after start. This exact symptom is called out in §17.

### Anti-windup

The integral updates **only if** the output is unsaturated (`|raw_cmd| < 100`)
**or** the integral is winding *down* toward zero (`error` and `∫` have opposite
signs). This is pre-integration anti-windup — no post-hoc rollback, so the
integrator state and the command actually sent never disagree. The
"winding-down" clause is what lets a forward-charged integral discharge during a
direction reversal even while the output is saturated in reverse (see §10, v7).

### Derivative EMA time constant

With `kDerivAlpha = 0.8` and `dt = 40 ms`:

```
τ = α / (1−α) · dt = 0.8 / 0.2 · 0.04 = 160 ms
```

---

## 8. Motor feedforward (kS/kV)

### Model

```
pwm_ff = kS[i] · sign(target) + kV[i] · target
```

- **`kS`** (per motor): the minimum PWM % needed to overcome static friction and
  start moving — the *dead zone*.
- **`kV`** (per motor): the PWM-per-%-of-speed slope in the linear region, under
  whatever load was present during calibration.
- At `target == 0` exactly, `ff_term = 0` (no `sign(0)·kS`) — the robot does not
  creep against its own friction when commanded to stop.
- **Backwards compatible:** until `calibrate_feedforward()` runs *and*
  `enable_feedforward(true)` is set, the effective gains are `{kS=0, kV=1}`, i.e.
  `ff_term = target` — exactly the pre-feedforward (v7) behaviour.

The PID then only corrects the **residual** around the feedforward prediction,
so `kp/ki/kd` are typically **smaller** than in the pre-feedforward tuning.

### Automatic calibration — `calibrate_feedforward()`

```cpp
double calibrate_feedforward(int throttle_min_pct = 5,
                             int throttle_max_pct = 70,
                             int step_pct         = 3,
                             int settle_ms        = 250,
                             int sample_ms        = 300);
```

Per motor (all four are driven together to simplify sequencing; each keeps its
own encoder measurement):

1. Sweep PWM from `throttle_min_pct` to `throttle_max_pct` in `step_pct` steps.
2. At each step, wait `settle_ms` (mechanical transient), then count ticks over
   `sample_ms`.
3. **`kS[i]`** = the first PWM step where the motor crosses a motion threshold
   (8 ticks over the sample window — above electrical/encoder noise).
4. **`kV[i]`** = least-squares fit of `PWM = kS + kV·speed` over the linear-region
   points. Guard rails:
   - degenerate matrix (all points at one speed, or a single point) → fall back
     to a simple ratio rather than an unstable solve;
   - if the fit's intercept is far below the *observed* dead zone → keep the
     observed value (more reliable than extrapolating the fit);
   - `kV` clamped `≥ 0.1` (a non-positive slope would produce feedforward
     opposing motion);
   - `kS` clamped to `[0, 50]`%.

**Preconditions:** PID off, wheels free to spin, auto-report on.

**After calibration you must enable it explicitly:**

```cpp
bot.calibrate_feedforward();
bot.enable_feedforward(true);   // without this, ff_term = target (v7 behaviour)
```

### Manual gains (skip auto-calibration)

```cpp
bot.set_feedforward_gains(/*motor_index=*/0, /*kS=*/8.0, /*kV=*/0.6);
double kS, kV;
bot.get_feedforward_gains(0, kS, kV);
```

---

## 9. Calibration

Mandatory order (each step assumes PID off and wheels free to spin):

```
1. create_receive_threading()
2. set_auto_report_state(true, true)
3. wait for first packets (battery_voltage != 0)
4. calibrate_motor_scales(...)   → per-motor ticks/s scales
5. calibrate_feedforward(...)    → per-motor kS/kV
6. enable_feedforward(true)
7. enable_pid_control(kp, ki, kd, scale_global)
```

### Why calibrate?

`% command → ticks/s` depends on the gearmotor variant (the NFP-JGB37-520 ships
in several reductions). Without calibration the feedforward is wrong and the
integral has to compensate on its own.

### `calibrate_motor_scales()` — the recommended method

```cpp
double scale_global = bot.calibrate_motor_scales(
    /*throttle_pct=*/60, /*duration_ms=*/800, /*n_runs=*/5,
    /*warmup_ms=*/3000,  /*use_per_motor=*/true);
```

Algorithm:

1. Optional thermal **warmup** (60% for `warmup_ms`, default 3 s) — DC coil
   resistance drifts in the first seconds.
2. `n_runs` independent runs at `throttle_pct`% for `duration_ms` each.
3. Each run: a *stable read* before (two reads 25 ms apart that agree) → motor
   pulse → stable read after.
4. **Inter-motor ratios** (`ticks[i] / run_mean`) are computed per run and
   averaged across runs — robust to global thermal drift (±5%), because the
   ratios stay stable even as the absolute value drifts.
5. **Global scale** = trimmed mean of the per-run global samples (min + max
   dropped when `n_runs ≥ 3`).
6. **Per-motor scale** = global scale × that motor's mean ratio.
7. A per-motor coefficient of variation (CV) is printed; `CV > 1.5%` warns of an
   irregular motor (dry bearing, marginal wiring).

If `use_per_motor=true`, `motor_scale_[i]` is populated and used by `pidLoop()`.
Otherwise a single global scale is used for all motors.

### `set_motor_scales()` — inject a pre-computed calibration

If calibration is done offline (e.g. a config file):

```cpp
std::array<double,4> scales = {10634.9, 8892.1, 9160.6, 9995.7};
double global = 9677.95;
bot.set_motor_scales(scales, global);   // also raises pid_scale_calibrated_
```

This populates `motor_scale_[]` and `ticks_per_second_at_100pct_`, and clears the
"using default scale" warning at `enable_pid_control()`.

---

## 10. The development story — PID v2 → v8

This section documents **what had to be done** to reach a stable loop — the bugs
found, the fixes applied, and *why*. It is the accumulated hard-won knowledge of
the project. Read it before changing the control law.

### v2 → v3 — Foundations
- Split gains and state into dedicated structs.
- Measure **real `dt`** instead of assuming it constant.
- First derivative filter, 32-bit encoder wrap handling, UART-state checks,
  explicit command saturation.

### v3 → v4 — Anti-windup and startup guards
- **Pre-integration anti-windup:** integrate *after* the saturation check, not
  before with a rollback. The old integrate-then-undo introduced a one-cycle
  jolt.
- **`encoder_received_` flag:** the PID may not start before the first valid
  encoder packet. Otherwise the first cycle computes velocity from
  uninitialised (often `0`) sensor data and produces a huge phantom velocity.
- `calibrate_pid_scale()` rejects durations `< 100 ms` (too few ticks for a
  reliable estimate).

### v4 → v5 — Robustness fixes

| Bug                                                              | Fix                                          |
| --------------------------------------------------------------- | -------------------------------------------- |
| Typo `deriv_filtered` → `derivative_filtered`                   | Rename                                       |
| `calibrate_pid_scale()` could silently return ≤ 0              | Explicit guard + exception                   |
| Measured velocity unbounded (glitch spike possible)             | Clamp to `[-200, 200]`%                      |
| `set_motor()` did not clamp targets while PID active            | Clamp `[-100, 100]` before storing to target |
| Division by a possibly-zero `scale` → `NaN`                   | Guard `std::max(scale, 1.0)`                  |
| `encoder_received_` not reset after a UART reconnect            | Reset in `receiveLoop()` each session        |

### v5 → v6 — Timing precision and multi-motor calibration
- **Window-bias fixed.** Encoder packets arrive at ~24.4 Hz, not 25 Hz. Counting
  `n_samples × dt_nominal` introduced a ~2.5% systematic bias. v6 stores a real
  timestamp per sample (`EncSlot{enc, ts_us}`) and derives `window_dt` from real
  timestamps — the bias disappears regardless of the encoder/PID rate ratio.
- **Derivative reworked** (interim): EMA on `measured` rather than
  `(error − prev_error)`. *This was itself wrong and is corrected in v7 — it
  caused the reversal instability.*
- **`calibrate_motor_scales()`** promoted from a standalone `main()` into a class
  method (ratio-based, multi-run). `calibrate_pid_scale_at()` kept as a
  single-run compatibility wrapper.
- `kVelWindow = 10` (~400 ms) confirmed empirically.

### v6 → v7 — Fixing direction-reversal instability

Three sign bugs, all surfaced by testing a direction reversal (+20% → −20%):

**Bug 1 — wrong D sign at reversal.** v6 filtered `measured`, not `error`. On
reversal, `measured` drops sharply, and the D term — subtracted in
`raw_cmd = target + kp·error + ki·∫ − d_term` — went *positive* just when
braking was needed. It pushed *with* the overshoot instead of damping it.

*Fix:* filter `error` (`target − measured`) and make D additive:

```
err_filt = α·err_filt_prev + (1−α)·error
d_term   = kd · (err_filt − err_filt_prev) / dt
raw_cmd  = target + kp·error + ki·∫ + d_term
```

Now D is sign-correct in both directions, and a setpoint step produces a positive
D impulse that *aids* acceleration toward the new target.

**Bug 2 — integral stuck during reversal.** The original anti-windup only
integrated when `|raw_cmd| < 100`. On reversal the command saturates
(`raw_cmd = −100`) for the entire deceleration phase, so the forward-charged
integral stayed pinned at its positive value and kept pushing the wrong way for
hundreds of milliseconds.

*Fix:* also integrate when `error` and `∫` have opposite signs (the integral
discharges toward zero — always safe):

```cpp
const bool not_saturated = std::abs(raw_cmd) < 100.0;
const bool winding_down  = (error * integral) < 0.0;
if (not_saturated || winding_down) integral += error * dt;
```

**Bug 3 — cleaned up `MotorPidState`.** The now-obsolete `prev_error` field
(v5) was removed. `MotorPidState` now holds only `integral`, `measured_filtered`
(for `get_pid_measured()`), and `error_filtered` (the D term).

> **Empirical note.** With `kd = 0.0` (the on-bench setting for a long time),
> fixes 1 and 3 are transparent — only fix 2 (anti-windup) was visible. The
> corrected D term becomes usable (`kd ≈ 0.02–0.05`) to damp the mechanical jerk
> intrinsic to mecanum wheels, without reintroducing reversal instability.

### v7 → v8 — Motor feedforward (kS + kV)

**Trigger.** The measured Yahboom motor datasheet:

| Condition        | Speed        | Current | Torque   |
| ---------------- | -----------: | ------: | -------: |
| No load          | 333 rpm      | 0.15 A  | —        |
| Nominal load     | 250 rpm      | 0.65 A  | 1 kg·cm  |
| Stall            | 0 rpm        | 2.4 A   | 4 kg·cm  |

The motor loses ~25% speed under nominal load (normal brushed-DC physics) and
has a dead zone before it starts (static friction). v7 treated `target` as an
implicit 1:1 feedforward, which assumes a perfectly linear, frictionless
motor — false in practice, especially at low speed.

**Fix:** an explicit feedforward model (see §8), so the PID corrects only the
residual. Backwards compatible until `enable_feedforward(true)`.

---

## 11. Serial-port lifecycle hardening (FIX-1 → FIX-7)

**Context.** A bug once required **rebooting the Raspberry Pi** after every
e-stop followed by a Yahboom power cycle. Root cause: the kernel tty layer
(CH340 / CP210x) keeps per-fd state; on USB power loss the kernel raises
`VHANGUP`, cascading into failures unless handled explicitly.

| #  | Problem                                                                          | Fix                                                                    |
| -- | ------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| 1  | `open()` blocks forever on a VHANGUP'd session                                  | `O_NONBLOCK` on open, cleared immediately afterward via `fcntl`        |
| 2  | `tcsetattr()` silently ignored on a just-reappeared fd                          | `tcflush(TCIOFLUSH)` **before** `tcgetattr()`                          |
| 3  | Another process (udev, ModemManager) grabs the port                             | `TIOCEXCL` right after `open()`                                        |
| 4  | Kernel keeps an exclusive lock after a malformed `close()`                       | `TIOCNXCL` before `close()`                                            |
| 5  | Thread/fd race in the destructor (`EBADF` if the port closes mid-`read()`)      | Strict order: `uart_running_=false` → `join()` → `ser_.close()`       |
| 6  | `EIO` unhandled in `receiveLoop()` left the fd open kernel-side                 | Explicit `ser_.close()` on any fatal error (`EIO`/`ENOTTY`/`EBADF`)    |
| 7  | DTR/RTS dropped at `close()`, making `/dev/ttyUSBx` disappear                    | `HUPCL` disabled in `c_cflag`                                          |

A **silence watchdog** (`kSilenceTimeoutMs = 1500`) also forces a clean close if
no valid frame arrives for 1.5 s — useful when a device disconnects without a
hard `EIO` (some USB hubs). `is_running()` returns `uart_running_`, which flips
to `false` on any fatal serial error and closes the fd cleanly, letting the next
`open()` fully re-enumerate the adapter.

---

## 12. Motor model & physical limits

Before blaming software for imprecision, know what is **physically** limiting on
this platform:

1. **The brushed DC motor** dominates at mid/high speed: ~25% speed loss under
   load, torque sensitive to current, non-linear PWM→speed (dead zone +
   friction). No software refinement makes it behave like an industrial servo
   (hardware current loop, FOC) — that is a hardware change, not code. The
   feedforward (v8) targets exactly this term.
2. **Encoder resolution** dominates at **very low speed** (fine manoeuvres, just
   after a reversal, approaching a near-zero setpoint): with few ticks per ~400 ms
   window, `measured` becomes a coarse staircase regardless of motor or PID
   quality. A time-between-pulses estimator would help here — see §18.
3. **The current loop** lives in the Yahboom firmware, invisible to ROS 2. The
   25 Hz software PID inherits all the noise and jerk produced below it with no
   visibility.

---

## 13. Test bench (standalone example)

The project previously shipped a `Rosmaster.cpp` test bench. It was removed to
keep the repo to a single header, but the full automatic sequence it ran is
reproduced here — copy it into `main.cpp` to reproduce the on-bench validation.

**Sequence:** open port + receive thread → auto-report → wait for first packet →
`calibrate_motor_scales` → `calibrate_feedforward` → enable feedforward → enable
PID → 50 s convergence test at −20% (logging every 100 ms) → clean stop.

```cpp
#include "Rosmaster.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <iomanip>

int main() {
    try {
        Rosmaster bot(1, "/dev/ttyUSB0", 0.002, false);
        bot.create_receive_threading();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bot.set_auto_report_state(true, true);

        // Robust wait for the first Yahboom packet.
        auto t0 = std::chrono::steady_clock::now();
        while (bot.get_battery_voltage() == 0.0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed > 2000)
                throw std::runtime_error("Timeout: no Yahboom packet after 2s — "
                                         "check power and USB cable");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "Battery: " << bot.get_battery_voltage() << " V\n";

        // Stage 1/2 — speed-scale calibration (multi-run, warmup included).
        const double scale_global = bot.calibrate_motor_scales(
            /*throttle_pct=*/60, /*duration_ms=*/800, /*n_runs=*/5,
            /*warmup_ms=*/3000,  /*use_per_motor=*/true);

        // Stage 2/2 — feedforward calibration (kS/kV per motor).
        const double avg_dead_zone = bot.calibrate_feedforward(
            /*throttle_min_pct=*/5, /*throttle_max_pct=*/70, /*step_pct=*/3,
            /*settle_ms=*/250, /*sample_ms=*/300);
        std::cout << "Mean dead zone: " << avg_dead_zone << "% PWM\n";

        for (int i = 0; i < 4; ++i) {
            double kS, kV;
            bot.get_feedforward_gains(i, kS, kV);
            std::cout << "  M" << (i + 1) << " : kS=" << std::fixed
                      << std::setprecision(2) << kS << "  kV=" << kV << "\n";
        }
        bot.enable_feedforward(true);

        // Enable PID. With feedforward active the PID only trims the residual,
        // so gains are smaller than the pre-feedforward tuning. kd=0.0 kept:
        // the mecanum oscillation is mechanical, not a software instability.
        bot.enable_pid_control(0.3, 0.05, 0.0, scale_global);

        // Convergence test: 50 s forward at -20%, log every 100 ms.
        bot.set_motor(-20.0, -20.0, -20.0, -20.0);
        int prev[4] = {0, 0, 0, 0};
        bot.get_motor_encoder(prev[0], prev[1], prev[2], prev[3]);

        for (int i = 0; i < 500; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int cur[4];
            bot.get_motor_encoder(cur[0], cur[1], cur[2], cur[3]);
            const auto meas = bot.get_pid_measured();

            std::cout << std::setw(6) << (i + 1) * 100;
            for (int j = 0; j < 4; ++j) {
                const int32_t delta = static_cast<int32_t>(
                    static_cast<uint32_t>(cur[j]) - static_cast<uint32_t>(prev[j]));
                std::cout << std::setw(8) << delta;
                prev[j] = cur[j];
            }
            std::cout << std::fixed << std::setprecision(1) << "  pid%:";
            for (int j = 0; j < 4; ++j) std::cout << std::setw(7) << meas[j];
            std::cout << "\n";
        }

        bot.set_motor(0.0, 0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));  // ~2 PID cycles
        bot.disable_pid_control();
    } catch (const std::exception & e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
```

Build & run:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pthread main.cpp -o main_test
./main_test
```

Reading the log:

```
  t(ms)  raw_M1  raw_M2  raw_M3  raw_M4  pid%:     M1     M2     M3     M4
    100    -130    -128    -131    -129        -19.8  -19.6  -19.9  -19.7
```

- `raw_Mx` — raw encoder-tick delta over the 100 ms window (signed).
- `pid%` (`get_pid_measured()`) — the velocity the control loop actually sees, in
  % of the calibrated scale. Compare it to the target (`-20.0` here).

> A visible oscillation in the `raw_Mx` columns while `pid%` stays flat is a
> **display artifact**, not real robot oscillation — see §17.

---

## 14. ros2_control integration

In a `ros2_control` architecture, a hardware interface implements
`hardware_interface::SystemInterface` and bridges ROS 2 to the driver:

```
ros2_control_node
  └─ MecaMateSystemHardware  (SystemInterface)
       └─ Rosmaster bot_
            ├─ recv_thread_   (encoder → atomics)
            └─ pid_thread_    (measure → command)
```

State interfaces (read): `joint_i/velocity ← get_pid_measured()[i]/100 · max_vel_rad_s`.
Command interfaces (write): `cmd_pct[i] = cmd_rad_s / max_vel_rad_s · 100`.

### Skeleton

```cpp
// MecaMateSystemHardware.hpp
#pragma once
#include "hardware_interface/system_interface.hpp"
#include "Rosmaster.hpp"
#include <memory>

class MecaMateSystemHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    std::vector<hardware_interface::StateInterface>  export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read (const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
    std::unique_ptr<Rosmaster> bot_;
    std::array<double, 4> vel_state_{};   // measured velocities [rad/s]
    std::array<double, 4> vel_cmd_{};     // commanded velocities [rad/s]
    double max_vel_rad_s_{10.0};          // speed at 100% [rad/s] — measure it
    double scale_global_{9677.0};         // ticks/s at 100% — from calibration
};
```

```cpp
// MecaMateSystemHardware.cpp (essentials)
CallbackReturn MecaMateSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
    const std::string port = info_.hardware_parameters.at("serial_port");
    bot_ = std::make_unique<Rosmaster>(1, port, 0.002, false);
    bot_->create_receive_threading();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bot_->set_auto_report_state(true, true);

    auto t0 = std::chrono::steady_clock::now();
    while (bot_->get_battery_voltage() == 0.0) {
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(3)) {
            RCLCPP_ERROR(rclcpp::get_logger("MecaMate"), "Timeout: no Yahboom packet");
            return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Inject pre-calibrated scales (or call calibrate_motor_scales() if wheels are free).
    std::array<double,4> per_motor = {
        std::stod(info_.hardware_parameters.at("scale_m1")),
        std::stod(info_.hardware_parameters.at("scale_m2")),
        std::stod(info_.hardware_parameters.at("scale_m3")),
        std::stod(info_.hardware_parameters.at("scale_m4"))};
    bot_->set_motor_scales(per_motor, scale_global_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecaMateSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
    bot_->enable_pid_control(0.6, 0.1, 0.0, scale_global_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecaMateSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    bot_->set_motor(0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bot_->disable_pid_control();
    return CallbackReturn::SUCCESS;
}

return_type MecaMateSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    const auto meas = bot_->get_pid_measured();
    for (size_t i = 0; i < 4; ++i) vel_state_[i] = meas[i] / 100.0 * max_vel_rad_s_;
    return return_type::OK;
}

return_type MecaMateSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    bot_->set_motor(vel_cmd_[0] / max_vel_rad_s_ * 100.0,
                    vel_cmd_[1] / max_vel_rad_s_ * 100.0,
                    vel_cmd_[2] / max_vel_rad_s_ * 100.0,
                    vel_cmd_[3] / max_vel_rad_s_ * 100.0);
    return return_type::OK;
}
```

### URDF (ros2_control xacro)

```xml
<ros2_control name="mecamate" type="system">
  <hardware>
    <plugin>mecamate_hardware/MecaMateSystemHardware</plugin>
    <param name="serial_port">/dev/myserial</param>
    <param name="max_velocity_rad_s">10.0</param>
    <param name="ticks_per_sec_at_100pct">9677.0</param>
    <param name="scale_m1">10634.9</param>
    <param name="scale_m2">8892.1</param>
    <param name="scale_m3">9160.6</param>
    <param name="scale_m4">9995.7</param>
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- front_right / rear_left / rear_right identical -->
</ros2_control>
```

### Reconnect handling

```cpp
if (!bot_->is_running()) {
    // The port received EIO (Yahboom powered off).
    // Recreate bot_, redo on_configure(), on_activate().
}
```

`receiveLoop()` flips `uart_running_` to `false` and cleanly closes the fd on any
`EIO`/`ENOTTY`/`EBADF`, which triggers a full CH340/CP210x re-enumeration on the
next `open()`.

---

## 15. Tuning the PID gains

### Before feedforward (v7 and earlier)

`raw_cmd = target + kp·error + ki·∫ + d_term` — `target` already acted as an
implicit 1:1 feedforward, so `kp/ki` only corrected the residual off a perfect
linear model. On-bench values: **`kp=0.6, ki=0.1, kd=0.0`**.

### With feedforward active (v8)

`raw_cmd = ff_term + kp·error + ki·∫ + d_term` — `ff_term` now carries the
predictive command, so `kp/ki` can be **smaller**. Starting point used on bench:
**`kp=0.3, ki=0.05, kd=0.0`** — a starting point to validate, not calibrated
values.

### Step-by-step on the ground

1. **Rate & window.** Do not change `kPidHz=25` or `kVelWindow=10` without
   recomputing:
   ```
   kVelWindow ≥ ceil(2 × packet_period_ms / (1000 / kPidHz))
              = ceil(2 × 41 / 40) = ceil(2.05) = 3   (theoretical minimum)
   ```
   10 is deliberately generous to smooth scheduling jitter. Below ~8 risks
   reintroducing aliasing.
2. **`kp` alone (ki=0, kd=0).** Raise until the step response is fast without
   visible oscillation on `get_pid_measured()`. Typically 0.4–1.0.
3. **`ki` for static bias.** With `kp` fixed, raise `ki` slowly. Too high →
   slow overshoot as the integral takes seconds to hit anti-windup. Typical
   0.05–0.2.
4. **`kd` if needed.** On a smooth surface `kd=0` is usually enough. Since the
   v7 fix (D on `error_filtered`), D is usable at `kd ≈ 0.02–0.05` to damp
   mechanical jerk. Stay below ~0.05 to avoid amplifying encoder quantisation
   noise.

### Per-motor overrides

If one motor behaves differently (high CV, very different dead zone):

```cpp
bot.set_motor_pid_gains(/*motor_index=*/2, kp, ki, kd, /*override=*/true);
bot.reset_motor_pid_gains(2);   // back to the global gains
```

### Recomputing `kVelWindow` if `kPidHz` changes

For the X3 (packet period ≈ 41 ms):

| kPidHz | dt (ms) | min kVelWindow | recommended |
| ------ | ------- | -------------- | ----------- |
| 100    | 10      | 9              | 15          |
| 50     | 20      | 5              | 10          |
| 25     | 40      | 3              | 10          |
| 10     | 100     | 1              | 5           |

---

## 16. Public API reference

### Construction & lifecycle

```cpp
Rosmaster bot(int car_type, const std::string& port,
              double cmd_delay_s = 0.002, bool debug = false);
// car_type: 1=X3, 2=X3_PLUS, 4=X1, 5=R2
```

| Method                                      | Role                                          |
| ------------------------------------------- | --------------------------------------------- |
| `create_receive_threading()`                | Start the UART receive thread.                |
| `set_auto_report_state(enable, forever)`    | Enable firmware auto-report of sensors/encoders. |
| `is_running()`                              | Receive-thread / serial health.               |

### Calibration

| Method                                                                            | Returns                       |
| --------------------------------------------------------------------------------- | ----------------------------- |
| `calibrate_motor_scales(throttle_pct, duration_ms, n_runs, warmup_ms, use_per_motor)` | global scale (ticks/s)     |
| `calibrate_pid_scale_at(throttle_pct, duration_ms, use_per_motor)`                | compat. wrapper (`n_runs=1`)  |
| `calibrate_pid_scale(duration_ms)`                                                | global scale, simple/legacy   |
| `calibrate_feedforward(min_pct, max_pct, step_pct, settle_ms, sample_ms)`         | mean dead zone (%)            |
| `set_motor_scales(scales, global)`                                                | inject scales manually        |
| `set_feedforward_gains(motor_index, kS, kV)` / `get_feedforward_gains(...)`        | set / read kS,kV              |

### PID control

| Method                                                | Role                                        |
| ----------------------------------------------------- | ------------------------------------------- |
| `enable_pid_control(kp, ki, kd, ticks_per_sec)`       | start the PID thread @ 25 Hz                |
| `disable_pid_control()`                               | stop the PID thread, reset state            |
| `set_pid_gains(kp, ki, kd)`                           | global gains (motors without override)      |
| `set_motor_pid_gains(motor_index, kp, ki, kd, override)` | per-motor gains                          |
| `reset_motor_pid_gains(motor_index)`                  | back to global gains for that motor         |
| `enable_feedforward(enable)`                          | enable/disable the feedforward term         |
| `get_pid_measured()`                                  | current measured speed (%) per motor        |

> `enable_pid_control()` header defaults are `kp=1.8, ki=0.4, kd=0.05,
> ticks_per_sec=1326.0` — historical bare-metal-100 Hz values. **Always pass your
> calibrated scale and validated gains explicitly**; do not rely on the defaults.

### Motor command

| Method                                        | Behaviour                                                |
| --------------------------------------------- | -------------------------------------------------------- |
| `set_motor(s1, s2, s3, s4)`                   | PID active → store targets; else → direct PWM. `[-100,100]%`. |
| `set_motor_with_compensation(s1..s4)`         | apply slope compensation, then `set_motor`.              |
| `set_car_motion(vx, vy, vz)`                  | high-level kinematic command (Yahboom firmware).         |

### Sensors

```cpp
int m1, m2, m3, m4; bot.get_motor_encoder(m1, m2, m3, m4);   // cumulative ticks
std::array<double,4> vel = bot.get_pid_measured();           // %, lock-free ({0,…} if PID off)
double v = bot.get_battery_voltage();                        // Volts
double roll, pitch, yaw;
bot.get_imu_attitude_data(roll, pitch, yaw, /*to_angle=*/true);  // degrees
```

Also available: `get_accelerometer_data`, `get_gyroscope_data`,
`get_magnetometer_data`, `get_motion_data`, plus servo / LED / buzzer / arm
controls ported from the Python driver.

### Slope compensation (optional)

`configure_slope_compensation(...)`, `set_max_pwm_flat(pwm)`,
`update_pitch(pitch_rad)`, `set_motor_with_compensation(...)` — reserve PWM
headroom and bias command on an incline. Off by default (`max_pwm_flat_=70`).

---

## 17. Troubleshooting

### `pid%` drops to 0 after ~400 ms
Symptom: `pid%` columns are non-zero during ramp-up, then snap to 0.
Cause: `oldest_idx` miscomputed in `pidLoop()` — happens exactly at the first
full ring-buffer wrap (`kVelWindow × dt ≈ 400 ms`). Ensure the code uses
`(enc_history_idx_ + 1) % kVelWindow`, not `enc_history_idx_`.

### `raw_Mx` oscillates but `pid%` is flat
Cause: aliasing between the bench display window (100 ms) and the encoder packet
period (~41 ms): 100/41 ≈ 2.44 packets/window — non-integer → beat. **Not real
robot oscillation.** Confirm by checking `pid%` (stable ±2% → display artifact).
To verify, set the display window to ~820 ms (~20 packets, near-integer).

### Serial port hangs on the second `open()` after a Yahboom power cut
Cause: a VHANGUP tty left by the previous session's unclosed fd blocks a blocking
`open()`. FIX-1…FIX-7 (§11) address this: `O_NONBLOCK` on `open()`,
`TIOCEXCL`/`TIOCNXCL`, `HUPCL` disabled, `tcflush` before `tcgetattr`, explicit
`close()` on `EIO`.

### `calibrate_*()` measures too few ticks
Check: (1) `set_auto_report_state(true)` was called **before** calibration;
(2) encoders respond (`get_motor_encoder()` changes while motors spin);
(3) `duration_ms ≥ 200`; (4) wheels are free to spin.

### `enable_pid_control()` throws "no encoder packet received yet"
`set_auto_report_state(true)` has not yet yielded a packet. Poll
`get_battery_voltage() > 0` (or a changing `get_motor_encoder()`) first, then
retry.

---

## 18. Known limitations & future work

- **Encoder resolution at low speed.** The fixed-window estimate
  (`kVelWindow=10`, ~400 ms) quantises `measured` coarsely when few ticks land in
  the window. A **time-between-pulses** estimator would be more precise at low
  speed, but needs the exact ticks-per-wheel-revolution (post-gearbox) to be
  worth it.
- **PID rate.** `kPidHz=25` is low versus industrial platforms (hundreds of Hz
  to kHz). Higher is possible if UART/firmware latency allows — untested.
- **Feedforward not tested under real load.** Free-wheel calibration gives a
  valid no-load `kV`; under real load (robot on the ground, terrain) the slope
  differs (§12, ~25% loss under nominal load). The PID must absorb the gap via
  the residual term — validate on the full robot, not just on a free-wheel stand.
- **v8 gains not finalised.** `kp=0.3, ki=0.05, kd=0.0` are a starting point.
  Iterate on the bench convergence test.
- **Current loop** is firmware-only — no ROS 2 visibility or control.

---

## 19. Parameters to adjust

| Parameter          | Location                      | Current     | Change when                              |
| ------------------ | ----------------------------- | ----------- | ---------------------------------------- |
| `kPidHz`           | `Rosmaster.hpp` constexpr     | 25          | encoder packet rate changes              |
| `kVelWindow`       | `Rosmaster.hpp` constexpr     | 10          | `kPidHz` changes — recompute             |
| `kDerivAlpha`      | `Rosmaster.hpp` constexpr     | 0.8         | `kPidHz` changes — recompute τ           |
| `kp, ki, kd`       | `enable_pid_control()` call   | see §15     | tuning on the ground                     |
| `scale_global`     | calibration                   | ~9670 ticks/s | gearmotors change                      |
| `motor_scale_[i]`  | calibration                   | per motor   | idem                                     |
| `max_vel_rad_s_`   | URDF param                    | 10.0        | measure the real top speed               |
| `serial_port`      | URDF param / constructor      | `/dev/ttyUSB0` | per udev rules                        |

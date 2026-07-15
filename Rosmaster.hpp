/*
** Rosmaster.hpp for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Fri May 15 17:18:57 2026 dirennoukpo
** Last update Thu Jul 15 12:08:53 PM 2026 dirennoukpo
*/

/**
 * @file Rosmaster.hpp
 * @brief Single-header, fully-inline C++17 driver for the Yahboom Rosmaster motion controller (4-wheel mecanum robot "MecaMate").
 *
 * Ports Yahboom's Python Rosmaster driver (V3.3.9) to a header-only (`#pragma once`)
 * C++17 library and adds capabilities not present in the original: a per-motor
 * software velocity PID (v8), a kS/kV motor feedforward, automatic scale and
 * feedforward calibration, and a power-loss-resilient raw-POSIX serial lifecycle
 * (FIX-1..FIX-7, documented above). Intended primarily as the backend for a
 * ros2_control hardware interface, but it also runs standalone.
 *
 * Threading model: the app/ROS2 thread issues commands and reads getters; a receive
 * thread parses UART auto-report frames into std::atomic sensor members; a 25 Hz PID
 * thread reads encoder atomics and drives the motors. Cross-thread state is lock-free
 * via std::atomic except pid_gains_/motor_gains_ (pid_gains_mutex_), ff_gains_ (ff_mutex_)
 * and the arm read buffer (arm_mutex_). See the per-declaration @note tags for details.
 *
 * @author dirennoukpo <diren.noukpo@epitech.eu>
 * @date   Started 15 May 2026; serial FIX-1..7 + PID v8 revision 17 Jun 2026.
 */
#pragma once
// Rosmaster.hpp — C++ port of Rosmaster Python driver (V3.3.9)
// Requires C++17 or later.
//
// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL PORT LIFECYCLE — FULL DIAGNOSIS & FIX HISTORY
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ROOT CAUSE of "must reboot Pi after e-stop + Yahboom power-cycle":
//  ──────────────────────────────────────────────────────────────────
//  The kernel tty layer (CP210x / CH340) maintains internal state per fd.
//  When the USB-serial adapter loses power, the kernel signals VHANGUP on the
//  open fd. The sequence of failures is:
//
//    1. open() without O_NONBLOCK: if a previous fd was left open (e.g. the
//       receive thread was mid-read when EIO arrived and the port was not
//       closed before the next open()), the kernel blocks the new open() on
//       the VHANGUP tty session — indefinitely.
//
//    2. tcgetattr() / tcsetattr() on a just-reappeared CH340/CP210x fd
//       without flushing first: the kernel tty layer may have stale baud/flag
//       state from the previous session; tcsetattr() silently succeeds but the
//       hardware ignores the new configuration — causing "port open but no
//       comms" symptoms.
//
//    3. No TIOCEXCL: without exclusive-access locking, udev / ModemManager /
//       other processes can grab the port between our close() and the next
//       open(), preventing re-acquisition.
//
//    4. No TIOCNXCL before close(): the exclusive lock is inherited by the
//       kernel fd slot. If the fd is closed without releasing the lock, the
//       kernel sometimes keeps the tty locked for the lifetime of the process
//       — so the next open() by the *same* process also fails.
//
//    5. Thread / fd race in destructor: if the receive thread is sleeping in
//       its VTIME window when ~Rosmaster() runs, and ser_.close() is called
//       concurrently, ::read() on the now-closed fd returns EBADF — but the
//       thread is already past the uart_running_ check, so it spins on errors
//       until the OS kills the process.
//
//    6. EIO not triggering ser_.close() inside receiveLoop(): previous code
//       set uart_running_=false and returned, but left fd_ open. The kernel
//       tty then considers the port "still open" and refuses to fully reset
//       the CH340/CP210x on re-enumeration.
//
//  Why muto_link (servo driver) does NOT exhibit this:
//  ────────────────────────────────────────────────────
//  muto_link uses libserial / boost::asio, which:
//    • always opens with O_NONBLOCK + select/epoll
//    • calls TIOCEXCL / TIOCNXCL automatically
//    • catches EIO and calls close() internally
//  Our raw-POSIX implementation must replicate all of that explicitly.
//
//  Fixes applied in this revision (2026-06-15):
//  ─────────────────────────────────────────────
//  FIX-1  open() uses O_NONBLOCK; immediately cleared via fcntl so that
//          normal VMIN/VTIME blocking reads still work, but the kernel tty
//          session is established without the blocking-open VHANGUP trap.
//
//  FIX-2  TIOCEXCL is set immediately after open() so no other process can
//          steal the port between our open() and tcsetattr().
//
//  FIX-3  tcflush(TCIOFLUSH) is called BEFORE tcgetattr() to discard any
//          stale bytes / corrupt baud state left by the power-cycle.
//
//  FIX-4  HUPCL is cleared (cflag &= ~HUPCL) so the kernel does NOT drop
//          DTR/RTS on close() — the USB-serial adapter stays enumerated.
//
//  FIX-5  close() calls TIOCNXCL before ::close(fd_) to release the
//          exclusive lock so the SAME process can re-open the port cleanly.
//
//  FIX-6  receiveLoop() calls ser_.close() explicitly on EIO / ENOTTY /
//          EBADF — this triggers the kernel to fully reset the tty session,
//          so the next open() gets a clean slate.
//
//  FIX-7  Destructor order: uart_running_=false → recv_thread_.join() →
//          ser_.close(). The port is never closed while the thread may be
//          in ::read(), eliminating the EBADF race entirely.
//
// ═══════════════════════════════════════════════════════════════════════════════
//  SOFTWARE PID — CHANGELOG
// ═══════════════════════════════════════════════════════════════════════════════
//
//  v2 → v3 : separate gain/state structs, real dt, D filter, encoder wrap,
//             UART check, explicit saturation
//  v3 → v4 : pre-integration anti-windup (removes rollback inconsistency),
//             encoder_received_ flag (PID only starts after first packet),
//             calibrate_pid_scale() enforces duration_ms >= 100
//  v4 → v5 : [FIXED] typo deriv_filtered → derivative_filtered,
//             [FIXED] calibrate_pid_scale() guards result <= 0,
//             [FIXED] measured velocity clamped to [-200, 200],
//             [FIXED] set_motor() clamps targets to [-100, 100] when PID active,
//             [FIXED] velocity scale guarded against 0 in pidLoop() (NaN-safe),
//             [FIXED] encoder_received_ reset on UART reconnect (see parseData),
//             [NOTE]  PID frequency and integral limit remain tuning parameters
//  v5 → v6 : [FIXED] window_dt now computed from real timestamps stored in the
//             ring buffer (enc_history_ carries {enc[4], ts_us}) instead of
//             n_samples × nominal dt — eliminates the ~2.5% systematic bias
//             caused by the 24.4 Hz encoder packet rate vs 25 Hz PID rate.
//             [FIXED] derivative term: EMA applied to measured velocity
//             directly (not to (error-prev_error)/window_dt).  Divisor is
//             real dt, not window_dt, so D gain is dimensionally correct.
//             [FIXED] MotorPidState carries measured_filtered for D EMA.
//             [ADDED] calibrate_motor_scales() — multi-run, ratio-based
//             calibration (N runs, trimmed mean, inter-motor ratios, optional
//             warmup) promoted from standalone main() into the class.
//             calibrate_pid_scale_at() is kept as a single-run compatibility
//             wrapper that delegates to calibrate_motor_scales(n_runs=1).
//             [NOTE]  kVelWindow=10 (≈400 ms window) confirmed by bench log.
//  v6 → v7 : [FIXED] Direction-reversal instability ("zailling") — three bugs:
//             (1) D term now computed on EMA of error (not measured), so its
//             sign is correct in both rotation directions and helps accelerate
//             toward a new setpoint instead of opposing it.
//             (2) Anti-windup extended: integration is also allowed when
//             sign(error) ≠ sign(integral) (integral winding DOWN), so the
//             integrator can discharge during reversal even while saturated.
//             Without this, the integrator stays charged in the old direction
//             and fights the new command for hundreds of milliseconds.
//             (3) MotorPidState gains error_filtered field; prev_meas_filtered
//             renamed prev_meas_filtered for clarity.
//  v7 → v8 : [ADDED] Motor feedforward model — PWM = kS·sign(target) + kV·target
//             + PID, compensating DC-motor static friction (dead zone) and the
//             approximately-linear PWM→speed slope under load. Motor datasheet
//             (333 RPM @ 0.15A no-load, 250 RPM @ 0.65A / 1 kg·cm rated load,
//             stall 4 kg·cm @ 2.4A) shows ~25% speed sag under load and a
//             non-negligible dead zone — the previous 1:1 `target` feedforward
//             term implicitly assumed a perfectly linear, frictionless motor.
//             [ADDED] Per-motor feedforward gains (kS[i], kV[i]) — consistent
//             with the existing per-motor velocity scale design.
//             [ADDED] calibrate_feedforward() — automatic PWM sweep per motor:
//             ramps PWM 0→100% in small steps, records the PWM at which
//             rotation first sustains (kS) and fits a linear PWM-vs-speed
//             slope above the dead zone (kV) via least squares.
//             [NOTE]  PID term now corrects the *residual* error around the
//             feedforward prediction rather than carrying the full command,
//             so kp/ki/kd may need re-tuning (likely smaller gains suffice).

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <iomanip>

// ── Platform serial abstraction ──────────────────────────────────────────────
#ifdef _WIN32
#  include <windows.h>
#else
#  include <errno.h>
#  include <fcntl.h>
#  include <sys/ioctl.h>
#  include <termios.h>
#  include <unistd.h>
#endif

// ── Tiny cross-platform delay helper ─────────────────────────────────────────
/**
 * @brief Sleep the calling thread for a fractional number of milliseconds.
 *
 * Cross-platform pacing helper used to space out UART command frames (the driver's
 * default inter-frame gap is 2 ms) and to wait between calibration steps. Converts to
 * microseconds internally so sub-millisecond values are honoured exactly.
 *
 * @param ms  Delay in milliseconds; fractional values allowed (e.g. 0.002 -> 2 us).
 *            Non-positive values return effectively immediately.
 * @note Blocks whichever thread calls it and performs no I/O; touches no shared state.
 */
inline void delay_ms(double ms) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(ms * 1000.0)));
}

// =============================================================================
//  SerialPort — thin POSIX/Win32 wrapper with robust open/close lifecycle
// =============================================================================
/**
 * @brief Thin RAII wrapper over a raw POSIX termios fd (or Win32 HANDLE) with a hardened open/close lifecycle.
 *
 * Owns exactly one serial handle and configures it for 115200 8N1 raw I/O with
 * VMIN=0 / VTIME=5 (a ~500 ms read timeout so a read loop can periodically re-check its
 * running flag). The open/close paths implement FIX-1..FIX-7 (see file header) so the
 * port survives CH340/CP210x USB power-cycles and VHANGUP without a host reboot after an
 * e-stop: O_NONBLOCK open then cleared, TIOCEXCL/TIOCNXCL exclusive locking, pre-config
 * tcflush, and HUPCL cleared so the adapter stays enumerated across close().
 *
 * @note Non-copyable/non-assignable: it exclusively owns the OS handle. Not internally
 *       synchronised -- in this driver the recv thread reads while the app/PID threads
 *       write; callers must ensure the object is not driven concurrently in conflicting ways.
 * @see open, close, readByte
 */
class SerialPort {
public:
    /// Constructs a closed port; no OS handle is acquired until open() succeeds.
    SerialPort() = default;
    /// RAII cleanup: closes the port (releasing the TIOCEXCL lock, FIX-5) if still open. No-op if already closed.
    ~SerialPort() { close(); }

    /// Non-copyable and non-assignable: the object exclusively owns a single OS serial handle.
    SerialPort(const SerialPort &)            = delete;
    SerialPort & operator=(const SerialPort&) = delete;

    /**
     * @brief Open and configure the serial device for 115200 8N1 raw I/O.
     *
     * Opens O_NONBLOCK to avoid the blocking-open VHANGUP trap (FIX-1) then clears it so
     * subsequent reads honour VMIN/VTIME; sets TIOCEXCL for exclusive access (FIX-2), calls
     * tcflush(TCIOFLUSH) before tcgetattr to discard stale baud/byte state from a prior
     * session (FIX-3), and clears HUPCL so DTR/RTS are not dropped on close (FIX-4).
     *
     * @param port  Device path, e.g. "/dev/myserial" (POSIX) or "COM3" (Win32).
     * @param baud  Line speed; only 115200 (the Rosmaster wire speed) is exercised.
     * @return true if the device was opened and configured; false on any open/config failure.
     * @note Called from the app/ROS2 thread during Rosmaster construction, before any worker thread is started.
     * @see close
     */
    bool open(const std::string & port, int baud = 115200);
    /**
     * @brief Release the port: drop the exclusive lock, then close the handle. Idempotent.
     *
     * Calls TIOCNXCL before closing the file descriptor (FIX-5) so the SAME process can cleanly re-open the
     * device after it reappears. Safe to call when already closed. Invoked by the destructor,
     * by the 1500 ms silence watchdog, and by receiveLoop() on EIO/ENOTTY/EBADF (FIX-6) to
     * force a full kernel tty reset before the next open().
     *
     * @warning Must NOT be called while another thread is blocked in readByte() on this fd
     *          (EBADF race). The driver joins the recv thread before closing the port (FIX-7).
     * @see open, readByte
     */
    void close();
    /**
     * @brief Report whether the port currently holds a valid OS handle.
     * @return true if a device is open, false otherwise.
     */
    bool isOpen() const;
    /**
     * @brief Transmit a raw byte buffer over the port.
     *
     * Writes a fully-formed outgoing frame (HEAD 0xFF, DEVICE_ID 0xFC, len, func, params,
     * checksum) as-is; no framing or checksum is added here.
     *
     * @param data  Bytes to send.
     * @return Number of bytes written, or -1 on write error.
     * @note Called from the app/ROS2 thread (command frames) and from the PID thread
     *       (writeMotorRaw at 25 Hz). This class does not lock, so those writers must not overlap.
     */
    int  write(const std::vector<uint8_t> & data);

    // Returns  1 : byte received
    //          0 : timeout (VTIME window elapsed)
    //         -1 : unrecoverable error (EIO/ENOTTY/EBADF — device gone)
    /**
     * @brief Read exactly one byte, blocking up to the VTIME window (~500 ms).
     *
     * The tri-state return lets the receive loop distinguish a merely idle line (keep looping
     * and re-check uart_running_) from a vanished device (close the port and attempt reopen).
     *
     * @param out  Receives the byte only when the return value is 1; left untouched otherwise.
     * @return  1 = one byte read into @p out;
     *          0 = timeout / no data within the VTIME window;
     *         -1 = unrecoverable error (EIO/ENOTTY/EBADF, i.e. device gone) -- caller should close().
     * @note Called only from recv_thread_ inside receiveLoop().
     */
    int  readByte(uint8_t & out);
    /**
     * @brief Discard any bytes already received by the OS but not yet read (input flush).
     *
     * Used to drop stale or partial frames, e.g. after a (re)open or before re-syncing the frame parser.
     */
    void flushInput();

private:
#ifdef _WIN32
    /// Win32 serial handle; INVALID_HANDLE_VALUE when closed. (Windows build only.)
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
#else
    /// POSIX file descriptor for the open tty; -1 when closed. (Non-Windows build.)
    int fd_ = -1;
#endif
};

// =============================================================================
//  Rosmaster
// =============================================================================
/**
 * @brief C++17 driver for the Yahboom Rosmaster controller on a 4-wheel mecanum robot ("MecaMate").
 *
 * Ports Yahboom's Python driver V3.3.9 and adds a per-motor software velocity PID, a kS/kV
 * motor feedforward, automatic calibration, and a power-loss-resilient POSIX serial lifecycle.
 * Intended as the backend for a ros2_control hardware interface, but also runs standalone.
 *
 * Threading model: the owning (app/ROS2) thread calls the setters, getters and configuration
 * methods declared here; a receive thread parses UART auto-report frames into std::atomic sensor
 * members; a 25 Hz PID thread reads encoder atomics and drives the motors. Cross-thread data is
 * lock-free via std::atomic, except pid_gains_/motor_gains_ (pid_gains_mutex_), ff_gains_
 * (ff_mutex_) and the arm read buffer (arm_mutex_).
 *
 * Typical startup: construct -> create_receive_threading() -> set_auto_report_state(true) ->
 * wait for the first encoder frame -> calibrate_* -> enable_pid_control().
 * Non-copyable: one instance owns one serial device plus its two background threads.
 */
class Rosmaster {
public:
    // car_type: 1=X3, 2=X3_PLUS, 4=X1, 5=R2
    /**
     * @brief Open the serial port and construct the driver (does NOT start any threads).
     *
     * Opens @p com at 115200 8N1, energises the arm servo torque, and stores the car type. You
     * must still call create_receive_threading() and set_auto_report_state(true) before any sensor
     * getter or the software PID will function.
     *
     * @param car_type  chassis/firmware profile: 1=X3, 2=X3_PLUS, 4=X1, 5=R2 (see CARTYPE_* constants).
     * @param com       serial device path, e.g. "/dev/myserial" or "/dev/ttyUSB0".
     * @param delay     inter-command pacing delay in seconds (default 0.002 s = 2 ms).
     * @param debug     when true, prints protocol/debug traces to stdout.
     * @throws std::runtime_error  if the serial port cannot be opened.
     * @note Owning thread only; construction alone spawns no background work.
     */
    explicit Rosmaster(int car_type = 1,
                       const std::string & com = "/dev/myserial",
                       double delay = 0.002,
                       bool debug = false);
    /**
     * @brief Stop the PID and receive threads and close the serial port.
     *
     * Shutdown order is deliberate: disable_pid_control() joins the PID thread first, then
     * uart_running_ is cleared and the receive thread is joined, then the port is closed. The PID
     * thread must go first because pidLoop() reads encoder atomics populated by receiveLoop().
     * @note Safe even if the threads never started or the device was unplugged; the serial layer
     *       closes cleanly so the host never needs a reboot after an e-stop.
     */
    ~Rosmaster();

    Rosmaster(const Rosmaster &)            = delete;
    Rosmaster & operator=(const Rosmaster&) = delete;

    // ── Threading ─────────────────────────────────────────────────────────
    /**
     * @brief Start the background UART receive thread (receiveLoop()).
     *
     * Spawns recv_thread_, which continuously reads the serial port and decodes auto-report frames
     * into the atomic sensor members. Idempotent: calling it again while already running is a no-op.
     * Must precede set_auto_report_state(true), any sensor getter, and enable_pid_control().
     * @note Call once from the owning thread after construction. Sleeps ~50 ms so the thread is up
     *       before returning.
     */
    void create_receive_threading();

    // ── Health check ──────────────────────────────────────────────────────
    /**
     * @brief Whether the receive thread is currently running.
     * @return true while recv_thread_ is active; false before start / after shutdown.
     * @note Lock-free read of the atomic uart_running_; callable from any thread.
     */
    bool is_running() const {
        return uart_running_.load(std::memory_order_relaxed);
    }

    // ── Car control ───────────────────────────────────────────────────────
    /**
     * @brief Enable/disable the firmware's periodic sensor auto-report stream.
     *
     * When enabled, the controller pushes IMU, encoder, battery and motion frames that the receive
     * thread decodes into the atomic members. Every sensor getter and the software PID depend on it.
     *
     * @param enable   true to start the auto-report stream, false to stop it.
     * @param forever   true persists the setting to controller flash (survives reboot); false applies
     *                 it for the current power cycle only.
     * @note Requires create_receive_threading() first. After enabling, wait ~100 ms for the first
     *       encoder frame before enabling PID. Sent from the owning thread; errors are logged, not thrown.
     */
    void set_auto_report_state(bool enable, bool forever = false);
    /**
     * @brief Sound the on-board buzzer.
     * @param on_time  buzzer duration in milliseconds, must be a multiple of 10 (Yahboom convention):
     *                 0 stops the beep, 1 beeps continuously until stopped, >=10 beeps for that many ms.
     *                 Negative values are rejected.
     * @note Fire-and-forget serial command; failures are swallowed and logged to stderr.
     */
    void set_beep(int on_time);
    /**
     * @brief Set the angle of one of the four PWM (hobby) servos S1..S4.
     * @param servo_id  PWM channel 1..4; out-of-range ids are ignored.
     * @param angle     target angle in degrees, clamped to [0, 180].
     * @note These are the camera/gimbal PWM servos, distinct from the UART bus (arm) servos.
     * @see set_pwm_servo_all
     */
    void set_pwm_servo(int servo_id, int angle);
    /**
     * @brief Set all four PWM servos S1..S4 in a single frame.
     * @param angle_s1  channel S1 angle in degrees, [0, 180].
     * @param angle_s2  channel S2 angle in degrees, [0, 180].
     * @param angle_s3  channel S3 angle in degrees, [0, 180].
     * @param angle_s4  channel S4 angle in degrees, [0, 180].
     * @see set_pwm_servo
     */
    void set_pwm_servo_all(int angle_s1, int angle_s2, int angle_s3, int angle_s4);
    /**
     * @brief Set the RGB colour of one addressable LED (or the whole strip).
     * @param led_id  zero-based LED index, or 0xFF (255) to address every LED at once.
     * @param red     red channel, 0..255.
     * @param green   green channel, 0..255.
     * @param blue    blue channel, 0..255.
     * @note Stop any running animation (set_colorful_effect(0)) before driving LEDs directly.
     */
    void set_colorful_lamps(int led_id, int red, int green, int blue);
    /**
     * @brief Run a built-in animated LED light effect.
     * @param effect  firmware effect id (0 = off / static).
     * @param speed   animation speed 0..255, larger = faster (default 255).
     * @param parm    effect-specific parameter, e.g. colour selector (default 255).
     * @see set_colorful_lamps
     */
    void set_colorful_effect(int effect, int speed = 255, int parm = 255);
    /**
     * @brief Command the four wheel motors, in percent of full scale.
     *
     * Dual-mode. If the software PID is enabled, the four values are clamped to [-100,100] and stored
     * atomically as the PID velocity targets (the PID thread does the real driving). Otherwise they
     * are packed and sent straight to the controller as open-loop motor commands.
     *
     * @param s1  FL wheel command in percent [-100,100]; sign sets direction, out-of-range clamped.
     * @param s2  FR wheel command in percent [-100,100].
     * @param s3  RL wheel command in percent [-100,100].
     * @param s4  RR wheel command in percent [-100,100].
     * @note Thread-safe: in PID mode it only writes target atomics; in open-loop mode it writes the
     *       serial port. Call from the owning thread.
     * @see enable_pid_control, set_motor_with_compensation, get_pid_measured
     */
    void set_motor(double speed_1, double speed_2, double speed_3, double speed_4);
    /**
     * @brief High-level directional drive using the firmware's motion presets.
     * @param state   direction preset (firmware-defined: stop / forward / back / left / right / spin).
     * @param speed   speed magnitude in the firmware's units (packed as int16).
     * @param adjust  when true, sets the gyro-assisted heading-hold adjust bit.
     * @note Bypasses the host software PID; the controller's own closed loop executes the motion.
     * @see set_car_motion
     */
    void set_car_run(int state, int speed, bool adjust = false);
    /**
     * @brief Command chassis body velocity (mecanum inverse kinematics resolved in firmware).
     * @param v_x  forward/back linear velocity in m/s (sent as v*1000 int16).
     * @param v_y  left/right strafe linear velocity in m/s.
     * @param v_z  yaw angular velocity in rad/s.
     * @note Bypasses the host wheel PID; the firmware computes per-wheel speeds. Owning thread.
     * @see set_car_run
     */
    void set_car_motion(double v_x, double v_y, double v_z);
    /**
     * @brief Program the CONTROLLER'S firmware velocity-PID gains (not the host software PID).
     *
     * These gains live on the Rosmaster board and only affect its firmware-side closed-loop modes
     * (set_car_run / set_car_motion). They are unrelated to enable_pid_control()'s on-host PID.
     *
     * @param kp  proportional gain, valid in [0, 10].
     * @param ki  integral gain, valid in [0, 10].
     * @param kd  derivative gain, valid in [0, 10]. Any out-of-range gain rejects the whole call with a warning.
     * @param forever  true persists the gains to controller flash (adds a ~100 ms delay).
     * @see set_pid_gains, enable_pid_control
     */
    void set_pid_param(double kp, double ki, double kd, bool forever = false);
    /**
     * @brief Tell the controller which chassis type is attached and persist it to flash.
     * @param car_type  one of CARTYPE_X3/X3_PLUS/X1/R2 (0..255 accepted).
     * @note Blocks ~100 ms while the controller writes flash.
     * @see get_car_type_from_machine
     */
    void set_car_type(int car_type);
    /**
     * @brief Move one bus (UART) servo to a raw pulse position over a time interval.
     * @param servo_id     bus servo id.
     * @param pulse_value  raw servo pulse target (firmware units).
     * @param run_time     time to reach the target, in ms (default 500).
     * @note These are the robotic-arm bus servos, distinct from the PWM servos.
     * @see set_uart_servo_angle
     */
    void set_uart_servo(int servo_id, int pulse_value, int run_time = 500);
    /**
     * @brief Move one arm bus servo to an angle in degrees (converted to a pulse internally).
     * @param s_id      arm joint id 1..6.
     * @param s_angle   target angle in degrees (per-joint range; joint 5 spans up to 270).
     * @param run_time  time to reach the target, in ms (default 500).
     * @see set_uart_servo_angle_array, get_uart_servo_angle
     */
    void set_uart_servo_angle(int s_id, int s_angle, int run_time = 500);
    void set_uart_servo_id(int servo_id);
    /**
     * @brief Enable or disable holding torque on all UART bus (arm) servos.
     * @param enable  1 = torque on (servos hold position and resist back-drive), 0 = limp/free.
     * @note Energised once in the constructor; set to 0 to hand-pose the arm.
     */
    void set_uart_servo_torque(int enable);
    /**
     * @brief Enable/disable UART bus-servo command output from the controller.
     * @param enable  true lets servo-angle commands reach the bus; false suppresses them.
     */
    void set_uart_servo_ctrl_enable(bool enable);
    /**
     * @brief Move all six arm joints to a set of angles in a single command.
     * @param angle_s   six joint angles in degrees {j1..j6}; default {90,90,90,90,90,180}.
     * @param run_time  time to reach the targets, in ms (default 500).
     * @see set_uart_servo_angle, get_uart_servo_angle_array
     */
    void set_uart_servo_angle_array(std::vector<int> angle_s = {90,90,90,90,90,180},
                                    int run_time = 500);
    /**
     * @brief Calibrate and store the mid-point offset of one arm servo at its current pose (blocking).
     * @param servo_id  arm joint id to zero at its present position.
     * @return firmware status/offset code from the controller (negative on timeout/error).
     * @note Blocking request/response; requires the receive thread + auto-report active.
     */
    int  set_uart_servo_offset(int servo_id);
    /**
     * @brief Set the Ackermann steering servo's centre (straight-ahead) angle.
     * @param angle    centre angle in degrees.
     * @param forever  true persists the centre to flash.
     * @note Ackermann-steer chassis only (e.g. R2).
     * @see set_akm_steering_angle, get_akm_default_angle
     */
    void set_akm_default_angle(int angle, bool forever = false);
    /**
     * @brief Command the Ackermann steering angle.
     * @param angle     steering angle in degrees relative to centre; sign selects the turn side.
     * @param ctrl_car  when true, the controller also drives the wheels to follow the turn.
     * @note Ackermann-steer chassis only.
     */
    void set_akm_steering_angle(int angle, bool ctrl_car = false);
    /**
     * @brief Erase controller flash back to factory defaults (car type, PID, offsets, etc.).
     * @warning Destructive and persistent: wipes all calibration stored on the board. Re-run your
     *          setup afterwards.
     */
    void reset_flash_value();
    /**
     * @brief Immediately stop all motion: zeroes motors, buzzer and lights on the controller.
     * @note Fast controller-side e-stop; does NOT stop the host PID thread. Also call
     *       disable_pid_control() (or set_motor(0,0,0,0)) to keep the loop from re-commanding motion.
     */
    void reset_car_state();
    /**
     * @brief Clear the host-side cached auto-report values back to zero/defaults.
     * @note Only resets the local atomic mirrors; sends nothing to the controller. Owning thread.
     */
    void clear_auto_report_data();

    // ── Getters ───────────────────────────────────────────────────────────
    /**
     * @brief Read the latest accelerometer sample.
     * @param ax  out: X-axis linear acceleration, in m/s^2.
     * @param ay  out: Y-axis linear acceleration, in m/s^2.
     * @param az  out: Z-axis linear acceleration, in m/s^2.
     * @note Lock-free read of atomics filled by the receive thread; values are 0 until the first IMU
     *       auto-report arrives. const, callable from any thread.
     */
    void   get_accelerometer_data(double & ax, double & ay, double & az) const;
    /**
     * @brief Read the latest gyroscope sample.
     * @param gx  out: X-axis angular rate, in rad/s.
     * @param gy  out: Y-axis angular rate, in rad/s.
     * @param gz  out: Z-axis angular rate, in rad/s.
     * @note Lock-free atomic read; 0 until the first IMU frame. @see get_accelerometer_data
     */
    void   get_gyroscope_data(double & gx, double & gy, double & gz) const;
    /**
     * @brief Read the latest magnetometer sample.
     * @param mx  out: X-axis magnetic field (raw firmware units).
     * @param my  out: Y-axis magnetic field.
     * @param mz  out: Z-axis magnetic field.
     * @note Lock-free atomic read; 0 until the first frame. Not populated on all IMU variants.
     */
    void   get_magnetometer_data(double & mx, double & my, double & mz) const;
    /**
     * @brief Read the fused IMU attitude (roll/pitch/yaw).
     * @param roll      out: roll angle.
     * @param pitch     out: pitch angle.
     * @param yaw       out: yaw angle.
     * @param to_angle  true returns degrees, false returns radians (default true).
     * @note Lock-free atomic read filled by the receive thread. Feed the pitch (in radians) to
     *       update_pitch() to drive slope compensation.
     */
    void   get_imu_attitude_data(double & roll, double & pitch, double & yaw,
                                 bool to_angle = true) const;
    /**
     * @brief Read the controller's estimated chassis body velocity.
     * @param vx  out: forward/back linear velocity, in m/s.
     * @param vy  out: strafe linear velocity, in m/s.
     * @param vz  out: yaw angular velocity, in rad/s.
     * @note Lock-free atomic read; reflects the last motion auto-report frame.
     */
    void   get_motion_data(double & vx, double & vy, double & vz) const;
    /**
     * @brief Battery pack voltage.
     * @return pack voltage in volts (firmware reports decivolts; divided by 10 here).
     * @note Lock-free atomic read; 0 until the first report frame.
     */
    double get_battery_voltage() const;
    /**
     * @brief Read the four cumulative wheel-encoder counters (FL, FR, RL, RR).
     * @param m1  out: FL cumulative tick count (signed, monotonic, never reset by firmware).
     * @param m2  out: FR cumulative tick count.
     * @param m3  out: RL cumulative tick count.
     * @param m4  out: RR cumulative tick count.
     * @note Lock-free atomic read from the receive thread. Deltas must be taken modulo 2^32 (int32
     *       cast) to survive wraparound. This is the raw feedback the software PID differentiates into velocity.
     */
    void   get_motor_encoder(int & m1, int & m2, int & m3, int & m4) const;
    /**
     * @brief Read the four wheel speeds as measured by the firmware, raw (blocking round-trip).
     *
     * The board differentiates its own encoders every 10 ms and reports the result directly,
     * so you get a wheel velocity without having to differentiate get_motor_encoder() yourself.
     * All four wheels are positive when the robot drives forward -- the same frame of reference
     * as get_motion_data().
     *
     * @return {FL, FR, RL, RR} in m/s, or std::nullopt on a ~30 ms timeout.
     * @note Request-only: this datum is NEVER auto-reported, so each call sends a
     *       FUNC_REQUEST_DATA and polls. Requires the receive thread; auto-report is not needed.
     * @note std::optional rather than a {-1,-1,-1,-1} sentinel ON PURPOSE: -1 m/s is a
     *       perfectly legitimate wheel speed, so a sentinel would be indistinguishable from data.
     *       std::nullopt is returned when the board does not answer, e.g. it runs a firmware older
     *       than V3.5.1 that has no 0x08 report.
     * @warning Blocks the calling thread for up to ~30 ms. Fine for diagnostics and PID tuning;
     *          do NOT put it in a ros2_control read() hot path.
     * @warning Writes to the serial port from the calling thread. If the software PID is running,
     *          its 25 Hz writeMotorRaw() writes concurrently and SerialPort is not internally
     *          synchronised (a pre-existing property of this class, shared with get_version() and
     *          get_motion_pid()).
     * @see get_motor_speed_lpf, get_motor_encoder, get_pid_measured
     */
    std::optional<std::array<double, 4>> get_motor_speed_raw();
    /**
     * @brief Read the four wheel speeds after the firmware's low-pass filter (blocking round-trip).
     *
     * Same payload as get_motor_speed_raw(), but passed through a first-order IIR on the board:
     * y[n] += 0.2 * (x[n] - y[n]) at a 10 ms sampling period, i.e. tau ~= 40 ms and a cutoff
     * around 4 Hz. Much quieter than the raw value -- it attenuates encoder dither by roughly a
     * factor of ten -- at the cost of a lag on acceleration (about 25 mm/s behind the raw value
     * on a 0.76 m/s^2 ramp, measured on the bench).
     *
     * Prefer this one for telemetry, logging and anything a human reads; prefer
     * get_motor_speed_raw() when you need the freshest possible measurement.
     *
     * @return {FL, FR, RL, RR} in m/s, or std::nullopt on a ~30 ms timeout.
     * @note Same request-only mechanics, same blocking cost and same serial-write caveat as
     *       get_motor_speed_raw(); see that method's notes.
     * @see get_motor_speed_raw
     */
    std::optional<std::array<double, 4>> get_motor_speed_lpf();
    /**
     * @brief Snapshot the firmware's low-pass wheel speeds {FL,FR,RL,RR} in m/s from the
     *        PUSHED 0x09 auto-report -- lock-free, never blocks, no round-trip.
     *
     * Unlike get_motor_speed_lpf() (request-then-poll), this reads the atomics that
     * receiveLoop() fills on every 0x09 frame the firmware pushes at 100 Hz. Meant for the
     * ros2_control read() hot path, exactly like get_motor_encoder() / get_accelerometer_data().
     *
     * @param fresh out (optional): true once the first pushed 0x09 frame has arrived (firmware
     *        with the retuned auto-report that pushes 0x09). Stays true while the stream runs;
     *        false against firmware that never pushes 0x09 -- differentiate get_motor_encoder()
     *        as a fallback in that case.
     * @return {FL, FR, RL, RR} in m/s (0 until the first 0x09 frame arrives).
     * @note Lock-free atomic read, never blocks; safe at any rate. Because the pushed stream
     *       never clears the arrival flag, do NOT interleave get_motor_speed_lpf() (the request
     *       version clears it) once you rely on *fresh.
     * @see get_motor_speed_lpf, get_motor_encoder, get_accelerometer_data
     */
    std::array<double, 4> get_motor_speed_lpf_data(bool * fresh = nullptr) const;
    /**
     * @brief Query the controller's firmware motion-PID gains (blocking round-trip).
     * @return {kp, ki, kd} read back from the board, or {-1,-1,-1} on timeout (~20 ms).
     * @note Sends a request then polls atomics filled by the receive thread; requires the receive
     *       thread + auto-report active. These are the firmware gains, not the host PID.
     * @see set_pid_param
     */
    std::vector<double>      get_motion_pid();
    /**
     * @brief Read one arm bus servo's id and raw position (blocking round-trip).
     * @param servo_id  bus servo id 1..250; invalid ids return {-1,-1}.
     * @return {id, raw_value} on success; {-1,-1} on ~30 ms timeout; {-2,-2} on exception.
     * @note Blocking request/response; requires the receive thread + auto-report active.
     */
    std::pair<int,int>       get_uart_servo_value(int servo_id);
    /**
     * @brief Read one arm joint's angle in degrees (blocking; wraps get_uart_servo_value).
     * @param s_id  arm joint id 1..6.
     * @return angle in degrees, or -1 on timeout / out-of-range, or -2 on exception.
     */
    int                      get_uart_servo_angle(int s_id);
    /**
     * @brief Read all six arm joint angles in one blocking round-trip.
     * @return six joint angles in degrees; entries are -1 where unread, or all -2 on exception.
     * @note Guards the shared read buffer with arm_mutex_; requires the receive thread + auto-report.
     */
    std::vector<int>         get_uart_servo_angle_array();
    /**
     * @brief Read the stored Ackermann steering centre angle (blocking, cached after first read).
     * @return centre angle in degrees, or -1 on timeout (~1 s).
     * @see set_akm_default_angle
     */
    int                      get_akm_default_angle();
    /**
     * @brief Read the controller firmware version (blocking, cached after first read).
     * @return version as major.minor (e.g. 3.9), or -1.0 on ~20 ms timeout.
     * @note Request/response over UART; needs the receive thread running.
     */
    double                   get_version();
    /**
     * @brief Query the chassis type currently stored on the controller (blocking round-trip).
     * @return the CARTYPE_* code reported by the board, or -1 on ~20 ms timeout.
     * @see set_car_type
     */
    int                      get_car_type_from_machine();

    // ── Software PID control ──────────────────────────────────────────────
    /**
     * @brief Start the host-side per-motor velocity PID loop (25 Hz pidLoop thread).
     *
     * Spawns pid_thread_, which reads encoder atomics, computes velocity error against the targets
     * stored by set_motor(), and drives the motors via writeMotorRaw(). Idempotent. After this call,
     * set_motor() values are interpreted as PID velocity targets in percent [-100,100].
     *
     * @param kp             global proportional gain (default 1.8).
     * @param ki             global integral gain (default 0.4).
     * @param kd             global derivative gain (default 0.05).
     * @param ticks_per_sec  encoder ticks/s that correspond to 100% command (the velocity scale;
     *                       default 1326.0). Get an accurate value from calibrate_motor_scales().
     * @throws std::runtime_error  if the receive thread is not running, or if no encoder frame has
     *                             been received yet (call set_auto_report_state(true) and wait ~100 ms).
     * @warning Warns to stderr (but still runs) when the scale or feedforward is uncalibrated.
     * @note Gains are stored under pid_gains_mutex_. Prefer calibrating first.
     * @see disable_pid_control, set_pid_gains, calibrate_motor_scales, enable_feedforward
     */
    void   enable_pid_control(double kp            = 1.8,
                              double ki            = 0.4,
                              double kd            = 0.05,
                              double ticks_per_sec = 1326.0);
    /**
     * @brief Stop the PID loop, join its thread, and reset PID state. Idempotent.
     *
     * After this returns, set_motor() reverts to open-loop pass-through. Automatically invoked by the
     * destructor. Blocks until pid_thread_ has joined (up to about one loop period).
     * @note Owning thread only. Leaves motors at their last command; send set_motor(0,0,0,0) to stop.
     * @see enable_pid_control
     */
    void   disable_pid_control();
    /**
     * @brief Update the global PID gains live, while the loop is running.
     * @param kp  new global proportional gain.
     * @param ki  new global integral gain.
     * @param kd  new global derivative gain, applied on the next tick.
     * @note Thread-safe: stored under pid_gains_mutex_. Per-motor overrides (set_motor_pid_gains) take
     *       precedence over these globals.
     */
    void   set_pid_gains(double kp, double ki, double kd);
    /**
     * @brief Clear integrators, filtered error, targets and measured velocities to zero.
     * @note Allowed only while the PID thread is stopped; ignored (with a stderr warning) if the loop
     *       is active. Call disable_pid_control() first.
     */
    void   reset_pids();
    /**
     * @brief Quick single-throttle estimate of the velocity scale (ticks/s at 100%).
     * @param duration_ms  measurement window in ms (>= 100 required).
     * @return measured ticks/s at 100% command, to feed enable_pid_control().
     * @throws std::logic_error       if the PID is currently enabled (disable it first).
     * @throws std::invalid_argument  if duration_ms < 100 (too few ticks for a reliable estimate).
     * @warning Spins the wheels at full throttle; ensure they are free and the robot is safe.
     * @see calibrate_motor_scales
     */
    double calibrate_pid_scale(int duration_ms = 300);

    // Multi-run ratio-based calibration (recommended).
    // throttle_pct : [20, 80]  — working point for the measurement
    // duration_ms  : run duration (>= 200 ms)
    // n_runs       : number of runs (>= 3 recommended; min+max trimmed)
    // warmup_ms    : thermal warmup at 60% before runs (0 = skip)
    // use_per_motor: populate motor_scale_[i] per motor
    // Returns global scale (ticks/s at 100%).
    /**
     * @brief Robust multi-run, ratio-based calibration of the per-motor velocity scales (recommended).
     *
     * Runs the motors N times at @p throttle_pct, derives the global ticks/s@100% via inter-motor
     * ratios (stable under +-5% thermal drift) and a trimmed mean (drops min+max when n_runs>=3), then
     * sets each motor's scale = global * that motor's mean ratio. Prints the coefficient of variation
     * and warns if CV > 1.5%.
     *
     * @param throttle_pct   working-point throttle in percent, [20,80] (default 60).
     * @param duration_ms    per-run duration in ms (>= 200; default 800).
     * @param n_runs         number of runs (>= 3 recommended; min+max trimmed; default 5).
     * @param warmup_ms      thermal warm-up at 60% before the runs, in ms (0 = skip; default 3000).
     * @param use_per_motor  when true, populates motor_scale_[i]; else only the global scale.
     * @return the global velocity scale in ticks/s at 100% command.
     * @throws std::logic_error  if the PID is enabled (disable it first).
     * @warning Spins all wheels; keep the robot safe and the wheels free.
     * @see set_motor_scales, calibrate_pid_scale_at, enable_pid_control
     */
    double calibrate_motor_scales(int throttle_pct   = 60,
                                  int duration_ms    = 800,
                                  int n_runs         = 5,
                                  int warmup_ms      = 3000,
                                  bool use_per_motor = true);

    // Single-run compatibility wrapper → delegates to calibrate_motor_scales(n=1).
    /**
     * @brief Single-run convenience wrapper around calibrate_motor_scales(n_runs=1).
     * @param throttle_pct   working-point throttle in percent (default 60).
     * @param duration_ms    run duration in ms (default 800).
     * @param use_per_motor  when true, also fills the per-motor scales.
     * @return the global velocity scale (ticks/s at 100%).
     * @note Faster but noisier than the multi-run calibrate_motor_scales(); prefer that one.
     */
    double calibrate_pid_scale_at(int throttle_pct  = 60,
                                  int duration_ms   = 800,
                                  bool use_per_motor = true);

    // Injection directe des échelles après calibration externe
    /**
     * @brief Inject externally-computed velocity scales, bypassing on-board calibration.
     * @param scales  per-motor ticks/s at 100% command, {FL, FR, RL, RR}.
     * @param global  the global ticks/s@100% used as the PID velocity scale.
     * @note Marks the scale as calibrated so enable_pid_control() will not warn. Use to restore
     *       calibration values saved from a previous session.
     */
    void set_motor_scales(const std::array<double,4> & scales, double global);

    // ── Motor feedforward (v8) ──────────────────────────────────────────────
    // Model: pwm_ff = kS[i]·sign(target) + kV[i]·target   (target in %, [-100,100])
    //   kS : static-friction / dead-zone offset (% PWM needed just to start moving)
    //   kV : linear PWM-per-%-speed slope above the dead zone
    // Calibrated automatically per motor via a PWM sweep (see below), or set
    // directly with set_feedforward_gains() if you already have values.
    //
    // throttle_min/max_pct : sweep bounds (defaults span dead-zone to near-max)
    // step_pct             : PWM increment per sweep point
    // settle_ms            : time to wait for speed to stabilize at each step
    // sample_ms            : window used to measure speed at each step
    // Returns the average dead-zone PWM (%) across the 4 motors.
    /**
     * @brief Auto-calibrate the per-motor kS/kV feedforward via an upward PWM sweep.
     *
     * For each motor, sweeps PWM from @p throttle_min_pct to @p throttle_max_pct: kS[i] is the first
     * PWM% that breaks the dead zone (motor starts moving); kV[i] is the least-squares PWM-per-%-speed
     * slope in the linear region above kS. Results are stored and feedforward_calibrated_ is set, but
     * you must still call enable_feedforward(true) to apply them.
     *
     * @param throttle_min_pct  sweep start PWM% (default 5).
     * @param throttle_max_pct  sweep end PWM%, <= 100 (default 70).
     * @param step_pct          PWM increment per step, >= 1 (default 3).
     * @param settle_ms         settle time per step, in ms (default 250).
     * @param sample_ms         speed-measurement window per step, in ms (default 300).
     * @return the average dead-zone PWM (kS) across the four motors, in percent.
     * @throws std::logic_error / std::invalid_argument  if the PID is enabled or the sweep parameters
     *         are invalid; also throws if no motor ever breaks the dead zone.
     * @warning Spins all wheels; wheels must be free. Requires auto-report active.
     * @see set_feedforward_gains, enable_feedforward
     */
    double calibrate_feedforward(int throttle_min_pct = 5,
                                 int throttle_max_pct  = 70,
                                 int step_pct          = 3,
                                 int settle_ms         = 250,
                                 int sample_ms         = 300);

    /**
     * @brief Set one motor's feedforward gains directly (skip calibration).
     * @param motor_index  0=FL, 1=FR, 2=RL, 3=RR; out-of-range is ignored.
     * @param kS           static / dead-zone PWM offset in percent, applied as kS*sign(target).
     * @param kV           PWM-per-%-speed slope (kS=0, kV=1 reproduces the pre-feedforward behaviour).
     * @note Thread-safe under ff_mutex_; also marks feedforward as calibrated.
     * @see get_feedforward_gains, calibrate_feedforward
     */
    void set_feedforward_gains(int motor_index, double kS, double kV);
    /**
     * @brief Read back one motor's feedforward gains.
     * @param motor_index  0=FL, 1=FR, 2=RL, 3=RR; out-of-range returns the neutral {kS=0, kV=1}.
     * @param kS  out: the stored static/dead-zone offset (percent).
     * @param kV  out: the stored velocity slope.
     * @note Thread-safe read under ff_mutex_. const.
     */
    void get_feedforward_gains(int motor_index, double & kS, double & kV) const;
    /**
     * @brief Turn the kS/kV feedforward term of the PID law on or off at runtime.
     * @param enable  true adds ff_term = kS*sign(target) + kV*target to each motor command; false makes
     *                the loop pure PID. No feedforward is applied when target==0 (avoids creep at rest).
     * @note Lock-free atomic flag; safe from any thread. Calibrate first for meaningful gains.
     */
    void enable_feedforward(bool enable) {
        feedforward_enabled_.store(enable, std::memory_order_relaxed);
    }

    // Accès à writeMotorRaw depuis main() pour la calibration
    void writeMotorRaw_public(const std::array<double,4> & cmd) {
        writeMotorRaw(cmd);
    }

    // ── Per-motor PID gain overrides ──────────────────────────────────────
    // motor_index : 0=FL, 1=FR, 2=RL, 3=RR
    /**
     * @brief Override the global PID gains for a single motor (e.g. a weaker or mismatched wheel).
     * @param motor_index  0=FL, 1=FR, 2=RL, 3=RR; out-of-range is ignored.
     * @param kp           per-motor proportional gain used when the override is active.
     * @param ki           per-motor integral gain.
     * @param kd           per-motor derivative gain.
     * @param override     true activates the per-motor gains; false leaves them stored but inactive
     *                     (global gains apply). Default true.
     * @note Thread-safe under pid_gains_mutex_; overrides take precedence over set_pid_gains().
     * @see reset_motor_pid_gains
     */
    void set_motor_pid_gains(int motor_index,
                             double kp, double ki, double kd,
                             bool override = true);
    /**
     * @brief Clear a motor's per-motor gain override so it reverts to the global PID gains.
     * @param motor_index  0=FL, 1=FR, 2=RL, 3=RR; out-of-range is ignored.
     * @note Thread-safe under pid_gains_mutex_. Deactivates the override only; the stored gains remain.
     */
    void reset_motor_pid_gains(int motor_index);

    // ── Slope compensation ────────────────────────────────────────────────
    /**
     * @brief Configure gravity/slope compensation applied by set_motor_with_compensation().
     * @param enabled       master on/off for slope compensation.
     * @param k_gravity     gain mapping pitch into extra motor command to hold position on an incline.
     * @param deadband_rad  pitch magnitude, in radians, below which no compensation is applied.
     * @param timeout_ns    max age of the last update_pitch() sample, in ns; a staler sample disables comp.
     * @note Stores plain members read on the owning thread; feed pitch via update_pitch().
     * @see set_max_pwm_flat, update_pitch, set_motor_with_compensation
     */
    void configure_slope_compensation(bool enabled,
                                      double k_gravity,
                                      double deadband_rad,
                                      uint64_t timeout_ns);
    /**
     * @brief Set the PWM ceiling reserved for flat-ground driving so slope comp has headroom.
     * @param pwm  max flat-ground command in percent, clamped to [10, 100].
     * @note set_motor_with_compensation() reserves headroom below this so the gravity term never
     *       saturates the motor on an incline.
     */
    void set_max_pwm_flat(double pwm);
    /**
     * @brief Feed the current chassis pitch to the slope-compensation system.
     * @param pitch_rad  pitch angle in radians (positive = nose-up), typically from the IMU.
     * @note Lock-free: stores the value and a steady_clock timestamp as atomics. If not refreshed
     *       within the configured timeout, compensation self-disables for safety.
     * @see configure_slope_compensation, get_imu_attitude_data
     */
    void update_pitch(double pitch_rad);
    /**
     * @brief Like set_motor(), but first reserves headroom and adds the gravity/slope term.
     * @param s1  FL wheel command in percent [-100,100].
     * @param s2  FR wheel command in percent [-100,100].
     * @param s3  RL wheel command in percent [-100,100].
     * @param s4  RR wheel command in percent [-100,100].
     * @note Uses the most recent update_pitch() sample; falls back to plain set_motor() behaviour when
     *       compensation is disabled or the pitch sample is stale.
     * @see configure_slope_compensation, set_max_pwm_flat, update_pitch
     */
    void set_motor_with_compensation(double s1, double s2, double s3, double s4);

    // Last velocity measured by the PID loop (%), {0,0,0,0} when PID disabled.
    /**
     * @brief Latest per-motor velocity measured by the PID loop, in percent.
     * @return {FL, FR, RL, RR} measured velocities in percent (nominally [-100,100], clamped to +-200
     *         internally); all zero when the PID loop is disabled.
     * @note Lock-free atomic read; safe from any thread. Handy for telemetry/logging.
     * @see set_motor, get_motor_encoder
     */
    std::array<double, 4> get_pid_measured() const {
        return {
            pid_measured_[0].load(std::memory_order_relaxed),
            pid_measured_[1].load(std::memory_order_relaxed),
            pid_measured_[2].load(std::memory_order_relaxed),
            pid_measured_[3].load(std::memory_order_relaxed)
        };
    }

    // ── Protocol / car-type constants ─────────────────────────────────────
    /**
     * @name Protocol function codes (FUNC_*)
     * @brief One-byte command/report identifiers, wire-compatible with Yahboom V3.3.9.
     *
     * Each value is the `func` byte of an outgoing frame
     * (HEAD DEVICE_ID len FUNC params... checksum) and, for the reporting codes, the
     * `ext_type` of an incoming auto-report frame. Ranges: 0x01-0x0F reporting & state,
     * 0x10-0x15 motion/motor/PID/car-type, 0x20-0x24 UART bus-servo & arm,
     * 0x30-0x31 Ackermann steering, 0x50-0xA0 request-data/version/flash-reset.
     * FUNC_REPORT_ENCODER (0x0D) carries the four int32 cumulative wheel counters the
     * software PID integrates; FUNC_MOTOR (0x10) is the raw per-wheel command frame.
     * @{
     */
    static constexpr uint8_t FUNC_AUTO_REPORT      = 0x01;
    static constexpr uint8_t FUNC_BEEP             = 0x02;
    static constexpr uint8_t FUNC_PWM_SERVO        = 0x03;
    static constexpr uint8_t FUNC_PWM_SERVO_ALL    = 0x04;
    static constexpr uint8_t FUNC_RGB              = 0x05;
    static constexpr uint8_t FUNC_RGB_EFFECT       = 0x06;
    // Speed of the four motors, in mm/s on the wire (int16 x4, little-endian).
    // RAW is computed straight from the encoders; LPF is the same value after a
    // first-order IIR in the firmware (alpha = 0.2 at a 10 ms sampling period, so
    // tau ~= 40 ms, cutoff ~4 Hz).
    // Request-only: unlike FUNC_REPORT_* below, these are NEVER auto-reported. They
    // are pulled with FUNC_REQUEST_DATA, exactly like FUNC_VERSION.
    // Requires firmware >= V3.5.1 with the motor-speed report; older boards simply
    // never answer, which is why the getters return std::nullopt on timeout.
    static constexpr uint8_t FUNC_REPORT_MOTOR_RAW = 0x08;
    static constexpr uint8_t FUNC_REPORT_MOTOR_LPF = 0x09;
    static constexpr uint8_t FUNC_REPORT_SPEED     = 0x0A;
    static constexpr uint8_t FUNC_REPORT_MPU_RAW   = 0x0B;
    static constexpr uint8_t FUNC_REPORT_IMU_ATT   = 0x0C;
    static constexpr uint8_t FUNC_REPORT_ENCODER   = 0x0D;
    static constexpr uint8_t FUNC_REPORT_ICM_RAW   = 0x0E;
    static constexpr uint8_t FUNC_RESET_STATE      = 0x0F;
    static constexpr uint8_t FUNC_MOTOR            = 0x10;
    static constexpr uint8_t FUNC_CAR_RUN          = 0x11;
    static constexpr uint8_t FUNC_MOTION           = 0x12;
    static constexpr uint8_t FUNC_SET_MOTOR_PID    = 0x13;
    static constexpr uint8_t FUNC_SET_YAW_PID      = 0x14;
    static constexpr uint8_t FUNC_SET_CAR_TYPE     = 0x15;
    static constexpr uint8_t FUNC_UART_SERVO       = 0x20;
    static constexpr uint8_t FUNC_UART_SERVO_ID    = 0x21;
    static constexpr uint8_t FUNC_UART_SERVO_TORQUE= 0x22;
    static constexpr uint8_t FUNC_ARM_CTRL         = 0x23;
    static constexpr uint8_t FUNC_ARM_OFFSET       = 0x24;
    static constexpr uint8_t FUNC_AKM_DEF_ANGLE    = 0x30;
    static constexpr uint8_t FUNC_AKM_STEER_ANGLE  = 0x31;
    static constexpr uint8_t FUNC_REQUEST_DATA     = 0x50;
    static constexpr uint8_t FUNC_VERSION          = 0x51;
    static constexpr uint8_t FUNC_RESET_FLASH      = 0xA0;

    /// @}
    /**
     * @name Car-type identifiers (CARTYPE_*)
     * @brief `param` values for FUNC_SET_CAR_TYPE selecting the board's kinematics.
     *
     * MecaMate is a 4-wheel mecanum platform and uses CARTYPE_X3. X3_PLUS, X1 and R2
     * are other Yahboom chassis kept only for protocol compatibility.
     * @{
     */
    static constexpr uint8_t CARTYPE_X3      = 0x01;
    static constexpr uint8_t CARTYPE_X3_PLUS = 0x02;
    static constexpr uint8_t CARTYPE_X1      = 0x04;
    static constexpr uint8_t CARTYPE_R2      = 0x05;

private:
    // ── Protocol constants ────────────────────────────────────────────────
    /// @}
    /**
     * @brief Outgoing-frame framing constants (Yahboom protocol).
     *
     * HEAD (0xFF) is the start-of-frame byte and DEVICE_ID (0xFC) the target board
     * address, both prepended to every command. COMPLEMENT (=257-DEVICE_ID=5) is the
     * additive constant folded into the outgoing checksum,
     * checksum = (COMPLEMENT + sum(bytes)) & 0xFF, so the C++ checksum matches the
     * board exactly. Incoming report frames instead use DEVICE_ID-1 (0xFB) as their
     * address byte.
     */
    static constexpr uint8_t HEAD       = 0xFF;
    static constexpr uint8_t DEVICE_ID  = 0xFC;
    static constexpr int     COMPLEMENT = 257 - DEVICE_ID;   // = 5
    /**
     * @brief Miscellaneous protocol magic numbers.
     *
     * CAR_ADJUST (0x80) is the mode/adjust flag byte for the closed-loop motion
     * command variant. AKM_SERVO_ID (0x01) is the bus-servo id of the Ackermann
     * steering servo addressed by the FUNC_AKM_* commands. Both are unused on a pure
     * mecanum MecaMate but kept for protocol parity.
     */
    static constexpr uint8_t CAR_ADJUST = 0x80;
    static constexpr uint8_t AKM_SERVO_ID = 0x01;

    // ── PID loop tuning constants ─────────────────────────────────────────
    // α = 0.8  →  τ_eq = α/(1−α) × dt = 0.8/0.2 × (1/kPidHz) ≈ 40 ms @ 25 Hz
    /// EMA smoothing factor for the PID error filter (dimensionless, 0..1).
    /// alpha=0.8 gives an equivalent time constant tau = alpha/(1-alpha)*dt ~= 40 ms
    /// at kPidHz=25. Higher = smoother D term but more phase lag. Compile-time constant.
    static constexpr double kDerivAlpha = 0.8;
    /// Software velocity-PID / pidLoop() tick rate in Hz (25 Hz -> 40 ms period).
    /// kPidPeriod (next line) is that same interval as a std::chrono duration used to
    /// pace the PID thread. Compile-time constants; changing kPidHz rescales dt everywhere.
    static constexpr int    kPidHz      = 25;
    static constexpr auto   kPidPeriod  = std::chrono::microseconds(1'000'000 / kPidHz);
    // kVelWindow: ring-buffer depth for velocity estimation.
    // Encoder packets arrive at ~24.4 Hz (≈41 ms). At kPidHz=25 (dt≈40 ms),
    // kVelWindow=10 covers ≈10 × 41 ms = 410 ms of real encoder data.
    // window_dt is computed from real timestamps (v6), so the 24.4 vs 25 Hz
    // mismatch no longer introduces a bias.
    // Minimum for aliasing suppression: ceil(2 × 41/40) = 3. 10 is comfortable.
    // Recalculate if kPidHz or packet rate changes significantly.
    /// Depth of the encoder ring buffer (enc_history_) used to estimate wheel velocity
    /// in pidLoop(). 10 slots ~= 410 ms of history at the ~24.4 Hz encoder report rate:
    /// wide enough to suppress aliasing, short enough to stay responsive. Compile-time
    /// constant; recompute if kPidHz or the packet rate changes. See enc_history_.
    static constexpr int    kVelWindow  = 10;

    // ── Software PID — structs ────────────────────────────────────────────
    //
    // IMPORTANT: these structs MUST be declared before any member variable
    // that uses them (motor_gains_, motor_state_). C++ requires types to be
    // complete at the point of member declaration, even within the same class.
    //
    // PidGains      : shared between threads → protected by pid_gains_mutex_
    // MotorPidState : exclusive to the PID thread → no mutex required
    // FeedforwardGains : shared between threads → protected by ff_mutex_
    //
    // Control law (v8 — feedforward + sign-correct PID for both directions):
    //   measured[i]  = signed(delta_ticks) / window_dt / scale × 100   [%]
    //   meas_filt[i] = α·meas_filt_prev + (1−α)·measured               [EMA signal]
    //   error        = target − measured                                 [signed]
    //   err_filt[i]  = α·err_filt_prev  + (1−α)·error                  [EMA error]
    //   d_term       = kd · (err_filt − err_filt_prev) / dt             [+: error grows → push]
    //   ff_term      = kS[i]·sign(target) + kV[i]·target                [feedforward]
    //   raw_cmd      = ff_term + kp·error + ki·∫ + d_term
    //   anti-windup  : integrate if |raw_cmd|<100 OR sign(error)≠sign(∫) [allow wind-down]
    //   cmd          = clamp(raw_cmd, −100, 100)
    /**
     * @brief Velocity-PID gains for one motor (or the global default).
     *
     * kp/ki/kd act on velocity error expressed in percent of full speed (both target
     * and measured are in [-100,100] %). Shared between the app/ROS2 thread (setters)
     * and the PID thread (per-tick reader), so every access goes through
     * pid_gains_mutex_. Defaults kp=1.8, ki=0.4, kd=0.05 are the tuned MecaMate values.
     */
    struct PidGains {
        double kp{1.8};
        double ki{0.4};
        double kd{0.05};
    };

    /**
     * @brief Per-motor integrator/filter state carried between PID ticks.
     *
     * integral is the anti-windup-clamped error integral (+-80), measured_filtered is
     * an EMA of measured velocity kept only for display, and error_filtered is the EMA
     * of error that feeds the sign-correct D term. These live entirely inside pidLoop()
     * on the PID thread, so no lock is needed. reset() zeroes all three (called when
     * PID is (re)enabled or a wheel is stopped).
     */
    struct MotorPidState {
        double integral{0.0};
        double measured_filtered{0.0};   // EMA on measured velocity (for display)
        double error_filtered{0.0};      // EMA on error (for D term — sign-correct)

        void reset() {
            integral = measured_filtered = error_filtered = 0.0;
        }
    };

    // Per-motor feedforward model: pwm = kS·sign(target) + kV·target.
    // kS absorbs static friction / dead zone (PWM% needed just to start
    // moving). kV is the linear PWM-per-%-speed slope above the dead zone,
    // measured under whatever load the calibration sweep was run with.
    /**
     * @brief Per-motor open-loop feedforward model: pwm = kS*sign(target) + kV*target.
     *
     * kS (static/Coulomb term, PWM %) absorbs the dead zone -- the minimum command
     * needed to break stiction; kV (PWM % per % speed) is the linear slope above the
     * dead zone. Both are produced by calibrate_feedforward(). Defaults kS=0, kV=1 pass
     * the target straight through, reproducing the pre-feedforward v7 behaviour. Shared
     * with the PID thread, so guarded by ff_mutex_.
     */
    struct FeedforwardGains {
        double kS{0.0};
        double kV{1.0};   // 1.0 == previous behaviour (target passed through 1:1)
    };

    // ── Internal helpers ──────────────────────────────────────────────────
    /**
     * @brief Compute the 1-byte checksum of an outgoing command frame.
     *
     * Returns (COMPLEMENT + sum(cmd bytes)) & 0xFF over the frame content (everything
     * except the checksum byte itself), matching the board's algorithm.
     * @param cmd  the frame bytes built so far (HEAD..last param), checksum not yet appended.
     * @return the checksum byte to append.
     * @note Pure/const; callable from any thread.
     */
    uint8_t checksum(const std::vector<uint8_t> & cmd) const;
    /**
     * @brief Append the checksum and write a fully-formed command frame to the UART.
     *
     * Fills in the len/checksum and hands the bytes to ser_.
     * @param cmd  frame without its trailing checksum; the checksum is appended in place.
     * @note Usually called from the app/ROS2 thread; OS serializes the write. Silently
     *       does nothing useful if the port is closed.
     */
    void    writeCmd(std::vector<uint8_t> & cmd);
    /**
     * @brief Send a FUNC_REQUEST_DATA frame asking the board to report one datum.
     *
     * Used to pull one-shot values (version, car type, servo angle...) that are not
     * part of the periodic auto-report stream; the answer arrives asynchronously via
     * receiveLoop()/parseData(). Includes a short post-write delay.
     * @param function  the FUNC_* code whose data is requested.
     * @param param     optional sub-selector (e.g. servo id); 0 if unused.
     */
    void    requestData(uint8_t function, uint8_t param = 0);
    /**
     * @brief Decode one validated auto-report payload into the atomic sensor cache.
     *
     * Dispatches on ext_type (a FUNC_REPORT_* /FUNC_* code) and writes decoded
     * sensor/readback values into the atomic members (velocities, IMU, encoders,
     * battery, version, arm angles...). Sets encoder_received_ on the first encoder frame.
     * @param ext_type  the report type byte from the incoming frame.
     * @param d  the little-endian payload bytes (checksum already verified).
     * @note Runs only on recv_thread_; writers are atomics, except read_arm_ (arm_mutex_).
     */
    void    parseData(uint8_t ext_type, const std::vector<uint8_t> & ext_data);
    /**
     * @brief Body of recv_thread_: continuously read the UART and parse report frames.
     *
     * Loops while uart_running_ is true, reassembling HEAD/0xFB-framed auto-report
     * packets, verifying their checksum, and calling parseData(). Implements the
     * 1500 ms silence watchdog and reconnect logic, and clears encoder_received_ on a
     * reconnect.
     * @note The sole writer of the sensor atomics. Started by create_receive_threading()
     *       and joined by the destructor after uart_running_ is cleared.
     * @warning Must be joined before ser_.close() so it never touches a closed fd.
     */
    void    receiveLoop();
    /**
     * @brief Clamp and round a motor command to the wire range.
     * @param v  desired command in percent [-100,100] (values outside are clamped).
     * @return the command as an int8_t in [-100,100] for a FUNC_MOTOR frame.
     * @note Pure/const.
     */
    int8_t  limitMotorValue(double v) const;
    /**
     * @brief Convert a bus-servo angle (degrees) to its raw controller value.
     *
     * Maps a human angle to the servo's raw pulse units using the per-servo range of
     * the Yahboom arm. Inverse of armConvertAngle().
     * @param s_id     servo id (1..6) selecting the conversion range.
     * @param s_angle  target angle in degrees.
     * @return the raw servo value to transmit.
     */
    int     armConvertValue(int s_id, int s_angle) const;
    /**
     * @brief Convert a raw bus-servo value back to an angle in degrees.
     *
     * Inverse of armConvertValue(); used when decoding FUNC_ARM_CTRL readbacks.
     * @param s_id     servo id (1..6) selecting the conversion range.
     * @param s_value  raw controller value as reported by the board.
     * @return the angle in degrees.
     */
    int     armConvertAngle(int s_id, int s_value) const;

    // ── Software PID internal methods ─────────────────────────────────────
    /**
     * @brief Body of pid_thread_: run the per-motor velocity PID at kPidHz (v8 law).
     *
     * Each 40 ms tick it snapshots the encoder atomics into enc_history_, computes a
     * timestamp-based velocity, runs the control law per motor (EMA-filtered error, D
     * term, kS/kV feedforward, wind-down-aware anti-windup), and issues the result via
     * writeMotorRaw(). Publishes measured velocity into pid_measured_.
     * @note Reads pid_gains_/motor_gains_ under pid_gains_mutex_ and ff_gains_ under
     *       ff_mutex_ (one snapshot per tick); motor_state_ and enc_history_ are
     *       thread-private. Started by enable_pid_control(), joined by disable_pid_control().
     */
    void pidLoop();
    /**
     * @brief Send a raw four-wheel command frame, bypassing the software PID.
     *
     * Applies slope compensation / headroom limiting, clamps to [-100,100] %, and emits
     * a FUNC_MOTOR frame. Called by pidLoop() every tick with the PID output, and
     * directly by the calibration sweeps with known PWM steps.
     * @param cmd  per-wheel commands {FL,FR,RL,RR} in percent [-100,100].
     * @note Normally invoked from the PID thread; also from calibration on the app thread.
     */
    void writeMotorRaw(const std::array<double, 4> & cmd);

    // Feedforward evaluation, shared by pidLoop() and calibrate_feedforward()
    // diagnostics. Not used for the calibration sweep itself (which drives
    // writeMotorRaw directly with known PWM steps).
    /**
     * @brief Evaluate the feedforward term for motor i at a given target.
     *
     * Returns kS*sign(target) + kV*target, or exactly 0 when target==0 so a stopped
     * wheel never fights static friction. Used by pidLoop() and by
     * calibrate_feedforward() diagnostics (not by the calibration sweep itself).
     * @param i       motor index 0..3 (FL,FR,RL,RR).
     * @param target  desired speed in percent [-100,100].
     * @return the feedforward command contribution in percent.
     * @note Reads the shared ff_gains_ member; callers must hold ff_mutex_ (or use the
     *       pidLoop per-tick snapshot) for thread-safety.
     */
    inline double feedforwardOf(int i, double target) const {
        const FeedforwardGains & g = ff_gains_[i];
        if (target == 0.0) return 0.0;
        const double sign = (target > 0.0) ? 1.0 : -1.0;
        return g.kS * sign + g.kV * target;
    }

    // ── Slope compensation helpers ─────────────────────────────────────────
    /**
     * @brief Clamp a command to the flat-ground PWM ceiling, reserving slope headroom.
     *
     * Limits |cmd| to max_pwm_flat_ so apply_slope() can later scale the command up on
     * inclines without saturating past 100%.
     * @param cmd  command in percent.
     * @return the clamped command in percent.
     */
    inline double reserve_headroom(double cmd) const {
        return std::clamp(cmd, -max_pwm_flat_, max_pwm_flat_);
    }

    /**
     * @brief Scale a command by the gravity-compensation factor on inclines.
     *
     * When slope compensation is enabled and a fresh pitch reading exists (younger than
     * pitch_timeout_ns_) and exceeds deadband_rad_, multiplies cmd by
     * 1 + k_gravity_*sin(pitch) and clamps to [-100,100]; otherwise returns cmd
     * unchanged. Reads pitch_rad_ and last_pitch_time_ns_ atomically.
     * @param cmd  command in percent.
     * @return the possibly-boosted command in percent.
     */
    inline double apply_slope(double cmd) const {
        if (!slope_comp_enabled_) return cmd;
        const uint64_t now_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        if (now_ns - last_pitch_time_ns_.load(std::memory_order_relaxed)
                > pitch_timeout_ns_) {
            return cmd;
        }
        const double pitch = pitch_rad_.load(std::memory_order_relaxed);
        if (std::abs(pitch) < deadband_rad_) return cmd;
        const double factor = 1.0 + k_gravity_ * std::sin(pitch);
        return std::clamp(cmd * factor, -100.0, 100.0);
    }

    // ── Little-endian pack/unpack helpers ─────────────────────────────────
    /**
     * @brief Read a signed little-endian 16-bit integer from a payload.
     * @param d    payload bytes.
     * @param off  byte offset of the low byte.
     * @return the decoded int16_t.
     */
    static int16_t le16s(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off + 1]) << 8));
    }
    /**
     * @brief Read an unsigned little-endian 16-bit integer from a payload.
     * @param d    payload bytes.
     * @param off  byte offset of the low byte.
     * @return the decoded uint16_t.
     */
    static uint16_t le16u(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off + 1]) << 8));
    }
    /**
     * @brief Read a signed little-endian 32-bit integer from a payload.
     *
     * Used to decode the four cumulative encoder counters of a FUNC_REPORT_ENCODER frame.
     * @param d    payload bytes.
     * @param off  byte offset of the least-significant byte.
     * @return the decoded int32_t.
     */
    static int32_t le32s(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<int32_t>(
            static_cast<uint32_t>(d[off])              |
            (static_cast<uint32_t>(d[off + 1]) <<  8)  |
            (static_cast<uint32_t>(d[off + 2]) << 16)  |
            (static_cast<uint32_t>(d[off + 3]) << 24));
    }
    /**
     * @brief Split a signed 16-bit value into its little-endian byte pair.
     * @param v  value to encode.
     * @return {low byte, high byte} for insertion into an outgoing frame.
     */
    static std::pair<uint8_t, uint8_t> packI16(int16_t v) {
        return { static_cast<uint8_t>(v & 0xff),
                 static_cast<uint8_t>((v >> 8) & 0xff) };
    }

    // ── Serial port ───────────────────────────────────────────────────────
    /// Owning handle to the POSIX serial port (115200 8N1, VMIN=0/VTIME=5).
    /// Opened by the ctor; written by the app thread, read by recv_thread_ (OS
    /// serializes I/O). Closed by the destructor only after both threads are joined.
    /// port_name_ (next line) caches the device path for reopen/reconnect.
    SerialPort  ser_;
    std::string port_name_;

    // ── Config ────────────────────────────────────────────────────────────
    /// delay_time_: default inter-command settle delay in seconds (app-thread config).
    /// debug_ (next): when true, frames are dumped to stdout for protocol debugging.
    /// car_type_ (next): cached CARTYPE_* selecting the board kinematics (default X3).
    /// All three are plain construction-time config, not shared across threads.
    double  delay_time_;
    bool    debug_;
    uint8_t car_type_;

    // ── Background receive thread ─────────────────────────────────────────
    // Shutdown sequence (guaranteed by destructor):
    //   1. uart_running_ = false
    //   2. recv_thread_.join()   ← waits for thread exit
    //   3. ser_.close()          ← fd closed AFTER thread is gone
    /// recv_thread_: the background thread running receiveLoop(), created by
    /// create_receive_threading(). uart_running_ (next) is its atomic run flag: set true
    /// to start, cleared by the destructor so the loop exits before ser_.close().
    std::thread       recv_thread_;
    std::atomic<bool> uart_running_{false};

    // ── Sensor cache (atomic for lock-free reads) ─────────────────────────
    /// Linear acceleration {ax,ay,az} in units of g, from FUNC_REPORT_MPU_RAW/ICM_RAW.
    /// Part of the lock-free sensor cache: written only by recv_thread_ (parseData),
    /// read by app-thread getters; each field is a std::atomic<double>.
    std::atomic<double> ax_{0}, ay_{0}, az_{0};
    /// Angular rate {gx,gy,gz} in rad/s (raw * 1/3754.9 for MPU); the next line holds
    /// magnetometer {mx,my,mz} in raw/scaled counts. Same lock-free cache: written by
    /// recv_thread_ in parseData(), read by getters. Some axes are sign-flipped to the
    /// robot frame.
    std::atomic<double> gx_{0}, gy_{0}, gz_{0};
    std::atomic<double> mx_{0}, my_{0}, mz_{0};
    /// Body-frame velocities from FUNC_REPORT_SPEED (raw/1000): vx,vy in m/s (linear
    /// x,y), vz in rad/s (yaw rate). Written by recv_thread_, read by get_motion_data()
    /// on the app thread; lock-free atomics.
    std::atomic<double> vx_{0}, vy_{0}, vz_{0};
    /// Fused IMU attitude {roll,pitch,yaw} in radians (raw/10000) from FUNC_REPORT_IMU_ATT.
    /// get_imu_attitude_data() can convert to degrees on request. Written by recv_thread_,
    /// read on the app thread; lock-free atomics. (Distinct from the slope-comp pitch_rad_.)
    std::atomic<double> roll_{0}, pitch_{0}, yaw_{0};
    /// Cumulative wheel encoder counts {m1..m4} = {FL,FR,RL,RR}, signed int32 ticks that
    /// never reset (deltas are taken modulo 2^32 to survive wraparound). Written by
    /// recv_thread_ from FUNC_REPORT_ENCODER; read by the PID thread and app getters.
    std::atomic<int>    encoder_m1_{0}, encoder_m2_{0},
                        encoder_m3_{0}, encoder_m4_{0};
    /// Battery voltage in decivolts (tenths of a volt); get_battery_voltage() returns
    /// /10.0 as volts. Written by recv_thread_ from FUNC_REPORT_SPEED, read on the app thread.
    std::atomic<int>    battery_voltage_{0};

    /// Per-wheel speed {FL,FR,RL,RR} in m/s, from the request-only FUNC_REPORT_MOTOR_RAW
    /// (straight from the encoders) and FUNC_REPORT_MOTOR_LPF (firmware first-order IIR).
    /// The wire carries mm/s; parseData() divides by 1000 so the unit matches
    /// get_motion_data(), which is what the rest of this driver exposes.
    /// motor_speed_*_ok_ are the request/reply arrival sentinels, cleared by the getter
    /// before it sends and release-stored by recv_thread_ once the frame is decoded.
    /// All lock-free atomics: recv_thread_ writes, the app thread reads.
    std::array<std::atomic<double>, 4> motor_speed_raw_{};
    std::array<std::atomic<double>, 4> motor_speed_lpf_{};
    std::atomic<bool>   motor_speed_raw_ok_{false};
    std::atomic<bool>   motor_speed_lpf_ok_{false};

    /// Last bus-servo readback: read_id_ = servo id, read_val_ = raw position value from
    /// a FUNC_UART_SERVO reply. Written by recv_thread_, polled by the app thread; atomics.
    std::atomic<int>    read_id_{0}, read_val_{0};

    /// Arm servo readback (FUNC_ARM_CTRL): read_arm_[6] holds the six joint angles and is
    /// the one non-atomic sensor field, so it is guarded by arm_mutex_ (written by
    /// recv_thread_, read by the app thread). read_arm_ok_ is an atomic ready-flag
    /// (1 = a fresh set has been latched, 0 = waiting).
    mutable std::mutex  arm_mutex_;
    std::atomic<int>    read_arm_ok_{0};
    int                 read_arm_[6] = {-1,-1,-1,-1,-1,-1};

    /// Firmware version bytes from FUNC_VERSION: version_H_ (major) and version_L_
    /// (minor), written atomically by recv_thread_. version_ (next line) is the combined
    /// major.minor value assembled on the app thread once the bytes arrive.
    std::atomic<uint8_t> version_H_{0}, version_L_{0};
    double               version_{0};

    /// Hardware-PID readback (echo of FUNC_SET_MOTOR_PID / FUNC_SET_YAW_PID):
    /// pid_index_ is the addressed channel and kp1_/ki1_/kd1_ (next line) the raw int16
    /// gains the board holds. Written by recv_thread_, read on the app thread. Distinct
    /// from the software PID (pid_gains_) this driver actually runs.
    std::atomic<int>     pid_index_{0};
    std::atomic<int16_t> kp1_{0}, ki1_{0}, kd1_{0};

    /// Miscellaneous board command-readback state (written by recv_thread_, read by the
    /// app thread unless noted):
    ///  - arm_offset_id_/arm_offset_state_: result of a FUNC_ARM_OFFSET calibration.
    ///  - arm_ctrl_enable_ (next): app-only bool gating arm-control writes.
    ///  - akm_def_angle_/akm_readed_angle_: Ackermann steering default angle and its
    ///    "value has been read" flag.
    ///  - read_car_type_: car-type echo from FUNC_SET_CAR_TYPE.
    /// All the readback fields are atomics.
    std::atomic<int>     arm_offset_id_{0}, arm_offset_state_{0};
    bool                 arm_ctrl_enable_{true};

    std::atomic<int>     akm_def_angle_{100};
    std::atomic<bool>    akm_readed_angle_{false};

    std::atomic<int>     read_car_type_{0};

    // ── Software PID — members ────────────────────────────────────────────

    // Shared gains — always accessed under pid_gains_mutex_
    /// Global software-PID gains (the default for every motor). pid_gains_ is shared
    /// between the app thread (setters) and the PID thread (per-tick snapshot reader),
    /// so every access is serialized by pid_gains_mutex_ declared here.
    mutable std::mutex pid_gains_mutex_;
    PidGains           pid_gains_{};

    // Per-motor PID gain overrides (FL, FR, RL, RR)
    // motor_gains_override_[i]==true  → motor i uses motor_gains_[i]
    // motor_gains_override_[i]==false → motor i uses global pid_gains_
    /// Optional per-motor {FL,FR,RL,RR} PID gain overrides. When
    /// motor_gains_override_[i] is true, motor i uses motor_gains_[i]; otherwise it falls
    /// back to the global pid_gains_. Both arrays are shared with the PID thread and
    /// protected by pid_gains_mutex_.
    std::array<PidGains, 4> motor_gains_{};
    std::array<bool, 4>     motor_gains_override_{false, false, false, false};

    // Per-motor state — exclusive to the PID thread, no mutex needed
    /// Per-motor integrator/EMA state ({FL,FR,RL,RR}). Owned exclusively by the PID
    /// thread (pidLoop), so no lock is needed. See MotorPidState.
    std::array<MotorPidState, 4> motor_state_{};

    // ── Feedforward — members (v8) ─────────────────────────────────────────
    // Shared gains — accessed under ff_mutex_. Read once per pidLoop()
    // iteration (snapshot pattern, same as pid_gains_/motor_gains_).
    /// Feedforward state. ff_gains_[4] holds the per-motor kS/kV model and is shared with
    /// the PID thread, so it is read under ff_mutex_ (one snapshot per pidLoop tick).
    /// feedforward_enabled_ (atomic) gates whether the ff term is applied;
    /// feedforward_calibrated_ (plain app-thread bool) records that calibrate_feedforward()
    /// has run.
    mutable std::mutex             ff_mutex_;
    std::array<FeedforwardGains,4> ff_gains_{};
    std::atomic<bool>              feedforward_enabled_{false};
    bool                            feedforward_calibrated_{false};

    // Setpoints deposited by set_motor(), consumed by pidLoop()
    /// Per-motor velocity setpoints {FL,FR,RL,RR} in percent [-100,100]. Deposited by
    /// set_motor() on the app thread, consumed each tick by pidLoop(); lock-free atomics.
    std::array<std::atomic<double>, 4> target_{};

    // Per-motor velocity scales: ticks/s at 100% cmd.
    // 0.0 → pidLoop() falls back to ticks_per_second_at_100pct_ (global).
    /// Per-motor velocity scale in encoder ticks/s at 100% command, used to convert a
    /// measured tick-rate into percent. 0.0 means "use the global
    /// ticks_per_second_at_100pct_". Filled by calibrate_motor_scales(): written on the
    /// app thread, read by the PID thread (steady after calibration).
    std::array<double, 4> motor_scale_{0.0, 0.0, 0.0, 0.0};

    // Reference encoder values (kept for calibrate_pid_scale compatibility)
    /// Legacy per-motor reference encoder snapshot kept only for calibrate_pid_scale()
    /// compatibility; not used by the timestamp-based velocity estimator. Owned by the
    /// calibration/app path.
    std::array<uint32_t, 4> enc_prev_pid_{0u, 0u, 0u, 0u};

    // Sliding window for velocity estimation in pidLoop().
    // Each slot stores the four encoder counts AND the wall-clock timestamp
    // (microseconds since epoch) at which they were sampled.
    // window_dt = (ts_us of newest slot - ts_us of oldest slot) / 1e6
    // This eliminates the ~2.5% bias from 24.4 Hz encoders vs 25 Hz PID.
    /**
     * @brief One ring-buffer sample: the four encoder counts plus their timestamp.
     *
     * enc holds the raw uint32 counters {FL,FR,RL,RR} at sample time; ts_us is the
     * steady_clock timestamp in microseconds. pidLoop() derives velocity from the
     * difference between the newest and oldest slot using the real elapsed time
     * (window_dt), which removes the 24.4 Hz-encoder vs 25 Hz-PID timing bias.
     */
    struct EncSlot {
        std::array<uint32_t, 4> enc{};
        int64_t ts_us{0};   // std::chrono::steady_clock microseconds
    };
    /// Sliding window of the last kVelWindow EncSlot samples used for velocity
    /// estimation. enc_history_idx_ (next) is the next write position and
    /// enc_history_full_ (next) marks that the buffer has wrapped (all slots valid).
    /// All three are private to the PID thread; no lock required.
    std::array<EncSlot, kVelWindow> enc_history_{};
    int  enc_history_idx_{0};
    bool enc_history_full_{false};

    // Set by parseData() on first valid FUNC_REPORT_ENCODER packet.
    // enable_pid_control() refuses to start until this flag is true.
    // Reset to false by receiveLoop() on each reconnection.
    /// Set true by parseData() on the first valid FUNC_REPORT_ENCODER frame and cleared
    /// by receiveLoop() on each reconnect. enable_pid_control() refuses to start until it
    /// is true, so the PID never runs on stale/zero encoders. Atomic.
    std::atomic<bool> encoder_received_{false};

    // Global velocity scale: ticks/s at 100% cmd.
    // Default: 1000 ticks/rev × (0.5 m/s ÷ π×0.12 m) ≈ 1326 ticks/s
    /// Global fallback velocity scale in encoder ticks/s at 100% command, used for any
    /// motor whose motor_scale_ is still 0. Default ~1326 ticks/s (1000 ticks/rev at
    /// ~0.5 m/s on 0.12 m wheels); overwritten by calibration. App-thread config read by
    /// the PID thread.
    double ticks_per_second_at_100pct_{1326.0};

    // True once calibrate_pid_scale() has completed successfully.
    /// True once calibrate_pid_scale() has completed successfully. App-thread flag.
    bool pid_scale_calibrated_{false};

    // pid_enabled_: atomic — set_motor() reads it from the ROS2 thread
    /// Master enable for the software-PID path. set_motor() reads it on the app thread to
    /// decide whether to feed target_ (PID on) or drive writeMotorRaw() directly (PID
    /// off). Atomic; toggled by enable_/disable_pid_control().
    std::atomic<bool> pid_enabled_{false};

    /// pid_thread_ runs pidLoop() at kPidHz. pid_running_ (next) is its atomic run flag:
    /// enable_pid_control() sets it true and starts the thread; disable_pid_control()
    /// clears it and joins this thread FIRST (before recv_thread_), since pidLoop reads
    /// atomics that receiveLoop populates.
    std::thread       pid_thread_;
    std::atomic<bool> pid_running_{false};

    // Last measured velocity per motor (%), written by pidLoop()
    /// Last measured wheel velocity {FL,FR,RL,RR} in percent, published by pidLoop() each
    /// tick and exposed via get_pid_measured(); {0,0,0,0} while PID is disabled. Lock-free
    /// atomics (PID thread writes, app thread reads).
    std::array<std::atomic<double>, 4> pid_measured_{};

    // ── Slope compensation ────────────────────────────────────────────────
    /// Slope-compensation state (used by apply_slope()/reserve_headroom()):
    ///  - pitch_rad_: latest chassis pitch in radians, with last_pitch_time_ns_ its
    ///    steady_clock timestamp (ns) -- both atomic, set by update_pitch() from any
    ///    thread and read by the PID thread.
    ///  - slope_comp_enabled_ / k_gravity_ / deadband_rad_ / max_pwm_flat_ /
    ///    pitch_timeout_ns_: config -- enable flag, gravity gain, min pitch to act on
    ///    (rad), flat-ground PWM ceiling (%), and max reading staleness (ns) before
    ///    compensation is ignored. Set on the app thread via
    ///    configure_slope_compensation()/set_max_pwm_flat().
    std::atomic<double>   pitch_rad_{0.0};
    std::atomic<uint64_t> last_pitch_time_ns_{0};
    bool     slope_comp_enabled_{false};
    double   k_gravity_{0.80};
    double   deadband_rad_{0.03};
    double   max_pwm_flat_{70.0};
    uint64_t pitch_timeout_ns_{200'000'000ULL};
};

// =============================================================================
//  SerialPort implementation
// =============================================================================

#ifdef _WIN32
// ── Windows ──────────────────────────────────────────────────────────────────
inline bool SerialPort::open(const std::string & port, int baud) {
    /**
     * @brief Windows backend for SerialPort::open() — opens the COM device via
     *        the Win32 handle API and configures 8N1 at the requested baud.
     */
    // The "\\.\COMx" device-namespace prefix is required for COM ports >= 10.
    std::string full = "\\\\.\\" + port;
    // Open the existing device for read/write, no sharing (dwShareMode = 0).
    hSerial_ = CreateFileA(full.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0, nullptr, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hSerial_ == INVALID_HANDLE_VALUE) return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial_, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hSerial_, &dcb);

    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout        = 0;
    // 500 ms total read timeout — mirrors the POSIX VTIME=5 so the read loop
    // wakes periodically and can re-check uart_running_.
    to.ReadTotalTimeoutConstant   = 500;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant  = 1000;
    to.WriteTotalTimeoutMultiplier= 0;
    SetCommTimeouts(hSerial_, &to);
    return true;
}
inline void SerialPort::close() {
    if (hSerial_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial_);
        hSerial_ = INVALID_HANDLE_VALUE;
    }
}
inline bool SerialPort::isOpen() const { return hSerial_ != INVALID_HANDLE_VALUE; }
inline int SerialPort::write(const std::vector<uint8_t> & data) {
    DWORD written = 0;
    WriteFile(hSerial_, data.data(), static_cast<DWORD>(data.size()), &written, nullptr);
    return static_cast<int>(written);
}
inline int SerialPort::readByte(uint8_t & out) {
    DWORD rd = 0;
    // NOTE: the Windows readByte convention differs from POSIX — here a timeout
    // (rd==0) and a hard error both collapse to -1; there is no benign 0-return.
    if (!ReadFile(hSerial_, &out, 1, &rd, nullptr) || rd == 0) return -1;
    return 1;
}
inline void SerialPort::flushInput() { PurgeComm(hSerial_, PURGE_RXCLEAR); }

#else
// ── POSIX ────────────────────────────────────────────────────────────────────

inline bool SerialPort::open(const std::string & port, int baud) {
    /**
     * @brief POSIX backend for SerialPort::open() — opens the tty in raw mode
     *        (8N1 at the requested baud) and applies the FIX-1..FIX-5 hardening
     *        so a CH340/CP210x USB power-cycle or e-stop never wedges the port
     *        (the Pi never needs a reboot to recover the /dev/ttyUSBx node).
     */
    // FIX-1: O_NONBLOCK avoids blocking on VHANGUP tty session left by
    // a previous power-cycle. Cleared immediately via fcntl so that
    // normal VMIN/VTIME blocking reads still work.
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "SerialPort::open(" << port << "): " << strerror(errno) << "\n";
        return false;
    }

    // FIX-2: TIOCEXCL — exclusive kernel-level lock
    if (::ioctl(fd_, TIOCEXCL) < 0) {
        std::cerr << "SerialPort: TIOCEXCL failed (non-fatal): "
                  << strerror(errno) << "\n";
    }

    // FIX-1 cont.: revert to blocking I/O
    {
        const int flags = fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK) < 0) {
            std::cerr << "SerialPort: fcntl clear O_NONBLOCK failed: "
                      << strerror(errno) << "\n";
            ::ioctl(fd_, TIOCNXCL);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
    }

    // FIX-3: flush BEFORE tcgetattr — discards stale baud/line-discipline state
    ::tcflush(fd_, TCIOFLUSH);

    // From here on, build a fresh raw-mode termios line-discipline config.
    termios tty{};
    if (tcgetattr(fd_, &tty) < 0) {
        std::cerr << "SerialPort: tcgetattr failed: " << strerror(errno) << "\n";
        ::ioctl(fd_, TIOCNXCL);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Translate the integer baud into the termios speed_t constant (115200 default).
    speed_t speed = B115200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8 data bits: clear the character-size mask (CSIZE), then OR in CS8.
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    // FIX-4: clear HUPCL — prevents DTR/RTS drop on close() which would
    // re-enumerate the USB adapter and cause the /dev/ttyUSBx node to vanish.
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS | HUPCL);
    // CLOCAL: ignore modem-control lines (no owner hangup); CREAD: enable receiver.
    tty.c_cflag |=  CLOCAL | CREAD;
    // Raw input: ignore parity errors, no CR/NL translation or SW flow control.
    // c_oflag/c_lflag = 0 below give raw output with no canonical mode/echo/signals.
    tty.c_iflag  = IGNPAR;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;

    // VMIN=0, VTIME=5 (500 ms): readByte() returns 0 (timeout) if no byte
    // arrives within 500 ms, allowing receiveLoop() to re-check uart_running_.
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd_, TCSANOW, &tty) < 0) {
        std::cerr << "SerialPort: tcsetattr failed: " << strerror(errno) << "\n";
        ::ioctl(fd_, TIOCNXCL);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

inline void SerialPort::close() {
    /**
     * @brief POSIX backend for SerialPort::close(). Because HUPCL was cleared in
     *        open() (FIX-4), DTR/RTS stay asserted so the USB adapter does not
     *        re-enumerate and the tty node survives; FIX-5 below flushes, drops
     *        the TIOCEXCL exclusive lock (TIOCNXCL), then closes the fd.
     */
    if (fd_ < 0) return;
    // FIX-5: flush → TIOCNXCL → close
    ::tcflush(fd_, TCIOFLUSH);
    ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    fd_ = -1;
}

inline bool SerialPort::isOpen() const { return fd_ >= 0; }

inline int SerialPort::write(const std::vector<uint8_t> & data) {
    if (fd_ < 0) return -1;
    // One-shot write of the whole frame; returns bytes written, or -1 on error.
    // Partial writes are not retried — frames are only a handful of bytes at 115200.
    ssize_t n = ::write(fd_, data.data(), data.size());
    return static_cast<int>(n);
}

inline int SerialPort::readByte(uint8_t & out) {
    if (fd_ < 0) return -1;
    /**
     * @brief POSIX backend for SerialPort::readByte(). Return convention:
     *        1 = one byte stored in @p out; 0 = benign timeout (VTIME expiry or
     *        EAGAIN) so receiveLoop() simply loops again; -1 = fatal I/O error
     *        (device vanished) and the caller must stop and close/reconnect.
     */
    ssize_t n;
    do {
        n = ::read(fd_, &out, 1);
    // Retry transparently when the read is interrupted by a signal (EINTR).
    } while (n < 0 && errno == EINTR);

    if (n == 1)  return  1;
    if (n == 0)  return  0;   // VTIME timeout
    // A non-blocking would-block is treated as a timeout (0), not a fatal error.
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;   // EIO / ENOTTY / EBADF — caller must stop and close
}

/**
 * @brief POSIX backend for SerialPort::flushInput() — discards unread RX bytes
 *        (TCIFLUSH); used on reconnect to drop a stale or half-received frame.
 */
inline void SerialPort::flushInput() {
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

#endif  // platform

// =============================================================================
//  Rosmaster implementation
// =============================================================================

/**
* @brief Opens the UART (115200 8N1) and powers on bus-servo torque.
*        Throws std::runtime_error if the port cannot be opened. The
*        receive/PID threads are NOT started here — the caller drives the
*        startup order documented in the class header (create_receive_threading
*        -> set_auto_report_state -> wait encoder_received_ -> calibrate -> enable_pid).
*/
inline Rosmaster::Rosmaster(int car_type, const std::string & com,
                            double delay, bool debug)
    : port_name_(com),
      delay_time_(delay),
      debug_(debug),
      car_type_(static_cast<uint8_t>(car_type))
{
    if (!ser_.open(com, 115200))
        throw std::runtime_error("Rosmaster: serial open failed: " + com);

    std::cout << "Rosmaster Serial Opened! Baudrate=115200\n";
    if (debug_) std::cout << "cmd_delay=" << delay_time_ << "s\n";

    // Energise the bus-servo torque so held arm poses survive a controller reset.
    set_uart_servo_torque(1);
    delay_ms(2);
}

// ── Destructor ────────────────────────────────────────────────────────────────
// Shutdown order:
//   1. disable_pid_control()  → pid_running_=false → joins pid_thread_
//   2. uart_running_=false    → joins recv_thread_
//   3. ser_.close()
// pid_thread_ must be joined before recv_thread_ because pidLoop() calls
// get_motor_encoder() which reads atomics populated by receiveLoop().
inline Rosmaster::~Rosmaster() {
    disable_pid_control();

    uart_running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();

    ser_.close();
    std::cout << "Rosmaster serial closed.\n";
}

// ── checksum ──────────────────────────────────────────────────────────────────
/**
* @brief Outgoing checksum = (COMPLEMENT + sum(bytes)) & 0xFF, taken over every
*        frame byte EXCEPT the checksum itself; must match the MCU firmware.
*/
inline uint8_t Rosmaster::checksum(const std::vector<uint8_t> & cmd) const {
    // Seed with COMPLEMENT(=5): a fixed offset the firmware bakes into the sum.
    int s = COMPLEMENT;
    for (auto b : cmd) s += b;
    return static_cast<uint8_t>(s & 0xff);
}

// ── writeCmd ──────────────────────────────────────────────────────────────────
/**
* @brief Appends the checksum, writes the full frame to the UART, then sleeps
*        delay_time_ so the MCU can digest back-to-back commands. Called from
*        both the app thread and pidLoop() (writeMotorRaw) — keep it re-entrant-safe.
*/
inline void Rosmaster::writeCmd(std::vector<uint8_t> & cmd) {
    cmd.push_back(checksum(cmd));
    ser_.write(cmd);
    if (debug_) {
        std::cout << "cmd:";
        for (auto b : cmd) std::cout << " " << std::hex << static_cast<int>(b);
        std::cout << std::dec << "\n";
    }
    // Inter-command pacing (delay_time_ is seconds -> ms) so we don't overrun the MCU.
    delay_ms(delay_time_ * 1000.0);
}

// ── requestData ───────────────────────────────────────────────────────────────
/**
* @brief Sends a FUNC_REQUEST_DATA poll asking the MCU to emit one report of
*        type `function`; the reply arrives asynchronously via receiveLoop()/parseData().
*/
inline void Rosmaster::requestData(uint8_t function, uint8_t param) {
    // Fixed body: HEAD(0xFF) DEVICE_ID(0xFC) len(0x05) FUNC_REQUEST_DATA function param.
    std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05,
                                 FUNC_REQUEST_DATA, function, param};
    cmd.push_back(checksum(cmd));
    ser_.write(cmd);
    if (debug_) {
        std::cout << "request:";
        for (auto b : cmd) std::cout << " " << std::hex << static_cast<int>(b);
        std::cout << std::dec << "\n";
    }
    delay_ms(2);
}

// ── parseData ─────────────────────────────────────────────────────────────────
/**
* @brief Dispatches one checksum-validated auto-report frame by ext_type, decoding
*        its little-endian payload into the std::atomic sensor members. Runs only on
*        recv_thread_; shared arm buffer is guarded by arm_mutex_.
*/
inline void Rosmaster::parseData(uint8_t ext_type,
                                  const std::vector<uint8_t> & d) {
    // Chassis odometry: vx,vy int16 mm/s and vz int16 mrad/s (all /1000); d[6] = battery voltage byte.
    if (ext_type == FUNC_REPORT_SPEED) {
        vx_ = le16s(d, 0) / 1000.0;
        vy_ = le16s(d, 2) / 1000.0;
        vz_ = le16s(d, 4) / 1000.0;
        battery_voltage_ = d[6];
    }
    // MPU9250: int16 raw counts -> physical units via fixed sensitivity ratios (gyro, accel).
    else if (ext_type == FUNC_REPORT_MPU_RAW) {
        constexpr double gyro_ratio  = 1.0 / 3754.9;
        constexpr double accel_ratio = 1.0 / 1671.84;
        gx_ =  le16s(d,  0) * gyro_ratio;
        // Negated: MPU9250 Y/Z axes are flipped relative to the robot body frame.
        gy_ =  le16s(d,  2) * (-gyro_ratio);
        gz_ =  le16s(d,  4) * (-gyro_ratio);
        ax_ =  le16s(d,  6) * accel_ratio;
        ay_ =  le16s(d,  8) * accel_ratio;
        az_ =  le16s(d, 10) * accel_ratio;
        // Magnetometer passed through raw (x1.0): no scaling/calibration applied here.
        mx_ =  le16s(d, 12) * 1.0;
        my_ =  le16s(d, 14) * 1.0;
        mz_ =  le16s(d, 16) * 1.0;
    }
    // ICM20948: MCU pre-scales every axis by 1000, so a uniform /1000 restores the
    // float value; no axis negation needed (sensor mounted axis-aligned).
    else if (ext_type == FUNC_REPORT_ICM_RAW) {
        constexpr double gyro_ratio  = 1.0 / 1000.0;
        constexpr double accel_ratio = 1.0 / 1000.0;
        constexpr double mag_ratio   = 1.0 / 1000.0;
        gx_ = le16s(d,  0) * gyro_ratio;
        gy_ = le16s(d,  2) * gyro_ratio;
        gz_ = le16s(d,  4) * gyro_ratio;
        ax_ = le16s(d,  6) * accel_ratio;
        ay_ = le16s(d,  8) * accel_ratio;
        az_ = le16s(d, 10) * accel_ratio;
        mx_ = le16s(d, 12) * mag_ratio;
        my_ = le16s(d, 14) * mag_ratio;
        mz_ = le16s(d, 16) * mag_ratio;
    }
    // Fused attitude: int16 fixed-point -> radians (/10000) for roll, pitch, yaw.
    else if (ext_type == FUNC_REPORT_IMU_ATT) {
        roll_  = le16s(d, 0) / 10000.0;
        pitch_ = le16s(d, 2) / 10000.0;
        yaw_   = le16s(d, 4) / 10000.0;
    }
    // Four little-endian int32 cumulative tick counters (FL,FR,RL,RR) that never reset.
    else if (ext_type == FUNC_REPORT_ENCODER) {
        encoder_m1_ = le32s(d,  0);
        encoder_m2_ = le32s(d,  4);
        encoder_m3_ = le32s(d,  8);
        encoder_m4_ = le32s(d, 12);
        // Release-store unblocks the startup wait and marks the serial link "live".
        encoder_received_.store(true, std::memory_order_release);
    }
    // Raw per-wheel speed: four int16 in mm/s -> m/s. All four wheels are positive when
    // the robot drives forward (same frame of reference as get_motion_data()).
    else if (ext_type == FUNC_REPORT_MOTOR_RAW) {
        for (int i = 0; i < 4; ++i)
            motor_speed_raw_[i].store(le16s(d, i * 2) / 1000.0,
                                      std::memory_order_relaxed);
        // Release pairs with the acquire in get_motor_speed_raw(): once the flag reads
        // true, the four speeds above are guaranteed visible to the app thread.
        motor_speed_raw_ok_.store(true, std::memory_order_release);
        if (debug_) std::cout << "FUNC_REPORT_MOTOR_RAW\n";
    }
    // Same payload, but after the firmware's first-order low-pass filter.
    else if (ext_type == FUNC_REPORT_MOTOR_LPF) {
        for (int i = 0; i < 4; ++i)
            motor_speed_lpf_[i].store(le16s(d, i * 2) / 1000.0,
                                      std::memory_order_relaxed);
        motor_speed_lpf_ok_.store(true, std::memory_order_release);
        if (debug_) std::cout << "FUNC_REPORT_MOTOR_LPF\n";
    }
    // Bus-servo read-back: d[0] = servo id, le16 at d[1] = position/parameter value.
    else if (ext_type == FUNC_UART_SERVO) {
        read_id_  = d[0];
        read_val_ = le16s(d, 1);
        if (debug_) std::cout << "FUNC_UART_SERVO: " << read_id_ << " " << read_val_ << "\n";
    }
    // Six arm-servo angles copied under arm_mutex_ (the app thread may be reading read_arm_).
    else if (ext_type == FUNC_ARM_CTRL) {
        {
            std::lock_guard<std::mutex> lk(arm_mutex_);
            for (int i = 0; i < 6; ++i) read_arm_[i] = le16s(d, i * 2);
        }
        read_arm_ok_.store(1, std::memory_order_release);
        if (debug_) {
            std::cout << "FUNC_ARM_CTRL:";
            std::lock_guard<std::mutex> lk(arm_mutex_);
            for (int v : read_arm_) std::cout << " " << v;
            std::cout << "\n";
        }
    }
    // Firmware version high/low bytes (returned by get_version()).
    else if (ext_type == FUNC_VERSION) {
        version_H_ = d[0];
        version_L_ = d[1];
        if (debug_) std::cout << "FUNC_VERSION: " << static_cast<int>(version_H_)
                              << " " << static_cast<int>(version_L_) << "\n";
    }
    // Echo of the MCU's HARDWARE motor-PID gains -- distinct from this driver's software PID.
    else if (ext_type == FUNC_SET_MOTOR_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_) std::cout << "FUNC_SET_MOTOR_PID: " << pid_index_
                              << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    // Echo of the MCU's yaw-hold PID gains (index + Kp,Ki,Kd as int16).
    else if (ext_type == FUNC_SET_YAW_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_) std::cout << "FUNC_SET_YAW_PID: " << pid_index_
                              << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    // Result of a servo mid-point offset write: d[0] = servo id, d[1] = success flag.
    else if (ext_type == FUNC_ARM_OFFSET) {
        arm_offset_id_    = d[0];
        arm_offset_state_ = d[1];
        if (debug_) std::cout << "FUNC_ARM_OFFSET: " << arm_offset_id_
                              << " " << arm_offset_state_ << "\n";
    }
    // Ackermann steering neutral angle; akm_readed_angle_ lets a blocking read return.
    else if (ext_type == FUNC_AKM_DEF_ANGLE) {
        akm_def_angle_    = d[1];
        akm_readed_angle_ = true;
        if (debug_) std::cout << "FUNC_AKM_DEF_ANGLE: " << static_cast<int>(d[0])
                              << " " << akm_def_angle_ << "\n";
    }
    // MCU-confirmed car-type id (echo of the configured chassis model).
    else if (ext_type == FUNC_SET_CAR_TYPE) {
        read_car_type_ = d[0];
    }
}

// ── receiveLoop ───────────────────────────────────────────────────────────────
//
// Frame format:
//   HEAD(0xFF) DEVICE_ID-1(0xFB) ext_len ext_type data[0..n-2] checksum
//
// FIX-6: ser_.close() on fatal read errors.
// FIX-WATCHDOG: 1500 ms silence → uart_running_=false.
inline void Rosmaster::receiveLoop() {
    // Reset encoder flag on every receive session (covers reconnection).
    encoder_received_.store(false, std::memory_order_release);
    ser_.flushInput();

    // FIX-6: on any fatal read error, drop uart_running_ and close the fd so a CH340/CP210x
    // USB power-cycle cannot wedge the port -- the Pi never needs a reboot after an e-stop.
    auto fatalExit = [&](const char * reason) {
        std::cerr << "Rosmaster[" << port_name_ << "]: " << reason
                  << " — closing port and exiting receive thread.\n";
        uart_running_ = false;
        ser_.close();   // FIX-6
    };

    constexpr int kSilenceTimeoutMs = 1500;
    auto last_valid_head = std::chrono::steady_clock::now();

    while (uart_running_.load(std::memory_order_relaxed)) {

        // Hunt for frame start: read one byte and test it against HEAD(0xFF).
        uint8_t head1 = 0;
        const int r1 = ser_.readByte(head1);
        if (r1 < 0) { fatalExit("serial read error (EIO/ENOTTY?) on HEAD byte"); return; }
        if (r1 == 0 || head1 != HEAD) {
            const auto now = std::chrono::steady_clock::now();
            const auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_valid_head).count();
            // Silence watchdog: >1500 ms with no valid HEAD => MCU offline/e-stopped -> clean close.
            if (silent_ms >= kSilenceTimeoutMs) {
                fatalExit("silence watchdog fired — no data for >1500 ms (Yahboom offline?)");
                return;
            }
            continue;
        }
        // Valid HEAD seen -> reset the silence-watchdog timer.
        last_valid_head = std::chrono::steady_clock::now();

        // Second sync byte must equal DEVICE_ID-1 (0xFB) to mark an inbound report frame.
        uint8_t head2 = 0;
        if (ser_.readByte(head2) != 1) continue;
        if (head2 != static_cast<uint8_t>(DEVICE_ID - 1)) continue;

        // Next two bytes: frame length and report type (ext_type selects the parseData branch).
        uint8_t ext_len = 0, ext_type = 0;
        if (ser_.readByte(ext_len)  != 1) continue;
        if (ser_.readByte(ext_type) != 1) continue;

        // Incoming checksum accumulator, seeded per firmware: (ext_len + ext_type + sum(payload)) & 0xFF.
        int check_sum = ext_len + ext_type;
        // Bytes still to read = payload bytes + 1 trailing checksum byte.
        const int data_len = ext_len - 2;
        if (data_len < 0) continue;

        std::vector<uint8_t> ext_data;
        ext_data.reserve(static_cast<size_t>(data_len));
        uint8_t rx_check_num = 0;
        bool read_error = false;

        while (static_cast<int>(ext_data.size()) < data_len) {
            uint8_t val = 0;
            const int rn = ser_.readByte(val);
            if (rn < 0) { read_error = true; break; }
            if (rn == 0) continue;
            ext_data.push_back(val);
            // The final byte of the frame IS the checksum: capture it, do not fold it into the running sum.
            if (static_cast<int>(ext_data.size()) == data_len) {
                rx_check_num = val;   // last byte is checksum — not accumulated
            } else {
                check_sum += val;
            }
        }

        if (read_error) {
            fatalExit("serial error mid-frame (EIO/ENOTTY?)");
            return;
        }

        if (static_cast<int>(ext_data.size()) < data_len) continue;

        // Checksum matches -> decode the frame; otherwise drop it (reported only in debug mode).
        if ((check_sum % 256) == rx_check_num) {
            parseData(ext_type, ext_data);
        } else if (debug_) {
            std::cout << "checksum error: len=" << static_cast<int>(ext_len)
                      << " type=0x" << std::hex << static_cast<int>(ext_type)
                      << " got=" << static_cast<int>(rx_check_num)
                      << " expected=" << (check_sum % 256)
                      << std::dec << "\n";
        }
    }
}

// ── create_receive_threading ──────────────────────────────────────────────────
/**
* @brief Spawns recv_thread_ running receiveLoop(). Idempotent: a no-op if the
*        receive loop is already running.
*/
inline void Rosmaster::create_receive_threading() {
    if (uart_running_.load()) return;
    uart_running_ = true;
    recv_thread_ = std::thread(&Rosmaster::receiveLoop, this);
    std::cout << "----------------create receive threading--------------\n";
    delay_ms(50);
}

// ── limitMotorValue ───────────────────────────────────────────────────────────
/**
* @brief Clamps a percent command [-100,100] into a signed byte. 127 is a passthrough
*        sentinel meaning "leave this motor unchanged" and bypasses the clamp.
*/
inline int8_t Rosmaster::limitMotorValue(double v) const {
    // 127 sentinel: caller wants this motor left as-is, so skip clamping.
    if (static_cast<int>(v) == 127) return 127;
    if (v >  100.0) return  100;
    if (v < -100.0) return -100;
    return static_cast<int8_t>(v);
}

// ── armConvertValue / armConvertAngle ─────────────────────────────────────────
/**
* @brief Maps a servo joint angle (degrees) to a raw pulse count via a per-servo
*        linear range. Servos 1-4: 0-180deg (inverted); servo 5: 0-270deg; servo 6:
*        0-180deg. Returns -1 for an unknown id.
*/
inline int Rosmaster::armConvertValue(int s_id, int s_angle) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
            // Servos 1-4: 0-180deg -> pulse 3100..900 (negative slope => mechanically inverted).
            return static_cast<int>((3100.0-900.0)*(s_angle-180.0)/(0.0-180.0)+900.0);
        case 5:
            // Servo 5 (wide range): 0-270deg -> pulse 380..3700.
            return static_cast<int>((3700.0-380.0)*(s_angle-0.0)/(270.0-0.0)+380.0);
        case 6:
            return static_cast<int>((3100.0-900.0)*(s_angle-0.0)/(180.0-0.0)+900.0);
        default: return -1;
    }
}
/**
* @brief Inverse of armConvertValue: raw pulse -> joint angle (degrees). The +0.5
*        term rounds to the nearest degree before truncation to int.
*/
inline int Rosmaster::armConvertAngle(int s_id, int s_value) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
            // Servos 1-4 inverse map (pulse 3100..900 -> 0-180deg); +0.5 rounds before int truncation.
            return static_cast<int>((s_value-900.0)*(0.0-180.0)/(3100.0-900.0)+180.0+0.5);
        case 5:
            return static_cast<int>((270.0-0.0)*(s_value-380.0)/(3700.0-380.0)+0.0+0.5);
        case 6:
            return static_cast<int>((180.0-0.0)*(s_value-900.0)/(3100.0-900.0)+0.0+0.5);
        default: return -1;
    }
}

// =============================================================================
//  Software PID implementation
// =============================================================================

// ── writeMotorRaw ─────────────────────────────────────────────────────────────
/**
 * @brief Encode four signed motor commands (percent, [-100,100]) into a
 *        FUNC_MOTOR frame and hand it to writeCmd() (which appends the
 *        checksum). This is the single low-level actuator write, called every
 *        tick from pidLoop() on pid_thread_.
 */
inline void Rosmaster::writeMotorRaw(const std::array<double, 4> & cmd) {
    // Turn a signed percent into one on-wire byte: round -> clamp to [-100,100]
    // -> int8_t (two's-complement) -> reinterpret as uint8_t. The controller reads
    // each motor byte back as a signed int8, so e.g. -100 travels as 0x9C.
    auto clamp_byte = [](double v) -> uint8_t {
        return static_cast<uint8_t>(
            static_cast<int8_t>(
                std::clamp(static_cast<int>(std::round(v)), -100, 100)));
    };
    // Frame byte layout: HEAD(0xFF) DEVICE_ID(0xFC) len(placeholder) FUNC_MOTOR,
    // then one signed motor byte each for FL, FR, RL, RR. The checksum byte is
    // computed and appended by writeCmd(); len is patched in just below.
    std::vector<uint8_t> raw = {
        HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
        clamp_byte(cmd[0]), clamp_byte(cmd[1]),
        clamp_byte(cmd[2]), clamp_byte(cmd[3])
    };
    // len field = frame size minus 1, because the trailing checksum byte (added
    // later by writeCmd) is excluded from the length count.
    raw[2] = static_cast<uint8_t>(raw.size() - 1);
    writeCmd(raw);
}

// ── pidLoop ───────────────────────────────────────────────────────────────────
//
// Per-motor control law (v8 — feedforward + sign-correct PID):
//
//  Velocity measurement (timestamp-based window, signed):
//   delta[i]  = int32(enc_now[i] − oldest_slot.enc[i])   (signed modulo-2³²)
//   window_dt = (ts_now − oldest_slot.ts_us) / 1e6        [real seconds]
//   velocity  = delta[i] / window_dt                      [ticks/s, signed]
//   measured  = clamp(velocity / scale × 100, −200, 200)  [%, signed]
//
//  EMA on measured (for display / feedforward quality):
//   meas_filt = α·meas_filt_prev + (1−α)·measured
//
//  EMA on error (for D term — preserves sign across direction reversal):
//   error      = target − measured                         [signed]
//   err_filt   = α·err_filt_prev + (1−α)·error
//   d_term     = kd · (err_filt − err_filt_prev) / dt
//     → positive when error is growing (motor lagging)  → pushes harder
//     → negative when error is shrinking (motor catching up) → backs off
//     → sign is consistent regardless of rotation direction
//
//  Feedforward (v8):
//   ff_term = kS[i]·sign(target) + kV[i]·target
//     → kS compensates static friction / dead zone (PWM% needed just to
//       start moving — the motor will not turn below this regardless of
//       the linear term)
//     → kV is the linear PWM-per-%-speed slope above the dead zone,
//       measured under whatever load was present during calibration
//     → when feedforward is disabled or uncalibrated, ff_gains_[i] defaults
//       to {kS=0, kV=1}, which reduces ff_term to "target" exactly as in v7
//
//  PID output:
//   raw_cmd = ff_term + kp·error + ki·integral + d_term
//     → the PID now corrects the *residual* error around the feedforward
//       prediction, instead of carrying the whole command as in v7. Gains
//       tuned for v7 will likely need to be reduced after calibration.
//
//  Anti-windup (improved, v7):
//   Integrate if |raw_cmd| < 100  (not saturated)
//   OR if sign(error) ≠ sign(integral)  (integral winding DOWN — always allow)
//   This lets the integrator discharge during direction reversal even when
//   saturated, preventing the stuck-integral bug at inversion.
//
/**
 * @brief Body of pid_thread_ (see the detailed law block above): paces at
 *        kPidHz, measures each motor's signed velocity over a real-timestamp
 *        window, runs the v8 feedforward + PID law, and actuates via
 *        writeMotorRaw(). Loops until disable_pid_control() clears pid_running_.
 */
inline void Rosmaster::pidLoop() {
    using Clock = std::chrono::steady_clock;

    // ── Seed sliding window ───────────────────────────────────────────────
    {
        int m1, m2, m3, m4;
        get_motor_encoder(m1, m2, m3, m4);
        const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now().time_since_epoch()).count();
        EncSlot seed;
        seed.enc  = { static_cast<uint32_t>(m1), static_cast<uint32_t>(m2),
                      static_cast<uint32_t>(m3), static_cast<uint32_t>(m4) };
        seed.ts_us = now_us;
        // Prime every ring slot with the current encoder reading + timestamp so the
        // first ticks compute a valid near-zero velocity instead of a huge spurious
        // delta against uninitialised history.
        for (auto & h : enc_history_) h = seed;
        enc_prev_pid_     = seed.enc;
        enc_history_idx_  = 0;
        enc_history_full_ = false;
    }

    // Pacing anchor: each tick sleeps until t_prev+kPidPeriod, then advances t_prev
    // to the ACTUAL wake time, so the dt fed to the control law is the true elapsed
    // interval rather than an assumed exact 40 ms.
    auto t_prev = Clock::now();

    while (pid_running_.load(std::memory_order_relaxed)) {

        // Block until this tick's absolute deadline (~40 ms at kPidHz=25). sleep_until
        // targets a fixed time point, so brief overruns don't push the next deadline out.
        std::this_thread::sleep_until(t_prev + kPidPeriod);

        const auto   t_now = Clock::now();
        const double dt    = std::max(
            std::chrono::duration<double>(t_now - t_prev).count(), 1e-6);
        t_prev = t_now;

        // Skip iteration if scheduler overslept more than 3× nominal period
        if (dt > 3.0 / kPidHz) continue;

        const int64_t ts_now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            t_now.time_since_epoch()).count();

        // ── Read encoders ─────────────────────────────────────────────────
        int m1, m2, m3, m4;
        get_motor_encoder(m1, m2, m3, m4);
        const std::array<uint32_t, 4> enc_now = {
            static_cast<uint32_t>(m1), static_cast<uint32_t>(m2),
            static_cast<uint32_t>(m3), static_cast<uint32_t>(m4)
        };

        // ── Update ring buffer ────────────────────────────────────────────
        // Oldest sample = the slot we're about to overwrite (idx+1, wrapped) once the
        // ring has filled; while still filling, slot 0 still holds the oldest seed.
        const int oldest_idx = enc_history_full_
            ? (enc_history_idx_ + 1) % kVelWindow
            : 0;

        enc_history_[enc_history_idx_] = { enc_now, ts_now_us };
        enc_history_idx_ = (enc_history_idx_ + 1) % kVelWindow;
        if (enc_history_idx_ == 0) enc_history_full_ = true;

        // ── Real window_dt from timestamps ────────────────────────────────
        const int64_t ts_oldest_us = enc_history_[oldest_idx].ts_us;
        const double  window_dt    = std::max(
            static_cast<double>(ts_now_us - ts_oldest_us) * 1e-6, 1e-6);

        // ── Gains snapshot ────────────────────────────────────────────────
        std::array<PidGains, 4> gains;
        {
            std::lock_guard<std::mutex> lk(pid_gains_mutex_);
            for (int i = 0; i < 4; ++i)
                gains[i] = motor_gains_override_[i] ? motor_gains_[i] : pid_gains_;
        }

        // ── Feedforward gains snapshot ─────────────────────────────────────
        std::array<FeedforwardGains, 4> ff_gains_snapshot;
        const bool ff_on = feedforward_enabled_.load(std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lk(ff_mutex_);
            ff_gains_snapshot = ff_gains_;
        }

        // Global fallback scale [ticks/s at 100% command], floored at 1 to avoid a
        // divide-by-zero in the velocity -> percent conversion. A calibrated per-motor
        // motor_scale_[i] overrides it inside the loop below.
        const double scale_global = std::max(ticks_per_second_at_100pct_, 1.0);

        std::array<double, 4> cmd_out{};

        for (int i = 0; i < 4; ++i) {
            // ── Signed velocity over real timestamp window ─────────────────
            // int32 cast of uint32 subtraction → signed delta, correct across
            // counter wraparound in both rotation directions.
            const int32_t delta = static_cast<int32_t>(
                enc_now[i] - enc_history_[oldest_idx].enc[i]);
            const double velocity = static_cast<double>(delta) / window_dt;

            const double scale_i = (motor_scale_[i] > 0.0)
                ? motor_scale_[i] : scale_global;

            // measured is signed: positive = forward, negative = reverse.
            const double measured = std::clamp(
                (velocity / scale_i) * 100.0, -200.0, 200.0);

            // ── EMA on measured (for display quality) ─────────────────────
            motor_state_[i].measured_filtered =
                kDerivAlpha           * motor_state_[i].measured_filtered
                + (1.0 - kDerivAlpha) * measured;

            pid_measured_[i].store(measured, std::memory_order_relaxed);

            const double target = target_[i].load(std::memory_order_relaxed);
            const double error  = target - measured;  // signed

            // ── EMA on error → D term (sign-correct in both directions) ───
            // Filtering error (not measured) means the D term pushes harder
            // when the error is growing and backs off when it is shrinking,
            // with the correct sign regardless of rotation direction.
            // Using error-derivative (not measured-derivative) also means
            // a step change in target produces a positive D impulse that
            // helps the motor accelerate toward the new setpoint.
            const double err_filt_prev = motor_state_[i].error_filtered;
            motor_state_[i].error_filtered =
                kDerivAlpha           * err_filt_prev
                + (1.0 - kDerivAlpha) * error;

            const double d_term = gains[i].kd
                * (motor_state_[i].error_filtered - err_filt_prev)
                / dt;

            // ── Feedforward term (v8) ───────────────────────────────────────
            // pwm_ff = kS·sign(target) + kV·target.
            // sign(target)==0 when target==0 → ff_term==0 exactly, so the
            // motor is not commanded to fight static friction while
            // explicitly stopped (no creeping at target=0).
            double ff_term = target;   // v7-equivalent fallback (kS=0, kV=1)
            if (ff_on) {
                const FeedforwardGains & g = ff_gains_snapshot[i];
                if (target == 0.0) {
                    ff_term = 0.0;
                } else {
                    const double sign = (target > 0.0) ? 1.0 : -1.0;
                    ff_term = g.kS * sign + g.kV * target;
                }
            }

            // ── PID output: feedforward + residual-error correction ───────
            const double raw_cmd =
                ff_term
                + gains[i].kp * error
                + gains[i].ki * motor_state_[i].integral
                + d_term;

            // ── Anti-windup (sign-aware) ───────────────────────────────────
            // Allow integration when:
            //   (a) output is not saturated  |raw_cmd| < 100, OR
            //   (b) error and integral have opposite signs → integral is
            //       winding DOWN toward zero, which is always safe.
            // Case (b) is critical at direction reversal: the integrator
            // charged in the forward direction must be able to discharge
            // even while the output is saturated in the reverse direction.
            const bool not_saturated  = std::abs(raw_cmd) < 100.0;
            // error * integral < 0  <=>  opposite signs  <=>  the integrator is discharging
            // toward zero, which the anti-windup rule always permits (even when saturated).
            const bool winding_down   = (error * motor_state_[i].integral) < 0.0;
            if (not_saturated || winding_down) {
                motor_state_[i].integral += error * dt;
                motor_state_[i].integral  = std::clamp(
                    motor_state_[i].integral, -80.0, 80.0);
            }

            cmd_out[i] = std::clamp(raw_cmd, -100.0, 100.0);
        }

        enc_prev_pid_ = enc_now;
        writeMotorRaw(cmd_out);
    }
}

// ── enable_pid_control ────────────────────────────────────────────────────────
/**
 * @brief Check preconditions (recv thread up, at least one encoder frame seen),
 *        warn if scale/feedforward are uncalibrated, latch gains + scale, zero
 *        the targets, then launch pid_thread_. Idempotent. Must run AFTER
 *        create_receive_threading() + set_auto_report_state(true).
 */
inline void Rosmaster::enable_pid_control(double kp, double ki, double kd,
                                           double ticks_per_sec) {
    // Precondition: pidLoop() reads sensor atomics populated by receiveLoop(), so
    // the receive thread must already be running (create_receive_threading()).
    if (!uart_running_.load(std::memory_order_acquire))
        throw std::runtime_error(
            "Rosmaster::enable_pid_control(): receive thread not running — "
            "call create_receive_threading() first");
    if (!encoder_received_.load(std::memory_order_acquire))
        throw std::runtime_error(
            "Rosmaster::enable_pid_control(): no encoder packet received yet — "
            "call set_auto_report_state(true) and wait ~100 ms before enabling PID");

    // Warn (non-fatal): without calibrate_pid_scale() the velocity->percent scale
    // is only a rough default, so closed-loop tracking will be inaccurate.
    if (!pid_scale_calibrated_)
        std::cerr << "Rosmaster: WARNING — using default PID scale ("
                  << ticks_per_sec
                  << " ticks/s). Run calibrate_pid_scale() for accurate control.\n";

    // Note (non-fatal): without calibrate_feedforward() there is no dead-zone
    // compensation; ff falls back to kS=0,kV=1 (v7 pass-through: ff_term == target).
    if (!feedforward_calibrated_)
        std::cerr << "Rosmaster: NOTE — feedforward not calibrated yet. "
                     "Run calibrate_feedforward() and enable_feedforward(true) "
                     "for dead-zone-compensated control.\n";

    if (pid_running_.load()) return;   // idempotent

    // Latch the global scale used by pidLoop's velocity->percent conversion.
    ticks_per_second_at_100pct_ = ticks_per_sec;

    {
        std::lock_guard<std::mutex> lk(pid_gains_mutex_);
        pid_gains_ = {kp, ki, kd};
    }

    // Start from a standstill: zero every per-motor target before the loop spins up.
    for (auto & t : target_) t.store(0.0, std::memory_order_relaxed);

    enc_history_full_ = false;
    enc_history_idx_  = 0;
    // EncSlot timestamps will be seeded at pidLoop() startup.

    pid_enabled_.store(true,  std::memory_order_release);
    pid_running_.store(true,  std::memory_order_release);
    // Publish the running/enabled flags (release order) BEFORE spawning so the new
    // thread's first load observes them, then start pidLoop() on pid_thread_.
    pid_thread_ = std::thread(&Rosmaster::pidLoop, this);

    std::cout << "Rosmaster: PID enabled @ " << kPidHz << " Hz"
              << "  window=" << kVelWindow << " cycles"
              << "  scale=" << ticks_per_second_at_100pct_ << " ticks/s"
              << "  feedforward=" << (feedforward_enabled_.load() ? "ON" : "OFF")
              << "\n";
}

// ── disable_pid_control ───────────────────────────────────────────────────────
/**
 * @brief Clear pid_running_ (so pidLoop() exits its while), join pid_thread_,
 *        then reset controller state. Idempotent. The destructor calls this
 *        BEFORE stopping the receive thread, because pidLoop() consumes the
 *        atomics that receiveLoop() produces.
 */
inline void Rosmaster::disable_pid_control() {
    if (!pid_running_.load()) return;   // idempotent

    pid_running_.store(false, std::memory_order_release);
    if (pid_thread_.joinable()) pid_thread_.join();
    pid_enabled_.store(false, std::memory_order_release);
    // Safe to reset now: the PID thread is joined, so there is no concurrent writer
    // of motor_state_.
    reset_pids();

    std::cout << "Rosmaster: PID disabled\n";
}

// ── reset_pids ────────────────────────────────────────────────────────────────
/**
 * @brief Clear per-motor integrators/EMA filters, targets, and reported
 *        measurements. Refuses to run while pid_thread_ is active, since
 *        motor_state_ is owned exclusively by that thread — disable first.
 */
inline void Rosmaster::reset_pids() {
    if (pid_running_.load(std::memory_order_acquire)) {
        std::cerr << "Rosmaster::reset_pids() ignored — PID thread is active. "
                     "Call disable_pid_control() first.\n";
        return;
    }
    // Zero each motor's integral and error/measured EMA state (PID-thread-owned).
    for (auto & s : motor_state_) s.reset();
    for (auto & t : target_)      t.store(0.0, std::memory_order_relaxed);
    for (auto & m : pid_measured_) m.store(0.0, std::memory_order_relaxed);
}

// ── set_pid_gains ─────────────────────────────────────────────────────────────
/**
 * @brief Atomically replace the shared default gains under pid_gains_mutex_ so
 *        pidLoop()'s per-tick snapshot never reads a half-written PidGains.
 *        Thread-safe; may be called live from the app thread.
 */
inline void Rosmaster::set_pid_gains(double kp, double ki, double kd) {
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    pid_gains_ = {kp, ki, kd};
}

// ── set_motor_pid_gains ───────────────────────────────────────────────────────
/**
 * @brief Store per-motor override gains under pid_gains_mutex_. With
 *        override=true, pidLoop() uses motor_gains_[i] for that wheel instead of
 *        the shared default. Out-of-range indices are silently ignored.
 */
inline void Rosmaster::set_motor_pid_gains(int motor_index,
                                            double kp, double ki, double kd,
                                            bool override) {
    if (motor_index < 0 || motor_index > 3) return;
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    motor_gains_[motor_index]          = {kp, ki, kd};
    motor_gains_override_[motor_index] = override;
}

// ── reset_motor_pid_gains ─────────────────────────────────────────────────────
/**
 * @brief Clear a motor's override flag so pidLoop() reverts it to the shared
 *        default gains; the stored per-motor values are left untouched.
 */
inline void Rosmaster::reset_motor_pid_gains(int motor_index) {
    if (motor_index < 0 || motor_index > 3) return;
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    motor_gains_override_[motor_index] = false;
}

// ── configure_slope_compensation ─────────────────────────────────────────────
/**
 * @brief Set the gravity-compensation parameters read by apply_slope(): enable
 *        flag, k_gravity boost, a pitch deadband (rad) below which the grade is
 *        ignored, and a staleness timeout (ns) after which a missing pitch
 *        update disables compensation. Plain scalars — configure before driving.
 */
inline void Rosmaster::configure_slope_compensation(bool enabled,
                                                     double k_gravity,
                                                     double deadband_rad,
                                                     uint64_t timeout_ns) {
    slope_comp_enabled_ = enabled;
    k_gravity_          = k_gravity;
    deadband_rad_        = deadband_rad;
    pitch_timeout_ns_    = timeout_ns;
}

// ── set_max_pwm_flat ──────────────────────────────────────────────────────────
/**
 * @brief Cap the flat-ground PWM (percent) used by reserve_headroom(), clamped
 *        to [10,100]. The reserved margin below 100% is what apply_slope() uses
 *        to add uphill boost without saturating.
 */
inline void Rosmaster::set_max_pwm_flat(double pwm) {
    max_pwm_flat_ = std::clamp(pwm, 10.0, 100.0);
}

// ── update_pitch ──────────────────────────────────────────────────────────────
/**
 * @brief Feed a fresh chassis pitch (radians) plus a monotonic timestamp so
 *        apply_slope() can both compensate for the grade and detect stale data.
 *        Lock-free via atomics; call from the IMU/odometry thread.
 */
inline void Rosmaster::update_pitch(double pitch_rad) {
    pitch_rad_.store(pitch_rad, std::memory_order_relaxed);
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    // Timestamp the update so apply_slope() can treat pitch older than
    // pitch_timeout_ns_ as stale and skip compensation (fail-safe on IMU dropout).
    last_pitch_time_ns_.store(now_ns, std::memory_order_relaxed);
}

// ── set_motor_with_compensation ───────────────────────────────────────────────
/**
 * @brief Open-loop convenience wrapper: reserve_headroom() caps each command to
 *        max_pwm_flat_, apply_slope() then adds pitch-based uphill boost into
 *        that reserved margin, and set_motor() sends it. Bypasses the velocity
 *        PID entirely.
 */
inline void Rosmaster::set_motor_with_compensation(double s1, double s2,
                                                    double s3, double s4) {
    set_motor(
        apply_slope(reserve_headroom(s1)),
        apply_slope(reserve_headroom(s2)),
        apply_slope(reserve_headroom(s3)),
        apply_slope(reserve_headroom(s4)));
}

// ── calibrate_pid_scale ───────────────────────────────────────────────────────
/** @brief Simplest calibration: one 100% burst, then average |Δticks|/dt over the
  *  4 motors to get ticks/s at 100% command. Sets a single global scale (no
  *  per-motor split). Requires PID disabled and wheels free to spin. */
inline double Rosmaster::calibrate_pid_scale(int duration_ms) {
    if (pid_enabled_.load())
        throw std::logic_error(
            "Rosmaster::calibrate_pid_scale(): disable PID control first");
    if (duration_ms < 100)
        throw std::invalid_argument(
            "Rosmaster::calibrate_pid_scale(): duration_ms < 100 — "
            "too few ticks for a reliable estimate");

    std::cout << "Rosmaster: calibration (" << duration_ms
              << " ms) — ensure all wheels are free to spin\n";

    // Snapshot the cumulative encoder counters just before the burst.
    int m1a, m2a, m3a, m4a;
    get_motor_encoder(m1a, m2a, m3a, m4a);

    // Drive all four wheels at full 100% command (bypasses the PID) for duration_ms.
    writeMotorRaw({100.0, 100.0, 100.0, 100.0});
    delay_ms(duration_ms);
    writeMotorRaw({0.0, 0.0, 0.0, 0.0});

    int m1b, m2b, m3b, m4b;
    get_motor_encoder(m1b, m2b, m3b, m4b);

    const double dt    = duration_ms / 1000.0;
    const int    ra[4] = {m1a, m2a, m3a, m4a};
    const int    rb[4] = {m1b, m2b, m3b, m4b};
    double total = 0.0;
    for (int i = 0; i < 4; ++i) {
        // Wrap-safe delta: subtract as uint32 then reinterpret as int32, so a counter
        // crossing 2^32 still yields the correct signed tick count.
        const int32_t delta = static_cast<int32_t>(
            static_cast<uint32_t>(rb[i]) - static_cast<uint32_t>(ra[i]));
        total += std::abs(static_cast<double>(delta));
    }

    // Mean ticks across the 4 motors ÷ elapsed seconds = ticks/s at 100% command.
    const double result = (total / 4.0) / dt;
    if (result <= 0.0)
        throw std::runtime_error(
            "Rosmaster::calibrate_pid_scale(): no encoder ticks measured — "
            "check UART connection, motor wiring, and auto-report state");

    ticks_per_second_at_100pct_ = result;
    pid_scale_calibrated_ = true;

    std::cout << "Rosmaster: scale = " << ticks_per_second_at_100pct_
              << " ticks/s (average over 4 motors)\n";
    return ticks_per_second_at_100pct_;
}

// ── calibrate_motor_scales ────────────────────────────────────────────────────
//
// Multi-run, ratio-based calibration (v6 — promoted from Rosmaster.cpp main()).
//
// Algorithm:
//   1. Optional thermal warmup at 60% for warmup_ms (skip with warmup_ms=0).
//   2. N runs at throttle_pct%.
//   3. Per-run: stable snapshot before → motor on → stable snapshot after.
//   4. Compute inter-motor ratios (M[i] / run_mean).  Ratios are stable
//      run-to-run even if absolute ticks drift ±5% thermally (CV < 1%).
//   5. Global scale = trimmed mean of per-run global samples (drop min+max).
//   6. Per-motor scale = global × mean_ratio[i].
//
// Preconditions: PID disabled, wheels free to spin, auto-report active.
//
inline double Rosmaster::calibrate_motor_scales(int throttle_pct,
                                                 int duration_ms,
                                                 int n_runs,
                                                 int warmup_ms,
                                                 bool use_per_motor) {
    if (pid_enabled_.load())
        throw std::logic_error(
            "Rosmaster::calibrate_motor_scales(): disable PID control first");
    if (throttle_pct < 20 || throttle_pct > 80)
        throw std::invalid_argument(
            "Rosmaster::calibrate_motor_scales(): throttle_pct must be in [20, 80]");
    if (duration_ms < 200)
        throw std::invalid_argument(
            "Rosmaster::calibrate_motor_scales(): duration_ms < 200");
    if (n_runs < 1)
        throw std::invalid_argument(
            "Rosmaster::calibrate_motor_scales(): n_runs < 1");

    // ── Stable double-read helper ─────────────────────────────────────────
    // Two identical consecutive reads 25 ms apart → encoder packet settled.
    auto stable_read = [&](int m[4]) {
        int a[4], b[4];
        // Retry up to 30 × 25 ms ≈ 0.75 s, waiting for two consecutive reads to agree.
        for (int i = 0; i < 30; ++i) {
            get_motor_encoder(a[0], a[1], a[2], a[3]);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            get_motor_encoder(b[0], b[1], b[2], b[3]);
            // Two identical reads 25 ms apart ⇒ the async encoder packet has settled; accept b[].
            if (a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]) {
                for (int j = 0; j < 4; ++j) m[j] = b[j];
                return;
            }
        }
        // Timeout — take most recent value
        for (int j = 0; j < 4; ++j) m[j] = b[j];
    };

    // ── Optional thermal warmup ───────────────────────────────────────────
    if (warmup_ms > 0) {
        std::cout << "Rosmaster: warmup " << warmup_ms << " ms @ 60%\n";
        writeMotorRaw({60.0, 60.0, 60.0, 60.0});
        std::this_thread::sleep_for(std::chrono::milliseconds(warmup_ms));
        writeMotorRaw({0.0, 0.0, 0.0, 0.0});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    const double t   = static_cast<double>(throttle_pct);
    const double dt  = duration_ms / 1000.0;

    std::array<double, 4> ratio_sum  = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> ratio_sum2 = {0.0, 0.0, 0.0, 0.0};
    std::vector<double>   global_samples;
    int valid_runs = 0;

    std::cout << "\nRosmaster: calibration @ " << throttle_pct
              << "% — " << n_runs << " runs × " << duration_ms << " ms\n";

    // Repeat the burst n_runs times so per-run noise and outliers can be trimmed/averaged.
    for (int run = 0; run < n_runs; ++run) {
        if (run > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int ma[4], mb[4];
        stable_read(ma);

        writeMotorRaw({t, t, t, t});
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        writeMotorRaw({0.0, 0.0, 0.0, 0.0});

        // Wait for mechanical stop before final snapshot
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        stable_read(mb);

        // Ticks per motor (uint32 modulo-2³² arithmetic)
        // Per-motor absolute tick counts for this run (wrap-safe, uint32 modulo-2^32).
        std::array<double, 4> ticks{};
        bool ok = true;
        for (int i = 0; i < 4; ++i) {
            const int32_t d = static_cast<int32_t>(
                static_cast<uint32_t>(mb[i]) - static_cast<uint32_t>(ma[i]));
            ticks[i] = std::abs(static_cast<double>(d));
            // Any motor that barely moved (<10 ticks) invalidates the whole run — likely a
            // stalled wheel or a dropped encoder packet.
            if (ticks[i] < 10.0) { ok = false; }
        }

        if (!ok) {
            std::cerr << "  [WARN] run " << run
                      << " : insufficient ticks — skipped\n";
            continue;
        }

        // Ratio vs run mean (robust: M1 anomaly doesn't skew all ratios)
        const double run_mean = (ticks[0]+ticks[1]+ticks[2]+ticks[3]) / 4.0;
        for (int i = 0; i < 4; ++i) {
            // This motor's share of the run average. Ratios stay stable run-to-run even under
            // ±5% thermal drift, so they are the robust part of the estimate (CV < 1%).
            const double r   = ticks[i] / run_mean;
            ratio_sum[i]    += r;
            ratio_sum2[i]   += r * r;
        }

        // Extrapolate: run-mean ticks/s measured at throttle_pct%, scaled up to a 100% reference.
        const double global = (run_mean / dt) * (100.0 / throttle_pct);
        global_samples.push_back(global);
        ++valid_runs;

        std::cout << "  Run " << run << " :";
        for (int i = 0; i < 4; ++i)
            std::cout << "  M" << (i+1) << "=" << static_cast<int>(ticks[i]);
        std::cout << "  global=" << static_cast<int>(global) << " ticks/s\n";
    }

    if (valid_runs < 1)
        throw std::runtime_error(
            "Rosmaster::calibrate_motor_scales(): no valid runs — "
            "check encoder wiring and set_auto_report_state");

    // ── Global scale: trimmed mean (drop min+max if n_runs >= 3) ─────────
    double result;
    // Trimmed mean: with ≥3 runs, sort and drop the single min and max samples before
    // averaging — rejects one bad run in each direction.
    if (valid_runs >= 3) {
        std::sort(global_samples.begin(), global_samples.end());
        double global_total = 0.0;
        // Sum only the interior samples [1 .. n-2] after sorting (min at 0, max at n-1 dropped).
        for (size_t i = 1; i < global_samples.size() - 1; ++i)
            global_total += global_samples[i];
        result = global_total / static_cast<double>(global_samples.size() - 2);
    } else {
        double sum = 0.0;
        for (double v : global_samples) sum += v;
        result = sum / static_cast<double>(valid_runs);
    }

    // ── Inter-motor ratios + CV ───────────────────────────────────────────
    std::cout << "\n=== Inter-motor ratios ===\n";
    std::array<double, 4> ratio_mean{};
    for (int i = 0; i < 4; ++i) {
        ratio_mean[i]     = ratio_sum[i] / valid_runs;
        // One-pass variance via E[r²] − E[r]², using the running sum and sum-of-squares.
        const double var  = ratio_sum2[i] / valid_runs
                          - ratio_mean[i] * ratio_mean[i];
        // Coefficient of variation = stddev/mean as a %. A scale-free spread metric;
        // >1.5% below flags an erratic/irregular motor.
        const double cv   = std::sqrt(std::max(var, 0.0))
                          / ratio_mean[i] * 100.0;
        std::cout << "  M" << (i+1)
                  << "  ratio=" << std::fixed << std::setprecision(4) << ratio_mean[i]
                  << "  CV="    << std::setprecision(2) << cv << "%";
        if (cv > 1.5)
            std::cout << "  [WARN: high CV — irregular motor?]";
        std::cout << "\n";
    }
    std::cout << std::defaultfloat;

    // ── Final per-motor scales ────────────────────────────────────────────
    std::cout << "\n=== Final scales ===\n";
    for (int i = 0; i < 4; ++i) {
        // Per-motor scale = global ticks/s@100% × this motor's mean inter-motor ratio.
        const double s = result * ratio_mean[i];
        std::cout << "  M" << (i+1)
                  << " = " << s << " ticks/s"
                  << "  (ratio=" << ratio_mean[i] << ")\n";
        // Only overwrite the live per-motor scales when the caller opted in; else just report.
        if (use_per_motor) motor_scale_[i] = s;
    }
    std::cout << "  Global = " << result << " ticks/s\n";

    ticks_per_second_at_100pct_ = result;
    pid_scale_calibrated_ = true;

    if (use_per_motor)
        std::cout << "Rosmaster: per-motor scales enabled\n";

    return result;
}

// ── calibrate_pid_scale_at ────────────────────────────────────────────────────
// Single-run compatibility wrapper — delegates to calibrate_motor_scales().
// For production use, prefer calibrate_motor_scales(n_runs >= 3).
inline double Rosmaster::calibrate_pid_scale_at(int throttle_pct,
                                                  int duration_ms,
                                                  bool use_per_motor) {
    return calibrate_motor_scales(throttle_pct, duration_ms,
                                   /*n_runs=*/1,
                                   /*warmup_ms=*/0,
                                   use_per_motor);
}

// ── set_motor_scales ──────────────────────────────────────────────────────────
/** @brief Install known-good scales manually (e.g. saved from a prior calibration) to
  *  skip re-calibrating on every boot. Marks pid_scale_calibrated_ so the PID may run. */
inline void Rosmaster::set_motor_scales(const std::array<double,4> & scales,
                                         double global) {
    for (int i = 0; i < 4; ++i) motor_scale_[i] = scales[i];
    ticks_per_second_at_100pct_ = global;
    pid_scale_calibrated_ = true;
    std::cout << "Rosmaster: motor_scale_ updated\n";
}

// ── set_feedforward_gains / get_feedforward_gains ─────────────────────────────
/** @brief Thread-safe write of one motor's (kS,kV) under ff_mutex_ — pidLoop reads
  *  ff_gains_ concurrently on every 40 ms tick. */
inline void Rosmaster::set_feedforward_gains(int motor_index, double kS, double kV) {
    if (motor_index < 0 || motor_index > 3) return;
    std::lock_guard<std::mutex> lk(ff_mutex_);
    ff_gains_[motor_index] = {kS, kV};
    feedforward_calibrated_ = true;
}

inline void Rosmaster::get_feedforward_gains(int motor_index,
                                              double & kS, double & kV) const {
    // Out-of-range index → return identity feedforward (kS=0, kV=1 ⇒ ff_term = target,
    // the pre-feedforward v7 default).
    if (motor_index < 0 || motor_index > 3) { kS = 0.0; kV = 1.0; return; }
    std::lock_guard<std::mutex> lk(ff_mutex_);
    kS = ff_gains_[motor_index].kS;
    kV = ff_gains_[motor_index].kV;
}

// ── calibrate_feedforward ──────────────────────────────────────────────────────
//
// Automatic PWM-sweep calibration of the per-motor feedforward model
//   pwm = kS·sign(v) + kV·v     (v = commanded %, pwm = PWM % applied)
//
// Method (per motor, independently):
//   1. Sweep PWM upward from throttle_min_pct to throttle_max_pct in
//      step_pct increments. At each step:
//        a. Command all 4 motors to the current PWM (every motor is swept
//           at the same time — simpler hardware sequencing; per-motor
//           results are still independent because each motor's own
//           encoder is read).
//        b. Wait settle_ms for mechanical transients to die down.
//        c. Measure speed over sample_ms via encoder delta.
//   2. kS[i] = the PWM% of the first step at which motor i's measured
//      speed exceeds a small "moving" threshold (a few ticks over
//      sample_ms — i.e. the dead zone has just been crossed).
//   3. kV[i] = least-squares slope of (PWM% vs measured speed%) using
//      only the points at or above kS[i] (the linear region) — fit as
//      PWM = kS + kV·speed, i.e. kV = ΔPWM / Δspeed in the linear region.
//   4. Results are written into ff_gains_[i] and feedforward_calibrated_
//      is set true. Caller must still call enable_feedforward(true).
//
// Preconditions: PID disabled, wheels free to spin, auto-report active.
// Returns the average dead-zone PWM (kS) across the 4 motors.
//
inline double Rosmaster::calibrate_feedforward(int throttle_min_pct,
                                                int throttle_max_pct,
                                                int step_pct,
                                                int settle_ms,
                                                int sample_ms) {
    if (pid_enabled_.load())
        throw std::logic_error(
            "Rosmaster::calibrate_feedforward(): disable PID control first");
    if (throttle_min_pct < 1 || throttle_min_pct >= throttle_max_pct)
        throw std::invalid_argument(
            "Rosmaster::calibrate_feedforward(): invalid throttle range");
    if (throttle_max_pct > 100)
        throw std::invalid_argument(
            "Rosmaster::calibrate_feedforward(): throttle_max_pct > 100");
    if (step_pct < 1)
        throw std::invalid_argument(
            "Rosmaster::calibrate_feedforward(): step_pct < 1");

    // Minimum tick count over sample_ms to declare "motor is moving".
    // Chosen conservatively above encoder/electrical noise floor.
    constexpr double kMovingTickThreshold = 8.0;

    std::cout << "\nRosmaster: feedforward calibration — PWM sweep "
              << throttle_min_pct << "% to " << throttle_max_pct
              << "% step " << step_pct << "%\n"
              << "Ensure all wheels are free to spin.\n";

    // Per-motor accumulated sweep samples: (pwm_pct, measured_speed_pct)
    struct SweepPoint { double pwm; double speed; };
    std::array<std::vector<SweepPoint>, 4> sweep;

    // Dead-zone PWM per motor — first step where the motor was observed
    // moving above kMovingTickThreshold. -1 until found.
    std::array<double, 4> dead_zone_pwm = {-1.0, -1.0, -1.0, -1.0};

    const double dt_sample = sample_ms / 1000.0;

    // Start from rest.
    writeMotorRaw({0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Sweep commanded PWM upward one step_pct at a time, from just above rest up to max.
    for (int pwm = throttle_min_pct; pwm <= throttle_max_pct; pwm += step_pct) {
        const double p = static_cast<double>(pwm);

        // Command all 4 motors at the same PWM together; results stay independent because
        // each motor's own encoder is read below.
        writeMotorRaw({p, p, p, p});
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));

        int ma[4];
        get_motor_encoder(ma[0], ma[1], ma[2], ma[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_ms));
        int mb[4];
        get_motor_encoder(mb[0], mb[1], mb[2], mb[3]);

        std::cout << "  PWM=" << pwm << "% :";
        for (int i = 0; i < 4; ++i) {
            const int32_t d = static_cast<int32_t>(
                static_cast<uint32_t>(mb[i]) - static_cast<uint32_t>(ma[i]));
            const double ticks = std::abs(static_cast<double>(d));
            const double scale_i = (motor_scale_[i] > 0.0)
                ? motor_scale_[i] : std::max(ticks_per_second_at_100pct_, 1.0);
            // Convert to percent: ticks over the sample window → ticks/s, ÷ scale (ticks/s@100%)
            // × 100. scale_i is the per-motor scale if calibrated, else the global fallback.
            const double speed_pct = (ticks / dt_sample) / scale_i * 100.0;

            std::cout << "  M" << (i+1) << "=" << static_cast<int>(ticks) << "t";

            // First PWM step where this motor crosses the moving threshold = its static-friction
            // dead zone; this PWM% becomes kS.
            if (dead_zone_pwm[i] < 0.0 && ticks >= kMovingTickThreshold) {
                dead_zone_pwm[i] = p;
            }
            // Only keep points once the motor is confirmed moving — points
            // below the dead zone would otherwise pull the linear fit
            // toward the origin and bias kV low.
            if (dead_zone_pwm[i] >= 0.0) {
                sweep[i].push_back({p, speed_pct});
            }
        }
        std::cout << "\n";
    }

    writeMotorRaw({0.0, 0.0, 0.0, 0.0});

    // ── Per-motor least-squares fit: PWM = kS + kV·speed ──────────────────
    // Fitting PWM as a function of speed (rather than the reverse) is what
    // we actually need at runtime: given a desired speed, what PWM do we
    // feed forward? Standard linear least squares on (x=speed, y=pwm).
    double dead_zone_sum = 0.0;
    int    dead_zone_n   = 0;

    std::cout << "\n=== Feedforward fit results ===\n";
    {
        std::lock_guard<std::mutex> lk(ff_mutex_);
        for (int i = 0; i < 4; ++i) {
            // Skip motors that never moved or yielded <3 linear-region points — keep their
            // default gains (kS=0, kV=1) untouched.
            if (dead_zone_pwm[i] < 0.0 || sweep[i].size() < 3) {
                std::cerr << "  [WARN] M" << (i+1)
                          << " : motor never exceeded moving threshold or "
                             "too few points — feedforward NOT updated for "
                             "this motor (kS=0, kV=1 kept)\n";
                continue;
            }

            const size_t n = sweep[i].size();
            // Least-squares accumulators over the linear region, with x = speed%, y = PWM%.
            double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
            for (const auto & pt : sweep[i]) {
                sx  += pt.speed;
                sy  += pt.pwm;
                sxx += pt.speed * pt.speed;
                sxy += pt.speed * pt.pwm;
            }
            // LS denominator n·Σx² − (Σx)²; near 0 when all speeds are identical (vertical data).
            const double denom = static_cast<double>(n) * sxx - sx * sx;

            double kV, kS;
            // Degenerate fit fallback: no solvable slope, so approximate kS/kV from the means.
            if (std::abs(denom) < 1e-9) {
                // Degenerate (near-vertical or single-speed data) — fall
                // back to dead-zone PWM as kS, kV from a single ratio.
                kS = dead_zone_pwm[i];
                kV = (sy / static_cast<double>(n)) / std::max(sx / n, 1.0);
            } else {
                // Slope kV = (n·Σxy − Σx·Σy) / denom  — ΔPWM per unit Δspeed%.
                kV = (static_cast<double>(n) * sxy - sx * sy) / denom;
                // Intercept kS = mean(PWM) − kV·mean(speed): the PWM the fit extrapolates to at v=0.
                kS = (sy - kV * sx) / static_cast<double>(n);
                // kS from the fit's y-intercept can be noisy/negative if
                // the linear region doesn't extrapolate cleanly to v=0.
                // The directly observed dead-zone crossing is a more
                // trustworthy floor — use it when the fit disagrees by a
                // large margin in the unsafe (too-low) direction.
                // If the fitted intercept is far below the observed dead zone (unsafe/too low),
                // trust the directly measured dead-zone PWM instead.
                if (kS < dead_zone_pwm[i] * 0.5) {
                    kS = dead_zone_pwm[i];
                }
            }

            kV = std::max(kV, 0.1);   // guard against non-positive slope
            // Clamp kS into a sane [0,50]% dead-zone band.
            kS = std::clamp(kS, 0.0, 50.0);

            // Commit this motor's fitted feedforward gains (still holding ff_mutex_).
            ff_gains_[i] = {kS, kV};
            dead_zone_sum += dead_zone_pwm[i];
            ++dead_zone_n;

            std::cout << "  M" << (i+1)
                      << "  kS=" << std::fixed << std::setprecision(2) << kS
                      << "%  kV=" << kV
                      << "  (dead zone observed at " << dead_zone_pwm[i]
                      << "% PWM, " << n << " linear-region points)\n";
        }
        std::cout << std::defaultfloat;
    }

    if (dead_zone_n == 0)
        throw std::runtime_error(
            "Rosmaster::calibrate_feedforward(): no motor exceeded the "
            "moving threshold during the sweep — check wiring, increase "
            "throttle_max_pct, or verify wheels are free to spin");

    feedforward_calibrated_ = true;

    const double avg_dead_zone = dead_zone_sum / dead_zone_n;
    std::cout << "Rosmaster: feedforward calibrated — average dead zone = "
              << avg_dead_zone << "% PWM. "
              << "Call enable_feedforward(true) to activate.\n";

    return avg_dead_zone;
}

// =============================================================================
//  Public setters
// =============================================================================

/**
 * @brief Toggle the firmware's periodic sensor auto-report stream (IMU / speed /
 *        encoder frames that receiveLoop() parses). Fixed 6-byte frame.
 *        Called during startup right after the threads spin up so the RX loop has
 *        data to decode. Run from the app thread only (no cross-thread guard).
 */
inline void Rosmaster::set_auto_report_state(bool enable, bool forever) {
    try {
        // Frame: HEAD(0xFF) DEVICE_ID(0xFC) len func params... checksum.
        // len is hard-coded 0x05 here: the byte count from this length field through the
        // checksum (writeCmd appends the checksum afterwards).
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05, FUNC_AUTO_REPORT,
                                    // byte4 = enable (1 = start streaming, 0 = stop); byte5 = 0x5F persists the
                                    // setting to firmware flash, 0 keeps it RAM-only until next power cycle.
                                    static_cast<uint8_t>(enable  ? 1 : 0),
                                    static_cast<uint8_t>(forever ? 0x5F : 0)};
        writeCmd(cmd);
        if (debug_) std::cout << "report: done\n";
    } catch (...) { std::cerr << "---set_auto_report_state error!---\n"; }
}

/**
 * @brief Sound the buzzer. on_time is a duration in milliseconds packed as a
 *        little-endian int16 (0 = off; special values held continuously by fw).
 */
inline void Rosmaster::set_beep(int on_time) {
    try {
        // Reject negative durations before touching the wire.
        if (on_time < 0) { std::cerr << "beep input error!\n"; return; }
        // Split the 16-bit duration into low/high bytes (little-endian on the wire).
        auto [lo, hi] = packI16(static_cast<int16_t>(on_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05, FUNC_BEEP, lo, hi};
        writeCmd(cmd);
        if (debug_) std::cout << "beep: done\n";
    } catch (...) { std::cerr << "---set_beep error!---\n"; }
}

/**
 * @brief Drive one PWM servo (id 1..4) to an angle in [0,180] degrees.
 *        First setter to compute the length byte dynamically: cmd[2] = size-1 =
 *        the byte count from the len field through the (yet-to-be-appended) checksum.
 */
inline void Rosmaster::set_pwm_servo(int servo_id, int angle) {
    try {
        // Silently ignore an out-of-range channel (only servos 1..4 exist).
        if (servo_id < 1 || servo_id > 4) return;
        // Saturate to the servo's mechanical 0..180 degree travel.
        angle = std::max(0, std::min(180, angle));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO,
                                    static_cast<uint8_t>(servo_id),
                                    static_cast<uint8_t>(angle)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_pwm_servo error!---\n"; }
}

/**
 * @brief Write all four PWM servos in one frame; a channel given an out-of-range
 *        angle is sent as the sentinel 255 so the firmware leaves it untouched.
 */
inline void Rosmaster::set_pwm_servo_all(int a1, int a2, int a3, int a4) {
    try {
        auto fix = [](int a) -> uint8_t {
            // 255 is the "no change" sentinel: any angle outside 0..180 maps to it.
            return (a < 0 || a > 180) ? 255 : static_cast<uint8_t>(a);
        };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO_ALL,
                                    fix(a1), fix(a2), fix(a3), fix(a4)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_pwm_servo_all error!---\n"; }
}

/**
 * @brief Set one addressable RGB LED (or 0xFF for the whole strip) to an R,G,B triple.
 */
inline void Rosmaster::set_colorful_lamps(int led_id, int red, int green, int blue) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_RGB,
                                    // byte4 = LED index (0-based) or 0xFF = all LEDs; then R,G,B each masked to 0..255.
                                    static_cast<uint8_t>(led_id & 0xff),
                                    static_cast<uint8_t>(red    & 0xff),
                                    static_cast<uint8_t>(green  & 0xff),
                                    static_cast<uint8_t>(blue   & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_colorful_lamps error!---\n"; }
}

/**
 * @brief Select a built-in RGB animation: effect id, animation speed, and an
 *        effect-specific parameter (e.g. hue), each masked to a single byte.
 */
inline void Rosmaster::set_colorful_effect(int effect, int speed, int parm) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_RGB_EFFECT,
                                    static_cast<uint8_t>(effect & 0xff),
                                    static_cast<uint8_t>(speed  & 0xff),
                                    static_cast<uint8_t>(parm   & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_colorful_effect error!---\n"; }
}

// ── set_motor ─────────────────────────────────────────────────────────────────
/**
 * @brief Dual-mode motor command. When the software velocity PID is running this
 *        only stores clamped targets (percent, [-100,100]) into the target_ atomics
 *        for pidLoop to service - NO frame is sent here. Otherwise it emits a raw
 *        open-loop FUNC_MOTOR PWM frame directly. Safe to call from the app thread.
 */
inline void Rosmaster::set_motor(double s1, double s2, double s3, double s4) {
    // PID path: hand the setpoints to pidLoop lock-free via atomics. The acquire load
    // pairs with the release store in enable/disable_pid_control().
    if (pid_enabled_.load(std::memory_order_acquire)) {
        // Targets are a percentage of full speed; clamp to +-100 % before publishing.
        auto clamp_target = [](double v) { return std::clamp(v, -100.0, 100.0); };
        target_[0].store(clamp_target(s1), std::memory_order_relaxed);
        target_[1].store(clamp_target(s2), std::memory_order_relaxed);
        target_[2].store(clamp_target(s3), std::memory_order_relaxed);
        target_[3].store(clamp_target(s4), std::memory_order_relaxed);
        return;
    }
    try {
        // Raw path: reinterpret the signed motor percent as a two's-complement byte.
        auto pack = [](int8_t v) -> uint8_t { return static_cast<uint8_t>(v); };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
                                    // limitMotorValue() saturates the double to an int8 percent in [-100,100].
                                    pack(limitMotorValue(s1)),
                                    pack(limitMotorValue(s2)),
                                    pack(limitMotorValue(s3)),
                                    pack(limitMotorValue(s4))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_motor error!---\n"; }
}

/**
 * @brief High-level directional drive: state = direction enum, speed = magnitude.
 *        Uses the cached car_type_; the CAR_ADJUST bit requests gyro-assisted
 *        straight-line correction from the firmware.
 */
inline void Rosmaster::set_car_run(int state, int speed, bool adjust) {
    try {
        uint8_t ct = car_type_;
        // OR in CAR_ADJUST (0x80) into the car-type byte to enable closed-loop yaw hold.
        if (adjust) ct = static_cast<uint8_t>(ct | CAR_ADJUST);
        auto [lo, hi] = packI16(static_cast<int16_t>(speed));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_CAR_RUN,
                                    ct, static_cast<uint8_t>(state & 0xff), lo, hi};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_car_run error!---\n"; }
}

/**
 * @brief Body-frame velocity command (mecanum): v_x, v_y in m/s, v_z yaw in rad/s.
 *        The firmware performs the wheel mixing from car_type_.
 */
inline void Rosmaster::set_car_motion(double v_x, double v_y, double v_z) {
    try {
        // Scale each velocity to integer milli-units (*1000) then pack LE int16
        // -> effective range about +-32.767 m/s (or rad/s).
        auto [vxl, vxh] = packI16(static_cast<int16_t>(v_x * 1000.0));
        auto [vyl, vyh] = packI16(static_cast<int16_t>(v_y * 1000.0));
        auto [vzl, vzh] = packI16(static_cast<int16_t>(v_z * 1000.0));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_MOTION,
                                    car_type_,
                                    vxl, vxh, vyl, vyh, vzl, vzh};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_car_motion error!---\n"; }
}

/**
 * @brief Program the FIRMWARE's onboard motor PID gains (distinct from this driver's
 *        software velocity PID). Gains constrained to [0,10]; forever(0x5F) persists
 *        to flash and waits 100 ms for the write.
 */
inline void Rosmaster::set_pid_param(double kp, double ki, double kd, bool forever) {
    try {
        // Firmware accepts each gain only in [0,10]; bail out otherwise.
        if (kp > 10 || ki > 10 || kd > 10 || kp < 0 || ki < 0 || kd < 0) {
            std::cerr << "PID value must be:[0, 10.00]\n"; return;
        }
        // Fixed-point encode as gain*1000 milli-units in LE int16 (e.g. 10.0 -> 10000).
        auto [kpl, kph] = packI16(static_cast<int16_t>(kp * 1000.0));
        auto [kil, kih] = packI16(static_cast<int16_t>(ki * 1000.0));
        auto [kdl, kdh] = packI16(static_cast<int16_t>(kd * 1000.0));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x0A, FUNC_SET_MOTOR_PID,
                                    kpl, kph, kil, kih, kdl, kdh,
                                    static_cast<uint8_t>(forever ? 0x5F : 0)};
        writeCmd(cmd);
        if (forever) delay_ms(100);
    } catch (...) { std::cerr << "---set_pid_param error!---\n"; }
}

/**
 * @brief Tell the firmware which chassis kinematics to use and cache car_type_
 *        locally (reused as a byte in every set_car_run/set_car_motion frame).
 *        0x5F persists the choice to flash.
 */
inline void Rosmaster::set_car_type(int car_type) {
    // Accept the full single-byte range 0..255.
    if (car_type >= 0 && car_type <= 255) {
        // Cache locally so later motion frames carry the matching car-type byte.
        car_type_ = static_cast<uint8_t>(car_type);
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_SET_CAR_TYPE,
                                    car_type_, 0x5F};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        delay_ms(100);
    } else {
        std::cerr << "set_car_type input invalid\n";
    }
}

/**
 * @brief Drive one serial-bus arm servo to a raw pulse over run_time ms.
 *        Gated by arm_ctrl_enable_ so a disabled arm silently no-ops.
 */
inline void Rosmaster::set_uart_servo(int servo_id, int pulse_value, int run_time) {
    try {
        if (!arm_ctrl_enable_) return;
        // Validate the servo id and the valid pulse window (96..4000), non-negative time.
        if (servo_id < 1 || pulse_value < 96 || pulse_value > 4000 || run_time < 0) {
            std::cerr << "set uart servo input error\n"; return;
        }
        run_time = std::max(0, std::min(2000, run_time));
        // Pulse value and run_time each go on the wire as LE int16.
        auto [pvl, pvh] = packI16(static_cast<int16_t>(pulse_value));
        auto [rtl, rth] = packI16(static_cast<int16_t>(run_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_UART_SERVO,
                                    static_cast<uint8_t>(servo_id & 0xff),
                                    pvl, pvh, rtl, rth};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo error!---\n"; }
}

/**
 * @brief Degree-based wrapper over set_uart_servo(): applies the per-joint angular
 *        range then armConvertValue() maps degrees -> raw pulse.
 */
inline void Rosmaster::set_uart_servo_angle(int s_id, int s_angle, int run_time) {
    try {
        // Only forward if the angle is within this joint's allowed max, else warn.
        auto send = [&](int max_a) {
            if (s_angle >= 0 && s_angle <= max_a)
                set_uart_servo(s_id, armConvertValue(s_id, s_angle), run_time);
            else
                std::cerr << "angle_" << s_id << " set error!\n";
        };
        switch (s_id) {
            case 1: case 2: case 3: case 4: send(180); break;
            // Joint 5 (wrist rotate) spans 0..270 deg; the other joints are 0..180.
            case 5: send(270); break;
            case 6: send(180); break;
            default: break;
        }
    } catch (...) { std::cerr << "---set_uart_servo_angle error! ID=" << s_id << "---\n"; }
}

/**
 * @brief Reprogram a bus servo's ID (1..250). Only one servo should be attached
 *        to the bus when calling, since the write is effectively a broadcast.
 */
inline void Rosmaster::set_uart_servo_id(int servo_id) {
    try {
        // Bus IDs are limited to 1..250 (0 and 255 are reserved).
        if (servo_id < 1 || servo_id > 250) { std::cerr << "servo id input error!\n"; return; }
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_UART_SERVO_ID,
                                    static_cast<uint8_t>(servo_id)};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo_id error!---\n"; }
}

/**
 * @brief Enable or free the holding torque on the bus servos.
 */
inline void Rosmaster::set_uart_servo_torque(int enable) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04,
                                    FUNC_UART_SERVO_TORQUE,
                                    // Any positive value -> 1 (hold torque); otherwise 0 (let the joints back-drive).
                                    static_cast<uint8_t>(enable > 0 ? 1 : 0)};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo_torque error!---\n"; }
}

/**
 * @brief Local software gate: when false the arm write helpers early-return.
 *        Pure flag write, performs no serial I/O.
 */
inline void Rosmaster::set_uart_servo_ctrl_enable(bool enable) {
    arm_ctrl_enable_ = enable;
}

/**
 * @brief Move all six arm joints in a single FUNC_ARM_CTRL frame. Validates each
 *        joint's degree range, then packs six pulse pairs plus the run_time.
 */
inline void Rosmaster::set_uart_servo_angle_array(std::vector<int> angle_s, int run_time) {
    try {
        if (!arm_ctrl_enable_) return;
        // Require all 6 joints and enforce per-joint degree limits
        // (joints 1-4 & 6: 0..180, joint 5: 0..270).
        if (angle_s.size() < 6 ||
            angle_s[0]<0||angle_s[0]>180||angle_s[1]<0||angle_s[1]>180||
            angle_s[2]<0||angle_s[2]>180||angle_s[3]<0||angle_s[3]>180||
            angle_s[4]<0||angle_s[4]>270||angle_s[5]<0||angle_s[5]>180) {
            std::cerr << "angle_s input error!\n"; return;
        }
        run_time = std::max(0, std::min(2000, run_time));
        auto [rtl, rth] = packI16(static_cast<int16_t>(run_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_ARM_CTRL};
        // Convert each joint's degrees -> pulse and append as an LE int16 pair.
        for (int i = 0; i < 6; ++i) {
            auto [lo, hi] = packI16(static_cast<int16_t>(armConvertValue(i + 1, angle_s[i])));
            cmd.push_back(lo);
            cmd.push_back(hi);
        }
        cmd.push_back(rtl);
        cmd.push_back(rth);
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo_angle_array error!---\n"; }
}

/**
 * @brief Ask the firmware to store the servo's current position as its zero offset,
 *        then block up to 200 ms polling the ack atomics that receiveLoop() fills.
 *        Returns the reported offset state (or 0 on timeout/error).
 */
inline int Rosmaster::set_uart_servo_offset(int servo_id) {
    try {
        // Reset the ack sentinels first so we only accept the fresh reply for this servo.
        arm_offset_id_    = 0xff;
        arm_offset_state_ = 0;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_ARM_OFFSET,
                                    static_cast<uint8_t>(servo_id & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        // Poll up to 200 x 1 ms for the RX thread to land the matching-id ack.
        for (int i = 0; i < 200; ++i) {
            // Ack for this servo arrived - return its stored offset state immediately.
            if (arm_offset_id_ == servo_id) return arm_offset_state_.load();
            delay_ms(1);
        }
        return arm_offset_state_.load();
    } catch (...) { std::cerr << "---set_uart_servo_offset error!---\n"; return 0; }
}

/**
 * @brief Ackermann steering center/trim angle (valid 60..120 deg, 90 = straight).
 *        forever caches akm_def_angle_ locally and persists it to flash.
 */
inline void Rosmaster::set_akm_default_angle(int angle, bool forever) {
    try {
        // Reject trim angles outside the mechanical 60..120 deg window.
        if (angle > 120 || angle < 60) return;
        // Remember the trim locally only when we are persisting it.
        if (forever) akm_def_angle_ = angle;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_AKM_DEF_ANGLE,
                                    AKM_SERVO_ID,
                                    static_cast<uint8_t>(angle),
                                    static_cast<uint8_t>(forever ? 0x5F : 0)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (forever) delay_ms(100);
    } catch (...) { std::cerr << "---set_akm_default_angle error!---\n"; }
}

/**
 * @brief Command the Ackermann steer angle in [-45,45] deg. ctrl_car sets the high
 *        bit of the servo id so the firmware also spins the drive wheels, not just steers.
 */
inline void Rosmaster::set_akm_steering_angle(int angle, bool ctrl_car) {
    try {
        // Steering is limited to +-45 deg.
        if (angle > 45 || angle < -45) return;
        const uint8_t id = ctrl_car
            // High bit (+0x80) on the id tells the firmware to drive the wheels too.
            ? static_cast<uint8_t>(AKM_SERVO_ID + 0x80)
            : AKM_SERVO_ID;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_AKM_STEER_ANGLE,
                                    id,
                                    // Encode the signed angle as a two's-complement int8 byte.
                                    static_cast<uint8_t>(static_cast<int8_t>(angle))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_akm_steering_angle error!---\n"; }
}

/**
 * @brief Factory-reset the firmware's persisted parameters. 0x5F is the confirm
 *        magic; waits 100 ms for the flash erase/write to complete.
 */
inline void Rosmaster::reset_flash_value() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_FLASH, 0x5F};
        writeCmd(cmd);
        delay_ms(100);
    } catch (...) { std::cerr << "---reset_flash_value error!---\n"; }
}

/**
 * @brief Soft-stop: command the firmware to halt the motors and zero its outputs.
 *        0x5F is the confirm byte.
 */
inline void Rosmaster::reset_car_state() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_STATE, 0x5F};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---reset_car_state error!---\n"; }
}

/**
 * @brief Zero the driver's cached sensor snapshot (battery/IMU/odometry). Local only
 *        - sends no serial frame. These plain members are also written by receiveLoop,
 *        so avoid calling it concurrently with active auto-reporting.
 */
inline void Rosmaster::clear_auto_report_data() {
    battery_voltage_ = 0;
    ax_ = ay_ = az_ = 0.0;
    gx_ = gy_ = gz_ = 0.0;
    mx_ = my_ = mz_ = 0.0;
    vx_ = vy_ = vz_ = 0.0;
    roll_ = pitch_ = yaw_ = 0.0;
}

// =============================================================================
//  Public getters
// =============================================================================

/** @brief Snapshot the latest accelerometer sample (m/s^2, body frame).
 *  Lock-free: plain atomic loads of values receiveLoop wrote from the last
 *  IMU auto-report frame. Never blocks; returns the most recent frame seen. */
inline void Rosmaster::get_accelerometer_data(double & ax, double & ay, double & az) const {
    ax = ax_; ay = ay_; az = az_;
}
/** @brief Snapshot the latest angular-rate sample (rad/s, body frame) via
 *  lock-free atomic loads populated by the receiveLoop IMU parse. */
inline void Rosmaster::get_gyroscope_data(double & gx, double & gy, double & gz) const {
    gx = gx_; gy = gy_; gz = gz_;
}
/** @brief Snapshot the latest magnetometer sample (raw units) via lock-free
 *  atomic loads populated by the receiveLoop IMU parse. */
inline void Rosmaster::get_magnetometer_data(double & mx, double & my, double & mz) const {
    mx = mx_; my = my_; mz = mz_;
}
/** @brief Snapshot the fused attitude. Stored internally in RADIANS (as the
 *  MCU reports it); optionally converts to degrees when to_angle is true. */
inline void Rosmaster::get_imu_attitude_data(double & roll, double & pitch,
                                              double & yaw, bool to_angle) const {
    if (to_angle) {
        // 57.2957795 = 180/pi: radians -> degrees conversion factor.
        constexpr double RtA = 57.2957795;
        roll  = roll_  * RtA;
        pitch = pitch_ * RtA;
        yaw   = yaw_   * RtA;
    } else {
        roll  = roll_;
        pitch = pitch_;
        yaw   = yaw_;
    }
}
/** @brief Snapshot chassis body-frame velocity: vx,vy in m/s, vz in rad/s,
 *  as decoded from the odometry auto-report. Lock-free atomic loads. */
inline void Rosmaster::get_motion_data(double & vx, double & vy, double & vz) const {
    vx = vx_; vy = vy_; vz = vz_;
}
/** @brief Pack voltage in volts. The MCU reports it as an integer in
 *  decivolts (tenths of a volt), so divide by 10 to recover volts. */
inline double Rosmaster::get_battery_voltage() const {
    // decivolts -> volts (firmware sends voltage * 10 as an integer).
    return battery_voltage_ / 10.0;
}
/** @brief Snapshot the four cumulative wheel encoder counters (FL,FR,RL,RR).
 *  These are monotonic int32 totals that never reset in firmware; wrap-safe
 *  deltas are taken elsewhere (PID/odom). Lock-free atomic loads. */
inline void Rosmaster::get_motor_encoder(int & m1, int & m2, int & m3, int & m4) const {
    m1 = encoder_m1_; m2 = encoder_m2_;
    m3 = encoder_m3_; m4 = encoder_m4_;
}

/** @brief Request-then-poll the firmware's own per-wheel speed measurement (raw).
 *  Clears the arrival sentinel, sends FUNC_REQUEST_DATA(0x08), then spins up to
 *  ~30 ms on the atomics receiveLoop fills. Returns std::nullopt on timeout --
 *  never a numeric sentinel, because any value in [-32.768, 32.767] m/s is a
 *  legitimate reading (see the header declaration). */
inline std::optional<std::array<double, 4>> Rosmaster::get_motor_speed_raw() {
    // Clear BEFORE requesting: otherwise a late reply from a previous call could be
    // mistaken for this one's answer.
    motor_speed_raw_ok_.store(false, std::memory_order_release);
    requestData(FUNC_REPORT_MOTOR_RAW);
    for (int i = 0; i < 30; ++i) {
        // acquire pairs with parseData's release store: once the flag reads true, the
        // four speeds are guaranteed visible to this thread.
        if (motor_speed_raw_ok_.load(std::memory_order_acquire)) {
            return std::array<double, 4>{
                motor_speed_raw_[0].load(std::memory_order_relaxed),
                motor_speed_raw_[1].load(std::memory_order_relaxed),
                motor_speed_raw_[2].load(std::memory_order_relaxed),
                motor_speed_raw_[3].load(std::memory_order_relaxed)
            };
        }
        delay_ms(1);
    }
    // No answer: either the link is down, or the board predates the 0x08 report.
    if (debug_) std::cerr << "get_motor_speed_raw: timeout (firmware < V3.5.1?)\n";
    return std::nullopt;
}

/** @brief Same as get_motor_speed_raw(), but pulls the firmware's low-pass-filtered
 *  speed (FUNC_REPORT_MOTOR_LPF, 0x09): quieter, but lagging on acceleration. */
inline std::optional<std::array<double, 4>> Rosmaster::get_motor_speed_lpf() {
    motor_speed_lpf_ok_.store(false, std::memory_order_release);
    requestData(FUNC_REPORT_MOTOR_LPF);
    for (int i = 0; i < 30; ++i) {
        if (motor_speed_lpf_ok_.load(std::memory_order_acquire)) {
            return std::array<double, 4>{
                motor_speed_lpf_[0].load(std::memory_order_relaxed),
                motor_speed_lpf_[1].load(std::memory_order_relaxed),
                motor_speed_lpf_[2].load(std::memory_order_relaxed),
                motor_speed_lpf_[3].load(std::memory_order_relaxed)
            };
        }
        delay_ms(1);
    }
    if (debug_) std::cerr << "get_motor_speed_lpf: timeout (firmware < V3.5.1?)\n";
    return std::nullopt;
}

/** @brief Lock-free snapshot of the PUSHED 0x09 low-pass wheel speeds (m/s). Mirrors
 *  get_motor_encoder(): reads only atomics receiveLoop fills, no request, no blocking.
 *  *fresh is an acquire-load of the arrival flag (pairs with parseData's release store of
 *  the four speeds, so a true flag guarantees the speeds are visible). Once the firmware
 *  pushes 0x09 at 100 Hz and the driver stops calling get_motor_speed_lpf() (which clears
 *  the flag), *fresh stays true for the life of the link -- the push/fallback discriminator. */
inline std::array<double, 4> Rosmaster::get_motor_speed_lpf_data(bool * fresh) const {
    if (fresh) *fresh = motor_speed_lpf_ok_.load(std::memory_order_acquire);
    return std::array<double, 4>{
        motor_speed_lpf_[0].load(std::memory_order_relaxed),
        motor_speed_lpf_[1].load(std::memory_order_relaxed),
        motor_speed_lpf_[2].load(std::memory_order_relaxed),
        motor_speed_lpf_[3].load(std::memory_order_relaxed)
    };
}

/** @brief Read back the MCU's OWN motor-PID gains (distinct from this driver's
 *  software velocity PID). Request-then-poll: fire a FUNC_REQUEST_DATA for
 *  FUNC_SET_MOTOR_PID, then spin the atomics receiveLoop fills. ~20 ms budget;
 *  returns {-1,-1,-1} on timeout. */
inline std::vector<double> Rosmaster::get_motion_pid() {
    // Clear the staging gains and reset pid_index_ (the arrival sentinel:
    // receiveLoop sets pid_index_ > 0 once the reply frame is parsed).
    kp1_ = ki1_ = kd1_ = 0;
    pid_index_ = 0;
    requestData(FUNC_SET_MOTOR_PID, 1);
    for (int i = 0; i < 20; ++i) {
        if (pid_index_ > 0)
            // Gains travel as fixed-point milli-units; /1000 restores the float value.
            return { kp1_ / 1000.0, ki1_ / 1000.0, kd1_ / 1000.0 };
        delay_ms(1);
    }
    return {-1, -1, -1};
}

/** @brief Request-then-poll a single bus servo's (id, raw position). Spin-polls
 *  read_id_ for up to 30 ms. {-1,-1} = bad id or timeout; {-2,-2} = exception. */
inline std::pair<int,int> Rosmaster::get_uart_servo_value(int servo_id) {
    try {
        if (servo_id < 1 || servo_id > 250) return {-1, -1};
        read_id_ = 0; read_val_ = 0;
        // The request param carries the target servo id the MCU should report on.
        requestData(FUNC_UART_SERVO, static_cast<uint8_t>(servo_id));
        for (int t = 30; t > 0; --t) {
            if (read_id_ > 0)
                return { static_cast<int>(read_id_), static_cast<int>(read_val_) };
            delay_ms(1);
        }
        return {-1, -1};
    } catch (...) { return {-2, -2}; }
}

/** @brief Thin wrapper over get_uart_servo_value that converts the raw servo
 *  tick reading into degrees and range-checks it per joint. -1 on out-of-range
 *  or id mismatch, -2 on exception. */
inline int Rosmaster::get_uart_servo_angle(int s_id) {
    try {
        auto [rid, value] = get_uart_servo_value(s_id);
        if (s_id >= 1 && s_id <= 6 && rid == s_id) {
            // Map raw servo ticks -> joint degrees using this joint's calibration.
            const int angle = armConvertAngle(s_id, value);
            // Joint 5 (wrist rotate) has 270 deg of travel; all other joints have 180.
            const int max_a = (s_id == 5) ? 270 : 180;
            if (angle < 0 || angle > max_a) return -1;
            return angle;
        }
        return -1;
    } catch (...) { return -2; }
}

/** @brief Read all 6 arm-joint angles in one request. Because receiveLoop fills
 *  read_arm_[] from another thread, a arm_mutex_ + read_arm_ok_ release/acquire
 *  handshake publishes the buffer. All -1 on timeout, all -2 on exception. */
inline std::vector<int> Rosmaster::get_uart_servo_angle_array() {
    try {
        { std::lock_guard<std::mutex> lk(arm_mutex_);
          for (int i = 0; i < 6; ++i) read_arm_[i] = -1; }
        // Clear the ready flag BEFORE requesting so a stale reply can't be mistaken
        // for this call's fresh data (release publishes the -1 buffer reset above).
        read_arm_ok_.store(0, std::memory_order_release);
        requestData(FUNC_ARM_CTRL, 1);
        std::vector<int> angle(6, -1);
        for (int t = 30; t > 0; --t) {
            // acquire pairs with receiveLoop's release store: once flag==1, read_arm_[]
            // is guaranteed visible and safe to copy under the mutex.
            if (read_arm_ok_.load(std::memory_order_acquire) == 1) {
                std::lock_guard<std::mutex> lk(arm_mutex_);
                for (int i = 0; i < 6; ++i)
                    if (read_arm_[i] > 0)
                        angle[i] = armConvertAngle(i + 1, read_arm_[i]);
                break;
            }
            delay_ms(1);
        }
        return angle;
    } catch (...) { return {-2,-2,-2,-2,-2,-2}; }
}

/** @brief Ackermann steering-servo centre angle. Cached after the first
 *  successful read (akm_readed_angle_), so only the first call hits the bus.
 *  Request-then-poll up to ~1 s (100 x 10 ms); returns -1 on timeout. */
inline int Rosmaster::get_akm_default_angle() {
    if (!akm_readed_angle_) {
        // Param selects which servo (the Ackermann steering servo) to query.
        requestData(FUNC_AKM_DEF_ANGLE, AKM_SERVO_ID);
        for (int i = 0; i < 100; ++i) {
            if (akm_readed_angle_) break;
            delay_ms(10);
        }
        if (!akm_readed_angle_) return -1;
    }
    return akm_def_angle_;
}

/** @brief Firmware version as H.L. Cached in version_ (version_H_==0 means
 *  not-yet-read), so only the first call round-trips the MCU. Request-then-poll
 *  ~20 ms; returns -1.0 on timeout. */
inline double Rosmaster::get_version() {
    if (version_H_ == 0) {
        requestData(FUNC_VERSION);
        for (int i = 0; i < 20; ++i) {
            if (version_H_ != 0) {
                // Assemble version = major + minor/10 (e.g. H=3, L=9 -> 3.9).
                version_ = static_cast<double>(version_H_.load())
                         + static_cast<double>(version_L_.load()) / 10.0;
                return version_;
            }
            delay_ms(1);
        }
        return -1.0;
    }
    return version_;
}

/** @brief Query the chassis/car type stored in the MCU. Request-then-poll
 *  ~20 ms; consume-and-reset each call so it always reflects a fresh reply.
 *  Returns -1 on timeout. */
inline int Rosmaster::get_car_type_from_machine() {
    requestData(FUNC_SET_CAR_TYPE);
    for (int i = 0; i < 20; ++i) {
        if (read_car_type_ != 0) {
            // Latch the reply, then clear the atomic so the next call re-requests.
            const int ct = read_car_type_.load();
            read_car_type_ = 0;
            return ct;
        }
        delay_ms(1);
    }
    return -1;
}
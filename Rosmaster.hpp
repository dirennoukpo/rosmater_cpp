/*
** Rosmaster.hpp for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Fri May 15 17:18:57 2026 dirennoukpo
** Last update Wed Jun 17 00:00:00 2026 dirennoukpo
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

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
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
inline void delay_ms(double ms) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(ms * 1000.0)));
}

// =============================================================================
//  SerialPort — thin POSIX/Win32 wrapper with robust open/close lifecycle
// =============================================================================
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    SerialPort(const SerialPort &)            = delete;
    SerialPort & operator=(const SerialPort&) = delete;

    bool open(const std::string & port, int baud = 115200);
    void close();
    bool isOpen() const;
    int  write(const std::vector<uint8_t> & data);

    // Returns  1 : byte received
    //          0 : timeout (VTIME window elapsed)
    //         -1 : unrecoverable error (EIO/ENOTTY/EBADF — device gone)
    int  readByte(uint8_t & out);
    void flushInput();

private:
#ifdef _WIN32
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
#else
    int fd_ = -1;
#endif
};

// =============================================================================
//  Rosmaster
// =============================================================================
class Rosmaster {
public:
    // car_type: 1=X3, 2=X3_PLUS, 4=X1, 5=R2
    explicit Rosmaster(int car_type = 1,
                       const std::string & com = "/dev/myserial",
                       double delay = 0.002,
                       bool debug = false);
    ~Rosmaster();

    Rosmaster(const Rosmaster &)            = delete;
    Rosmaster & operator=(const Rosmaster&) = delete;

    // ── Threading ─────────────────────────────────────────────────────────
    void create_receive_threading();

    // ── Health check ──────────────────────────────────────────────────────
    bool is_running() const {
        return uart_running_.load(std::memory_order_relaxed);
    }

    // ── Car control ───────────────────────────────────────────────────────
    void set_auto_report_state(bool enable, bool forever = false);
    void set_beep(int on_time);
    void set_pwm_servo(int servo_id, int angle);
    void set_pwm_servo_all(int angle_s1, int angle_s2, int angle_s3, int angle_s4);
    void set_colorful_lamps(int led_id, int red, int green, int blue);
    void set_colorful_effect(int effect, int speed = 255, int parm = 255);
    void set_motor(double speed_1, double speed_2, double speed_3, double speed_4);
    void set_car_run(int state, int speed, bool adjust = false);
    void set_car_motion(double v_x, double v_y, double v_z);
    void set_pid_param(double kp, double ki, double kd, bool forever = false);
    void set_car_type(int car_type);
    void set_uart_servo(int servo_id, int pulse_value, int run_time = 500);
    void set_uart_servo_angle(int s_id, int s_angle, int run_time = 500);
    void set_uart_servo_id(int servo_id);
    void set_uart_servo_torque(int enable);
    void set_uart_servo_ctrl_enable(bool enable);
    void set_uart_servo_angle_array(std::vector<int> angle_s = {90,90,90,90,90,180},
                                    int run_time = 500);
    int  set_uart_servo_offset(int servo_id);
    void set_akm_default_angle(int angle, bool forever = false);
    void set_akm_steering_angle(int angle, bool ctrl_car = false);
    void reset_flash_value();
    void reset_car_state();
    void clear_auto_report_data();

    // ── Getters ───────────────────────────────────────────────────────────
    void   get_accelerometer_data(double & ax, double & ay, double & az) const;
    void   get_gyroscope_data(double & gx, double & gy, double & gz) const;
    void   get_magnetometer_data(double & mx, double & my, double & mz) const;
    void   get_imu_attitude_data(double & roll, double & pitch, double & yaw,
                                 bool to_angle = true) const;
    void   get_motion_data(double & vx, double & vy, double & vz) const;
    double get_battery_voltage() const;
    void   get_motor_encoder(int & m1, int & m2, int & m3, int & m4) const;
    std::vector<double>      get_motion_pid();
    std::pair<int,int>       get_uart_servo_value(int servo_id);
    int                      get_uart_servo_angle(int s_id);
    std::vector<int>         get_uart_servo_angle_array();
    int                      get_akm_default_angle();
    double                   get_version();
    int                      get_car_type_from_machine();

    // ── Software PID control ──────────────────────────────────────────────
    void   enable_pid_control(double kp            = 1.8,
                              double ki            = 0.4,
                              double kd            = 0.05,
                              double ticks_per_sec = 1326.0);
    void   disable_pid_control();
    void   set_pid_gains(double kp, double ki, double kd);
    void   reset_pids();
    double calibrate_pid_scale(int duration_ms = 300);
    double calibrate_pid_scale_at(int throttle_pct  = 60,
                                  int duration_ms   = 800,
                                  bool use_per_motor = true);

    // Injection directe des échelles après calibration externe
    void set_motor_scales(const std::array<double,4> & scales, double global);

    // Accès à writeMotorRaw depuis main() pour la calibration
    void writeMotorRaw_public(const std::array<double,4> & cmd) {
        writeMotorRaw(cmd);
    }

    // ── Per-motor PID gain overrides ──────────────────────────────────────
    // motor_index : 0=FL, 1=FR, 2=RL, 3=RR
    void set_motor_pid_gains(int motor_index,
                             double kp, double ki, double kd,
                             bool override = true);
    void reset_motor_pid_gains(int motor_index);

    // ── Slope compensation ────────────────────────────────────────────────
    void configure_slope_compensation(bool enabled,
                                      double k_gravity,
                                      double deadband_rad,
                                      uint64_t timeout_ns);
    void set_max_pwm_flat(double pwm);
    void update_pitch(double pitch_rad);
    void set_motor_with_compensation(double s1, double s2, double s3, double s4);

    // Last velocity measured by the PID loop (%), {0,0,0,0} when PID disabled.
    std::array<double, 4> get_pid_measured() const {
        return {
            pid_measured_[0].load(std::memory_order_relaxed),
            pid_measured_[1].load(std::memory_order_relaxed),
            pid_measured_[2].load(std::memory_order_relaxed),
            pid_measured_[3].load(std::memory_order_relaxed)
        };
    }

    // ── Protocol / car-type constants ─────────────────────────────────────
    static constexpr uint8_t FUNC_AUTO_REPORT      = 0x01;
    static constexpr uint8_t FUNC_BEEP             = 0x02;
    static constexpr uint8_t FUNC_PWM_SERVO        = 0x03;
    static constexpr uint8_t FUNC_PWM_SERVO_ALL    = 0x04;
    static constexpr uint8_t FUNC_RGB              = 0x05;
    static constexpr uint8_t FUNC_RGB_EFFECT       = 0x06;
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

    static constexpr uint8_t CARTYPE_X3      = 0x01;
    static constexpr uint8_t CARTYPE_X3_PLUS = 0x02;
    static constexpr uint8_t CARTYPE_X1      = 0x04;
    static constexpr uint8_t CARTYPE_R2      = 0x05;

private:
    // ── Protocol constants ────────────────────────────────────────────────
    static constexpr uint8_t HEAD       = 0xFF;
    static constexpr uint8_t DEVICE_ID  = 0xFC;
    static constexpr int     COMPLEMENT = 257 - DEVICE_ID;   // = 5
    static constexpr uint8_t CAR_ADJUST = 0x80;
    static constexpr uint8_t AKM_SERVO_ID = 0x01;

    // ── PID loop tuning constants ─────────────────────────────────────────
    // α = 0.8  →  τ_eq = α/(1−α) × dt = 0.8/0.2 × (1/kPidHz) ≈ 40 ms @ 25 Hz
    static constexpr double kDerivAlpha = 0.8;
    static constexpr int    kPidHz      = 25;
    static constexpr auto   kPidPeriod  = std::chrono::microseconds(1'000'000 / kPidHz);
    // kVelWindow: sliding-window depth for velocity estimation.
    // Encoder packets arrive at ~24.4 Hz (≈41 ms). At kPidHz=25 (dt=40 ms),
    // kVelWindow=10 covers ~400 ms — ample averaging vs. packet jitter.
    // Recalculate if kPidHz or packet rate changes:
    //   kVelWindow ≈ ceil(2 × packet_period_ms / (1000/kPidHz))
    static constexpr int    kVelWindow  = 10;

    // ── Software PID — structs ────────────────────────────────────────────
    //
    // IMPORTANT: these structs MUST be declared before any member variable
    // that uses them (motor_gains_, motor_state_). C++ requires types to be
    // complete at the point of member declaration, even within the same class.
    //
    // PidGains      : shared between threads → protected by pid_gains_mutex_
    // MotorPidState : exclusive to the PID thread → no mutex required
    //
    // Pre-integration anti-windup (standard industrial control pattern):
    //   raw = target + kp·e + ki·∫ + kd·d_filtered
    //   if |raw| < 100  →  ∫ += e·dt        (integration allowed)
    //   cmd = clamp(raw, -100, 100)
    struct PidGains {
        double kp{1.8};
        double ki{0.4};
        double kd{0.05};
    };

    struct MotorPidState {
        double integral{0.0};
        double prev_error{0.0};
        double derivative_filtered{0.0};   // EMA filter on derivative term

        void reset() { integral = prev_error = derivative_filtered = 0.0; }
    };

    // ── Internal helpers ──────────────────────────────────────────────────
    uint8_t checksum(const std::vector<uint8_t> & cmd) const;
    void    writeCmd(std::vector<uint8_t> & cmd);
    void    requestData(uint8_t function, uint8_t param = 0);
    void    parseData(uint8_t ext_type, const std::vector<uint8_t> & ext_data);
    void    receiveLoop();
    int8_t  limitMotorValue(double v) const;
    int     armConvertValue(int s_id, int s_angle) const;
    int     armConvertAngle(int s_id, int s_value) const;

    // ── Software PID internal methods ─────────────────────────────────────
    void pidLoop();
    void writeMotorRaw(const std::array<double, 4> & cmd);

    // ── Slope compensation helpers ─────────────────────────────────────────
    inline double reserve_headroom(double cmd) const {
        return std::clamp(cmd, -max_pwm_flat_, max_pwm_flat_);
    }

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
    static int16_t le16s(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off + 1]) << 8));
    }
    static uint16_t le16u(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off + 1]) << 8));
    }
    static int32_t le32s(const std::vector<uint8_t> & d, size_t off) {
        return static_cast<int32_t>(
            static_cast<uint32_t>(d[off])              |
            (static_cast<uint32_t>(d[off + 1]) <<  8)  |
            (static_cast<uint32_t>(d[off + 2]) << 16)  |
            (static_cast<uint32_t>(d[off + 3]) << 24));
    }
    static std::pair<uint8_t, uint8_t> packI16(int16_t v) {
        return { static_cast<uint8_t>(v & 0xff),
                 static_cast<uint8_t>((v >> 8) & 0xff) };
    }

    // ── Serial port ───────────────────────────────────────────────────────
    SerialPort  ser_;
    std::string port_name_;

    // ── Config ────────────────────────────────────────────────────────────
    double  delay_time_;
    bool    debug_;
    uint8_t car_type_;

    // ── Background receive thread ─────────────────────────────────────────
    // Shutdown sequence (guaranteed by destructor):
    //   1. uart_running_ = false
    //   2. recv_thread_.join()   ← waits for thread exit
    //   3. ser_.close()          ← fd closed AFTER thread is gone
    std::thread       recv_thread_;
    std::atomic<bool> uart_running_{false};

    // ── Sensor cache (atomic for lock-free reads) ─────────────────────────
    std::atomic<double> ax_{0}, ay_{0}, az_{0};
    std::atomic<double> gx_{0}, gy_{0}, gz_{0};
    std::atomic<double> mx_{0}, my_{0}, mz_{0};
    std::atomic<double> vx_{0}, vy_{0}, vz_{0};
    std::atomic<double> roll_{0}, pitch_{0}, yaw_{0};
    std::atomic<int>    encoder_m1_{0}, encoder_m2_{0},
                        encoder_m3_{0}, encoder_m4_{0};
    std::atomic<int>    battery_voltage_{0};

    std::atomic<int>    read_id_{0}, read_val_{0};

    mutable std::mutex  arm_mutex_;
    std::atomic<int>    read_arm_ok_{0};
    int                 read_arm_[6] = {-1,-1,-1,-1,-1,-1};

    std::atomic<uint8_t> version_H_{0}, version_L_{0};
    double               version_{0};

    std::atomic<int>     pid_index_{0};
    std::atomic<int16_t> kp1_{0}, ki1_{0}, kd1_{0};

    std::atomic<int>     arm_offset_id_{0}, arm_offset_state_{0};
    bool                 arm_ctrl_enable_{true};

    std::atomic<int>     akm_def_angle_{100};
    std::atomic<bool>    akm_readed_angle_{false};

    std::atomic<int>     read_car_type_{0};

    // ── Software PID — members ────────────────────────────────────────────

    // Shared gains — always accessed under pid_gains_mutex_
    mutable std::mutex pid_gains_mutex_;
    PidGains           pid_gains_{};

    // Per-motor PID gain overrides (FL, FR, RL, RR)
    // motor_gains_override_[i]==true  → motor i uses motor_gains_[i]
    // motor_gains_override_[i]==false → motor i uses global pid_gains_
    std::array<PidGains, 4> motor_gains_{};
    std::array<bool, 4>     motor_gains_override_{false, false, false, false};

    // Per-motor state — exclusive to the PID thread, no mutex needed
    std::array<MotorPidState, 4> motor_state_{};

    // Setpoints deposited by set_motor(), consumed by pidLoop()
    std::array<std::atomic<double>, 4> target_{};

    // Per-motor velocity scales: ticks/s at 100% cmd.
    // 0.0 → pidLoop() falls back to ticks_per_second_at_100pct_ (global).
    std::array<double, 4> motor_scale_{0.0, 0.0, 0.0, 0.0};

    // Reference encoder values (kept for calibrate_pid_scale compatibility)
    std::array<uint32_t, 4> enc_prev_pid_{0u, 0u, 0u, 0u};

    // Sliding window for velocity estimation in pidLoop()
    std::array<std::array<uint32_t, 4>, kVelWindow> enc_history_{};
    int  enc_history_idx_{0};
    bool enc_history_full_{false};

    // Set by parseData() on first valid FUNC_REPORT_ENCODER packet.
    // enable_pid_control() refuses to start until this flag is true.
    // Reset to false by receiveLoop() on each reconnection.
    std::atomic<bool> encoder_received_{false};

    // Global velocity scale: ticks/s at 100% cmd.
    // Default: 1000 ticks/rev × (0.5 m/s ÷ π×0.12 m) ≈ 1326 ticks/s
    double ticks_per_second_at_100pct_{1326.0};

    // True once calibrate_pid_scale() has completed successfully.
    bool pid_scale_calibrated_{false};

    // pid_enabled_: atomic — set_motor() reads it from the ROS2 thread
    std::atomic<bool> pid_enabled_{false};

    std::thread       pid_thread_;
    std::atomic<bool> pid_running_{false};

    // Last measured velocity per motor (%), written by pidLoop()
    std::array<std::atomic<double>, 4> pid_measured_{};

    // ── Slope compensation ────────────────────────────────────────────────
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
    std::string full = "\\\\.\\" + port;
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
    if (!ReadFile(hSerial_, &out, 1, &rd, nullptr) || rd == 0) return -1;
    return 1;
}
inline void SerialPort::flushInput() { PurgeComm(hSerial_, PURGE_RXCLEAR); }

#else
// ── POSIX ────────────────────────────────────────────────────────────────────

inline bool SerialPort::open(const std::string & port, int baud) {
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

    termios tty{};
    if (tcgetattr(fd_, &tty) < 0) {
        std::cerr << "SerialPort: tcgetattr failed: " << strerror(errno) << "\n";
        ::ioctl(fd_, TIOCNXCL);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

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

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    // FIX-4: clear HUPCL — prevents DTR/RTS drop on close() which would
    // re-enumerate the USB adapter and cause the /dev/ttyUSBx node to vanish.
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS | HUPCL);
    tty.c_cflag |=  CLOCAL | CREAD;
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
    ssize_t n = ::write(fd_, data.data(), data.size());
    return static_cast<int>(n);
}

inline int SerialPort::readByte(uint8_t & out) {
    if (fd_ < 0) return -1;
    ssize_t n;
    do {
        n = ::read(fd_, &out, 1);
    } while (n < 0 && errno == EINTR);

    if (n == 1)  return  1;
    if (n == 0)  return  0;   // VTIME timeout
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;   // EIO / ENOTTY / EBADF — caller must stop and close
}

inline void SerialPort::flushInput() {
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

#endif  // platform

// =============================================================================
//  Rosmaster implementation
// =============================================================================

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
inline uint8_t Rosmaster::checksum(const std::vector<uint8_t> & cmd) const {
    int s = COMPLEMENT;
    for (auto b : cmd) s += b;
    return static_cast<uint8_t>(s & 0xff);
}

// ── writeCmd ──────────────────────────────────────────────────────────────────
inline void Rosmaster::writeCmd(std::vector<uint8_t> & cmd) {
    cmd.push_back(checksum(cmd));
    ser_.write(cmd);
    if (debug_) {
        std::cout << "cmd:";
        for (auto b : cmd) std::cout << " " << std::hex << static_cast<int>(b);
        std::cout << std::dec << "\n";
    }
    delay_ms(delay_time_ * 1000.0);
}

// ── requestData ───────────────────────────────────────────────────────────────
inline void Rosmaster::requestData(uint8_t function, uint8_t param) {
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
inline void Rosmaster::parseData(uint8_t ext_type,
                                  const std::vector<uint8_t> & d) {
    if (ext_type == FUNC_REPORT_SPEED) {
        vx_ = le16s(d, 0) / 1000.0;
        vy_ = le16s(d, 2) / 1000.0;
        vz_ = le16s(d, 4) / 1000.0;
        battery_voltage_ = d[6];
    }
    else if (ext_type == FUNC_REPORT_MPU_RAW) {
        constexpr double gyro_ratio  = 1.0 / 3754.9;
        constexpr double accel_ratio = 1.0 / 1671.84;
        gx_ =  le16s(d,  0) * gyro_ratio;
        gy_ =  le16s(d,  2) * (-gyro_ratio);
        gz_ =  le16s(d,  4) * (-gyro_ratio);
        ax_ =  le16s(d,  6) * accel_ratio;
        ay_ =  le16s(d,  8) * accel_ratio;
        az_ =  le16s(d, 10) * accel_ratio;
        mx_ =  le16s(d, 12) * 1.0;
        my_ =  le16s(d, 14) * 1.0;
        mz_ =  le16s(d, 16) * 1.0;
    }
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
    else if (ext_type == FUNC_REPORT_IMU_ATT) {
        roll_  = le16s(d, 0) / 10000.0;
        pitch_ = le16s(d, 2) / 10000.0;
        yaw_   = le16s(d, 4) / 10000.0;
    }
    else if (ext_type == FUNC_REPORT_ENCODER) {
        encoder_m1_ = le32s(d,  0);
        encoder_m2_ = le32s(d,  4);
        encoder_m3_ = le32s(d,  8);
        encoder_m4_ = le32s(d, 12);
        encoder_received_.store(true, std::memory_order_release);
    }
    else if (ext_type == FUNC_UART_SERVO) {
        read_id_  = d[0];
        read_val_ = le16s(d, 1);
        if (debug_) std::cout << "FUNC_UART_SERVO: " << read_id_ << " " << read_val_ << "\n";
    }
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
    else if (ext_type == FUNC_VERSION) {
        version_H_ = d[0];
        version_L_ = d[1];
        if (debug_) std::cout << "FUNC_VERSION: " << static_cast<int>(version_H_)
                              << " " << static_cast<int>(version_L_) << "\n";
    }
    else if (ext_type == FUNC_SET_MOTOR_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_) std::cout << "FUNC_SET_MOTOR_PID: " << pid_index_
                              << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    else if (ext_type == FUNC_SET_YAW_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_) std::cout << "FUNC_SET_YAW_PID: " << pid_index_
                              << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    else if (ext_type == FUNC_ARM_OFFSET) {
        arm_offset_id_    = d[0];
        arm_offset_state_ = d[1];
        if (debug_) std::cout << "FUNC_ARM_OFFSET: " << arm_offset_id_
                              << " " << arm_offset_state_ << "\n";
    }
    else if (ext_type == FUNC_AKM_DEF_ANGLE) {
        akm_def_angle_    = d[1];
        akm_readed_angle_ = true;
        if (debug_) std::cout << "FUNC_AKM_DEF_ANGLE: " << static_cast<int>(d[0])
                              << " " << akm_def_angle_ << "\n";
    }
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

    auto fatalExit = [&](const char * reason) {
        std::cerr << "Rosmaster[" << port_name_ << "]: " << reason
                  << " — closing port and exiting receive thread.\n";
        uart_running_ = false;
        ser_.close();   // FIX-6
    };

    constexpr int kSilenceTimeoutMs = 1500;
    auto last_valid_head = std::chrono::steady_clock::now();

    while (uart_running_.load(std::memory_order_relaxed)) {

        uint8_t head1 = 0;
        const int r1 = ser_.readByte(head1);
        if (r1 < 0) { fatalExit("serial read error (EIO/ENOTTY?) on HEAD byte"); return; }
        if (r1 == 0 || head1 != HEAD) {
            const auto now = std::chrono::steady_clock::now();
            const auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_valid_head).count();
            if (silent_ms >= kSilenceTimeoutMs) {
                fatalExit("silence watchdog fired — no data for >1500 ms (Yahboom offline?)");
                return;
            }
            continue;
        }
        last_valid_head = std::chrono::steady_clock::now();

        uint8_t head2 = 0;
        if (ser_.readByte(head2) != 1) continue;
        if (head2 != static_cast<uint8_t>(DEVICE_ID - 1)) continue;

        uint8_t ext_len = 0, ext_type = 0;
        if (ser_.readByte(ext_len)  != 1) continue;
        if (ser_.readByte(ext_type) != 1) continue;

        int check_sum = ext_len + ext_type;
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
inline void Rosmaster::create_receive_threading() {
    if (uart_running_.load()) return;
    uart_running_ = true;
    recv_thread_ = std::thread(&Rosmaster::receiveLoop, this);
    std::cout << "----------------create receive threading--------------\n";
    delay_ms(50);
}

// ── limitMotorValue ───────────────────────────────────────────────────────────
inline int8_t Rosmaster::limitMotorValue(double v) const {
    if (static_cast<int>(v) == 127) return 127;
    if (v >  100.0) return  100;
    if (v < -100.0) return -100;
    return static_cast<int8_t>(v);
}

// ── armConvertValue / armConvertAngle ─────────────────────────────────────────
inline int Rosmaster::armConvertValue(int s_id, int s_angle) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
            return static_cast<int>((3100.0-900.0)*(s_angle-180.0)/(0.0-180.0)+900.0);
        case 5:
            return static_cast<int>((3700.0-380.0)*(s_angle-0.0)/(270.0-0.0)+380.0);
        case 6:
            return static_cast<int>((3100.0-900.0)*(s_angle-0.0)/(180.0-0.0)+900.0);
        default: return -1;
    }
}
inline int Rosmaster::armConvertAngle(int s_id, int s_value) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
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
inline void Rosmaster::writeMotorRaw(const std::array<double, 4> & cmd) {
    auto clamp_byte = [](double v) -> uint8_t {
        return static_cast<uint8_t>(
            static_cast<int8_t>(
                std::clamp(static_cast<int>(std::round(v)), -100, 100)));
    };
    std::vector<uint8_t> raw = {
        HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
        clamp_byte(cmd[0]), clamp_byte(cmd[1]),
        clamp_byte(cmd[2]), clamp_byte(cmd[3])
    };
    raw[2] = static_cast<uint8_t>(raw.size() - 1);
    writeCmd(raw);
}

// ── pidLoop ───────────────────────────────────────────────────────────────────
//
// Per-motor control law (runs at kPidHz):
//
//   delta   = enc_now[i] − oldest_enc[i]      (uint32 modulo 2³²)
//   vel     = delta / window_dt               (ticks/s)
//   meas    = clamp(vel / scale × 100, −200, 200)  (%)
//   e       = target − meas
//   d_filt  = α·d_filt_prev + (1−α)·(e − e_prev)/window_dt
//   raw_cmd = target + kp·e + ki·∫ + kd·d_filt
//   if |raw_cmd| < 100  →  ∫ += e·dt           (pre-integration anti-windup)
//   cmd     = clamp(raw_cmd, −100, 100)
//
inline void Rosmaster::pidLoop() {
    // Seed sliding window with current encoder values
    {
        int m1, m2, m3, m4;
        get_motor_encoder(m1, m2, m3, m4);
        const std::array<uint32_t, 4> seed = {
            static_cast<uint32_t>(m1), static_cast<uint32_t>(m2),
            static_cast<uint32_t>(m3), static_cast<uint32_t>(m4)
        };
        for (auto & h : enc_history_) h = seed;
        enc_prev_pid_     = seed;
        enc_history_idx_  = 0;
        enc_history_full_ = false;
    }

    auto t_prev = std::chrono::steady_clock::now();

    while (pid_running_.load(std::memory_order_relaxed)) {

        std::this_thread::sleep_until(t_prev + kPidPeriod);

        const auto   t_now = std::chrono::steady_clock::now();
        const double dt    = std::max(
            std::chrono::duration<double>(t_now - t_prev).count(), 1e-6);
        t_prev = t_now;

        // Skip if the scheduler overslept significantly
        if (dt > 3.0 / kPidHz) continue;

        // ── Read encoders ─────────────────────────────────────────────────
        int m1, m2, m3, m4;
        get_motor_encoder(m1, m2, m3, m4);
        const std::array<uint32_t, 4> enc_now = {
            static_cast<uint32_t>(m1), static_cast<uint32_t>(m2),
            static_cast<uint32_t>(m3), static_cast<uint32_t>(m4)
        };

        // ── Sliding window ────────────────────────────────────────────────
        // oldest_idx points to the slot that will be overwritten next,
        // which holds the value from kVelWindow cycles ago.
        const int oldest_idx = enc_history_full_
            ? (enc_history_idx_ + 1) % kVelWindow
            : 0;

        enc_history_[enc_history_idx_] = enc_now;
        enc_history_idx_ = (enc_history_idx_ + 1) % kVelWindow;
        if (enc_history_idx_ == 0) enc_history_full_ = true;

        const int    n_samples = enc_history_full_ ? kVelWindow : enc_history_idx_;
        const double window_dt = std::max(
            static_cast<double>(n_samples) * dt, 1e-6);

        // ── Gains snapshot (per-motor with global fallback) ───────────────
        std::array<PidGains, 4> gains;
        {
            std::lock_guard<std::mutex> lk(pid_gains_mutex_);
            for (int i = 0; i < 4; ++i)
                gains[i] = motor_gains_override_[i] ? motor_gains_[i] : pid_gains_;
        }

        const double scale_global = std::max(ticks_per_second_at_100pct_, 1.0);

        std::array<double, 4> cmd_out{};

        for (int i = 0; i < 4; ++i) {
            // ── Velocity measurement over sliding window ───────────────────
            // uint32 modulo-2³² arithmetic — correct across counter wraparound
            const int32_t delta = static_cast<int32_t>(
                enc_now[i] - enc_history_[oldest_idx][i]);
            const double velocity = static_cast<double>(delta) / window_dt;

            const double scale_i = (motor_scale_[i] > 0.0)
                ? motor_scale_[i] : scale_global;

            const double measured = std::clamp(
                (velocity / scale_i) * 100.0, -200.0, 200.0);

            pid_measured_[i].store(measured, std::memory_order_relaxed);

            const double target = target_[i].load(std::memory_order_relaxed);
            const double error  = target - measured;

            // ── Filtered derivative (divide by window_dt, not dt) ──────────
            // `measured` is averaged over window_dt; dividing by dt would
            // produce a spike ~(kVelWindow)× too large on every new packet.
            const double raw_deriv =
                (error - motor_state_[i].prev_error) / window_dt;
            motor_state_[i].derivative_filtered =
                kDerivAlpha           * motor_state_[i].derivative_filtered
                + (1.0 - kDerivAlpha) * raw_deriv;
            motor_state_[i].prev_error = error;

            // ── PID output with feedforward ───────────────────────────────
            const double raw_cmd =
                target
                + gains[i].kp * error
                + gains[i].ki * motor_state_[i].integral
                + gains[i].kd * motor_state_[i].derivative_filtered;

            // ── Pre-integration anti-windup ────────────────────────────────
            if (std::abs(raw_cmd) < 100.0) {
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
inline void Rosmaster::enable_pid_control(double kp, double ki, double kd,
                                           double ticks_per_sec) {
    if (!uart_running_.load(std::memory_order_acquire))
        throw std::runtime_error(
            "Rosmaster::enable_pid_control(): receive thread not running — "
            "call create_receive_threading() first");
    if (!encoder_received_.load(std::memory_order_acquire))
        throw std::runtime_error(
            "Rosmaster::enable_pid_control(): no encoder packet received yet — "
            "call set_auto_report_state(true) and wait ~100 ms before enabling PID");

    if (!pid_scale_calibrated_)
        std::cerr << "Rosmaster: WARNING — using default PID scale ("
                  << ticks_per_sec
                  << " ticks/s). Run calibrate_pid_scale() for accurate control.\n";

    if (pid_running_.load()) return;   // idempotent

    ticks_per_second_at_100pct_ = ticks_per_sec;

    {
        std::lock_guard<std::mutex> lk(pid_gains_mutex_);
        pid_gains_ = {kp, ki, kd};
    }

    for (auto & t : target_) t.store(0.0, std::memory_order_relaxed);

    enc_history_full_ = false;
    enc_history_idx_  = 0;

    pid_enabled_.store(true,  std::memory_order_release);
    pid_running_.store(true,  std::memory_order_release);
    pid_thread_ = std::thread(&Rosmaster::pidLoop, this);

    std::cout << "Rosmaster: PID enabled @ " << kPidHz << " Hz"
              << "  window=" << kVelWindow << " cycles"
              << "  scale=" << ticks_per_second_at_100pct_ << " ticks/s\n";
}

// ── disable_pid_control ───────────────────────────────────────────────────────
inline void Rosmaster::disable_pid_control() {
    if (!pid_running_.load()) return;   // idempotent

    pid_running_.store(false, std::memory_order_release);
    if (pid_thread_.joinable()) pid_thread_.join();
    pid_enabled_.store(false, std::memory_order_release);
    reset_pids();

    std::cout << "Rosmaster: PID disabled\n";
}

// ── reset_pids ────────────────────────────────────────────────────────────────
inline void Rosmaster::reset_pids() {
    if (pid_running_.load(std::memory_order_acquire)) {
        std::cerr << "Rosmaster::reset_pids() ignored — PID thread is active. "
                     "Call disable_pid_control() first.\n";
        return;
    }
    for (auto & s : motor_state_) s.reset();
    for (auto & t : target_)      t.store(0.0, std::memory_order_relaxed);
    for (auto & m : pid_measured_) m.store(0.0, std::memory_order_relaxed);
}

// ── set_pid_gains ─────────────────────────────────────────────────────────────
inline void Rosmaster::set_pid_gains(double kp, double ki, double kd) {
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    pid_gains_ = {kp, ki, kd};
}

// ── set_motor_pid_gains ───────────────────────────────────────────────────────
inline void Rosmaster::set_motor_pid_gains(int motor_index,
                                            double kp, double ki, double kd,
                                            bool override) {
    if (motor_index < 0 || motor_index > 3) return;
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    motor_gains_[motor_index]          = {kp, ki, kd};
    motor_gains_override_[motor_index] = override;
}

// ── reset_motor_pid_gains ─────────────────────────────────────────────────────
inline void Rosmaster::reset_motor_pid_gains(int motor_index) {
    if (motor_index < 0 || motor_index > 3) return;
    std::lock_guard<std::mutex> lk(pid_gains_mutex_);
    motor_gains_override_[motor_index] = false;
}

// ── configure_slope_compensation ─────────────────────────────────────────────
inline void Rosmaster::configure_slope_compensation(bool enabled,
                                                     double k_gravity,
                                                     double deadband_rad,
                                                     uint64_t timeout_ns) {
    slope_comp_enabled_ = enabled;
    k_gravity_          = k_gravity;
    deadband_rad_       = deadband_rad;
    pitch_timeout_ns_   = timeout_ns;
}

// ── set_max_pwm_flat ──────────────────────────────────────────────────────────
inline void Rosmaster::set_max_pwm_flat(double pwm) {
    max_pwm_flat_ = std::clamp(pwm, 10.0, 100.0);
}

// ── update_pitch ──────────────────────────────────────────────────────────────
inline void Rosmaster::update_pitch(double pitch_rad) {
    pitch_rad_.store(pitch_rad, std::memory_order_relaxed);
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    last_pitch_time_ns_.store(now_ns, std::memory_order_relaxed);
}

// ── set_motor_with_compensation ───────────────────────────────────────────────
inline void Rosmaster::set_motor_with_compensation(double s1, double s2,
                                                    double s3, double s4) {
    set_motor(
        apply_slope(reserve_headroom(s1)),
        apply_slope(reserve_headroom(s2)),
        apply_slope(reserve_headroom(s3)),
        apply_slope(reserve_headroom(s4)));
}

// ── calibrate_pid_scale ───────────────────────────────────────────────────────
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

    int m1a, m2a, m3a, m4a;
    get_motor_encoder(m1a, m2a, m3a, m4a);

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
        const int32_t delta = static_cast<int32_t>(
            static_cast<uint32_t>(rb[i]) - static_cast<uint32_t>(ra[i]));
        total += std::abs(static_cast<double>(delta));
    }

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

// ── calibrate_pid_scale_at ────────────────────────────────────────────────────
inline double Rosmaster::calibrate_pid_scale_at(int throttle_pct,
                                                  int duration_ms,
                                                  bool use_per_motor) {
    if (pid_enabled_.load())
        throw std::logic_error(
            "Rosmaster::calibrate_pid_scale_at(): disable PID control first");
    if (throttle_pct < 20 || throttle_pct > 80)
        throw std::invalid_argument(
            "Rosmaster::calibrate_pid_scale_at(): throttle_pct must be in [20, 80]");
    if (duration_ms < 200)
        throw std::invalid_argument(
            "Rosmaster::calibrate_pid_scale_at(): duration_ms < 200");

    const double t = static_cast<double>(throttle_pct);

    std::cout << "Rosmaster: calibration @ " << throttle_pct
              << "% for " << duration_ms << " ms\n";

    auto stable_read = [&](int m[4]) {
        int a[4], b[4];
        for (int i = 0; i < 30; ++i) {
            get_motor_encoder(a[0], a[1], a[2], a[3]);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            get_motor_encoder(b[0], b[1], b[2], b[3]);
            if (a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]) {
                for (int j = 0; j < 4; ++j) m[j] = b[j];
                return;
            }
        }
        for (int j = 0; j < 4; ++j) m[j] = b[j];
    };

    int ma[4], mb[4];
    stable_read(ma);

    writeMotorRaw({t, t, t, t});
    delay_ms(duration_ms);
    writeMotorRaw({0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stable_read(mb);

    const double dt    = duration_ms / 1000.0;
    const double ratio = 100.0 / throttle_pct;
    double total = 0.0;

    for (int i = 0; i < 4; ++i) {
        const int32_t delta = static_cast<int32_t>(
            static_cast<uint32_t>(mb[i]) - static_cast<uint32_t>(ma[i]));
        const double ticks     = std::abs(static_cast<double>(delta));
        const double ticks_s   = ticks / dt;
        const double scale_100 = ticks_s * ratio;
        total += ticks;

        std::cout << "  M" << (i+1)
                  << " Δticks=" << static_cast<long>(ticks)
                  << "  @" << throttle_pct << "%=" << ticks_s
                  << "  @100%=" << scale_100 << " ticks/s\n";

        if (use_per_motor) {
            if (ticks_s <= 0.0)
                throw std::runtime_error(
                    "Rosmaster::calibrate_pid_scale_at(): M"
                    + std::to_string(i+1) + " no ticks — check wiring");
            motor_scale_[i] = scale_100;
        }
    }

    const double ticks_at_throttle = (total / 4.0) / dt;
    const double result = ticks_at_throttle * ratio;

    if (result <= 0.0)
        throw std::runtime_error(
            "Rosmaster::calibrate_pid_scale_at(): no encoder ticks measured");

    ticks_per_second_at_100pct_ = result;
    pid_scale_calibrated_ = true;

    if (use_per_motor) {
        std::cout << "Rosmaster: per-motor scales enabled\n";
        for (int i = 0; i < 4; ++i)
            std::cout << "  M" << (i+1) << " = " << motor_scale_[i] << " ticks/s\n";
    }
    std::cout << "Rosmaster: global scale@100% = " << result << " ticks/s\n";
    return result;
}

// ── set_motor_scales ──────────────────────────────────────────────────────────
inline void Rosmaster::set_motor_scales(const std::array<double,4> & scales,
                                         double global) {
    for (int i = 0; i < 4; ++i) motor_scale_[i] = scales[i];
    ticks_per_second_at_100pct_ = global;
    pid_scale_calibrated_ = true;
    std::cout << "Rosmaster: motor_scale_ updated\n";
}

// =============================================================================
//  Public setters
// =============================================================================

inline void Rosmaster::set_auto_report_state(bool enable, bool forever) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05, FUNC_AUTO_REPORT,
                                    static_cast<uint8_t>(enable  ? 1 : 0),
                                    static_cast<uint8_t>(forever ? 0x5F : 0)};
        writeCmd(cmd);
        if (debug_) std::cout << "report: done\n";
    } catch (...) { std::cerr << "---set_auto_report_state error!---\n"; }
}

inline void Rosmaster::set_beep(int on_time) {
    try {
        if (on_time < 0) { std::cerr << "beep input error!\n"; return; }
        auto [lo, hi] = packI16(static_cast<int16_t>(on_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05, FUNC_BEEP, lo, hi};
        writeCmd(cmd);
        if (debug_) std::cout << "beep: done\n";
    } catch (...) { std::cerr << "---set_beep error!---\n"; }
}

inline void Rosmaster::set_pwm_servo(int servo_id, int angle) {
    try {
        if (servo_id < 1 || servo_id > 4) return;
        angle = std::max(0, std::min(180, angle));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO,
                                    static_cast<uint8_t>(servo_id),
                                    static_cast<uint8_t>(angle)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_pwm_servo error!---\n"; }
}

inline void Rosmaster::set_pwm_servo_all(int a1, int a2, int a3, int a4) {
    try {
        auto fix = [](int a) -> uint8_t {
            return (a < 0 || a > 180) ? 255 : static_cast<uint8_t>(a);
        };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO_ALL,
                                    fix(a1), fix(a2), fix(a3), fix(a4)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_pwm_servo_all error!---\n"; }
}

inline void Rosmaster::set_colorful_lamps(int led_id, int red, int green, int blue) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_RGB,
                                    static_cast<uint8_t>(led_id & 0xff),
                                    static_cast<uint8_t>(red    & 0xff),
                                    static_cast<uint8_t>(green  & 0xff),
                                    static_cast<uint8_t>(blue   & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_colorful_lamps error!---\n"; }
}

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
inline void Rosmaster::set_motor(double s1, double s2, double s3, double s4) {
    if (pid_enabled_.load(std::memory_order_acquire)) {
        auto clamp_target = [](double v) { return std::clamp(v, -100.0, 100.0); };
        target_[0].store(clamp_target(s1), std::memory_order_relaxed);
        target_[1].store(clamp_target(s2), std::memory_order_relaxed);
        target_[2].store(clamp_target(s3), std::memory_order_relaxed);
        target_[3].store(clamp_target(s4), std::memory_order_relaxed);
        return;
    }
    try {
        auto pack = [](int8_t v) -> uint8_t { return static_cast<uint8_t>(v); };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
                                    pack(limitMotorValue(s1)),
                                    pack(limitMotorValue(s2)),
                                    pack(limitMotorValue(s3)),
                                    pack(limitMotorValue(s4))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_motor error!---\n"; }
}

inline void Rosmaster::set_car_run(int state, int speed, bool adjust) {
    try {
        uint8_t ct = car_type_;
        if (adjust) ct = static_cast<uint8_t>(ct | CAR_ADJUST);
        auto [lo, hi] = packI16(static_cast<int16_t>(speed));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_CAR_RUN,
                                    ct, static_cast<uint8_t>(state & 0xff), lo, hi};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_car_run error!---\n"; }
}

inline void Rosmaster::set_car_motion(double v_x, double v_y, double v_z) {
    try {
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

inline void Rosmaster::set_pid_param(double kp, double ki, double kd, bool forever) {
    try {
        if (kp > 10 || ki > 10 || kd > 10 || kp < 0 || ki < 0 || kd < 0) {
            std::cerr << "PID value must be:[0, 10.00]\n"; return;
        }
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

inline void Rosmaster::set_car_type(int car_type) {
    if (car_type >= 0 && car_type <= 255) {
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

inline void Rosmaster::set_uart_servo(int servo_id, int pulse_value, int run_time) {
    try {
        if (!arm_ctrl_enable_) return;
        if (servo_id < 1 || pulse_value < 96 || pulse_value > 4000 || run_time < 0) {
            std::cerr << "set uart servo input error\n"; return;
        }
        run_time = std::max(0, std::min(2000, run_time));
        auto [pvl, pvh] = packI16(static_cast<int16_t>(pulse_value));
        auto [rtl, rth] = packI16(static_cast<int16_t>(run_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_UART_SERVO,
                                    static_cast<uint8_t>(servo_id & 0xff),
                                    pvl, pvh, rtl, rth};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo error!---\n"; }
}

inline void Rosmaster::set_uart_servo_angle(int s_id, int s_angle, int run_time) {
    try {
        auto send = [&](int max_a) {
            if (s_angle >= 0 && s_angle <= max_a)
                set_uart_servo(s_id, armConvertValue(s_id, s_angle), run_time);
            else
                std::cerr << "angle_" << s_id << " set error!\n";
        };
        switch (s_id) {
            case 1: case 2: case 3: case 4: send(180); break;
            case 5: send(270); break;
            case 6: send(180); break;
            default: break;
        }
    } catch (...) { std::cerr << "---set_uart_servo_angle error! ID=" << s_id << "---\n"; }
}

inline void Rosmaster::set_uart_servo_id(int servo_id) {
    try {
        if (servo_id < 1 || servo_id > 250) { std::cerr << "servo id input error!\n"; return; }
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_UART_SERVO_ID,
                                    static_cast<uint8_t>(servo_id)};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo_id error!---\n"; }
}

inline void Rosmaster::set_uart_servo_torque(int enable) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04,
                                    FUNC_UART_SERVO_TORQUE,
                                    static_cast<uint8_t>(enable > 0 ? 1 : 0)};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_uart_servo_torque error!---\n"; }
}

inline void Rosmaster::set_uart_servo_ctrl_enable(bool enable) {
    arm_ctrl_enable_ = enable;
}

inline void Rosmaster::set_uart_servo_angle_array(std::vector<int> angle_s, int run_time) {
    try {
        if (!arm_ctrl_enable_) return;
        if (angle_s.size() < 6 ||
            angle_s[0]<0||angle_s[0]>180||angle_s[1]<0||angle_s[1]>180||
            angle_s[2]<0||angle_s[2]>180||angle_s[3]<0||angle_s[3]>180||
            angle_s[4]<0||angle_s[4]>270||angle_s[5]<0||angle_s[5]>180) {
            std::cerr << "angle_s input error!\n"; return;
        }
        run_time = std::max(0, std::min(2000, run_time));
        auto [rtl, rth] = packI16(static_cast<int16_t>(run_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_ARM_CTRL};
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

inline int Rosmaster::set_uart_servo_offset(int servo_id) {
    try {
        arm_offset_id_    = 0xff;
        arm_offset_state_ = 0;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_ARM_OFFSET,
                                    static_cast<uint8_t>(servo_id & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        for (int i = 0; i < 200; ++i) {
            if (arm_offset_id_ == servo_id) return arm_offset_state_.load();
            delay_ms(1);
        }
        return arm_offset_state_.load();
    } catch (...) { std::cerr << "---set_uart_servo_offset error!---\n"; return 0; }
}

inline void Rosmaster::set_akm_default_angle(int angle, bool forever) {
    try {
        if (angle > 120 || angle < 60) return;
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

inline void Rosmaster::set_akm_steering_angle(int angle, bool ctrl_car) {
    try {
        if (angle > 45 || angle < -45) return;
        const uint8_t id = ctrl_car
            ? static_cast<uint8_t>(AKM_SERVO_ID + 0x80)
            : AKM_SERVO_ID;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_AKM_STEER_ANGLE,
                                    id,
                                    static_cast<uint8_t>(static_cast<int8_t>(angle))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
    } catch (...) { std::cerr << "---set_akm_steering_angle error!---\n"; }
}

inline void Rosmaster::reset_flash_value() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_FLASH, 0x5F};
        writeCmd(cmd);
        delay_ms(100);
    } catch (...) { std::cerr << "---reset_flash_value error!---\n"; }
}

inline void Rosmaster::reset_car_state() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_STATE, 0x5F};
        writeCmd(cmd);
    } catch (...) { std::cerr << "---reset_car_state error!---\n"; }
}

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

inline void Rosmaster::get_accelerometer_data(double & ax, double & ay, double & az) const {
    ax = ax_; ay = ay_; az = az_;
}
inline void Rosmaster::get_gyroscope_data(double & gx, double & gy, double & gz) const {
    gx = gx_; gy = gy_; gz = gz_;
}
inline void Rosmaster::get_magnetometer_data(double & mx, double & my, double & mz) const {
    mx = mx_; my = my_; mz = mz_;
}
inline void Rosmaster::get_imu_attitude_data(double & roll, double & pitch,
                                              double & yaw, bool to_angle) const {
    if (to_angle) {
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
inline void Rosmaster::get_motion_data(double & vx, double & vy, double & vz) const {
    vx = vx_; vy = vy_; vz = vz_;
}
inline double Rosmaster::get_battery_voltage() const {
    return battery_voltage_ / 10.0;
}
inline void Rosmaster::get_motor_encoder(int & m1, int & m2, int & m3, int & m4) const {
    m1 = encoder_m1_; m2 = encoder_m2_;
    m3 = encoder_m3_; m4 = encoder_m4_;
}

inline std::vector<double> Rosmaster::get_motion_pid() {
    kp1_ = ki1_ = kd1_ = 0;
    pid_index_ = 0;
    requestData(FUNC_SET_MOTOR_PID, 1);
    for (int i = 0; i < 20; ++i) {
        if (pid_index_ > 0)
            return { kp1_ / 1000.0, ki1_ / 1000.0, kd1_ / 1000.0 };
        delay_ms(1);
    }
    return {-1, -1, -1};
}

inline std::pair<int,int> Rosmaster::get_uart_servo_value(int servo_id) {
    try {
        if (servo_id < 1 || servo_id > 250) return {-1, -1};
        read_id_ = 0; read_val_ = 0;
        requestData(FUNC_UART_SERVO, static_cast<uint8_t>(servo_id));
        for (int t = 30; t > 0; --t) {
            if (read_id_ > 0)
                return { static_cast<int>(read_id_), static_cast<int>(read_val_) };
            delay_ms(1);
        }
        return {-1, -1};
    } catch (...) { return {-2, -2}; }
}

inline int Rosmaster::get_uart_servo_angle(int s_id) {
    try {
        auto [rid, value] = get_uart_servo_value(s_id);
        if (s_id >= 1 && s_id <= 6 && rid == s_id) {
            const int angle = armConvertAngle(s_id, value);
            const int max_a = (s_id == 5) ? 270 : 180;
            if (angle < 0 || angle > max_a) return -1;
            return angle;
        }
        return -1;
    } catch (...) { return -2; }
}

inline std::vector<int> Rosmaster::get_uart_servo_angle_array() {
    try {
        { std::lock_guard<std::mutex> lk(arm_mutex_);
          for (int i = 0; i < 6; ++i) read_arm_[i] = -1; }
        read_arm_ok_.store(0, std::memory_order_release);
        requestData(FUNC_ARM_CTRL, 1);
        std::vector<int> angle(6, -1);
        for (int t = 30; t > 0; --t) {
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

inline int Rosmaster::get_akm_default_angle() {
    if (!akm_readed_angle_) {
        requestData(FUNC_AKM_DEF_ANGLE, AKM_SERVO_ID);
        for (int i = 0; i < 100; ++i) {
            if (akm_readed_angle_) break;
            delay_ms(10);
        }
        if (!akm_readed_angle_) return -1;
    }
    return akm_def_angle_;
}

inline double Rosmaster::get_version() {
    if (version_H_ == 0) {
        requestData(FUNC_VERSION);
        for (int i = 0; i < 20; ++i) {
            if (version_H_ != 0) {
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

inline int Rosmaster::get_car_type_from_machine() {
    requestData(FUNC_SET_CAR_TYPE);
    for (int i = 0; i < 20; ++i) {
        if (read_car_type_ != 0) {
            const int ct = read_car_type_.load();
            read_car_type_ = 0;
            return ct;
        }
        delay_ms(1);
    }
    return -1;
}
/*
** Rosmaster.hpp for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Fri May 15 17:18:57 2026 dirennoukpo
** Last update Sat May 15 17:19:19 2026 dirennoukpo
*/

#pragma once
// rosmaster.hpp — C++ port of Rosmaster Python driver (V3.3.9)
// Corrected version — all bugs from original C++ port fixed.
// Requires C++17 or later.

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <functional>

// ── Platform serial abstraction ──────────────────────────────────────────────
#ifdef _WIN32
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <termios.h>
#  include <unistd.h>
#  include <errno.h>
#endif

// ── Tiny cross-platform delay helper ─────────────────────────────────────────
inline void delay_ms(double ms) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(ms * 1000.0)));
}

// ── Cross-platform serial port ───────────────────────────────────────────────
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    bool open(const std::string& port, int baud = 115200);
    void close();
    bool isOpen() const;

    // Write raw bytes; returns number of bytes written or -1 on error
    int write(const std::vector<uint8_t>& data);

    // Read exactly one byte (blocking); returns -1 on error / 0 on timeout
    int readByte(uint8_t& out);

    // Flush input buffer
    void flushInput();

private:
#ifdef _WIN32
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
#else
    int fd_ = -1;
#endif
};

// ── Rosmaster ────────────────────────────────────────────────────────────────
class Rosmaster {
public:
    // car_type: 1=X3, 2=X3_PLUS, 4=X1, 5=R2
    explicit Rosmaster(int car_type = 1,
                       const std::string& com = "/dev/myserial",
                       double delay = 0.002,
                       bool debug = false);
    ~Rosmaster();

    // Disable copy
    Rosmaster(const Rosmaster&) = delete;
    Rosmaster& operator=(const Rosmaster&) = delete;

    // ── Threading ────────────────────────────────────────────────────────────
    void create_receive_threading();

    // ── Car control ──────────────────────────────────────────────────────────
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
    int  set_uart_servo_offset(int servo_id);  // returns offset_state
    void set_akm_default_angle(int angle, bool forever = false);
    void set_akm_steering_angle(int angle, bool ctrl_car = false);
    void reset_flash_value();
    void reset_car_state();
    void clear_auto_report_data();

    // ── Getters ───────────────────────────────────────────────────────────────
    void   get_accelerometer_data(double& ax, double& ay, double& az) const;
    void   get_gyroscope_data(double& gx, double& gy, double& gz) const;
    void   get_magnetometer_data(double& mx, double& my, double& mz) const;
    void   get_imu_attitude_data(double& roll, double& pitch, double& yaw,
                                 bool to_angle = true) const;
    void   get_motion_data(double& vx, double& vy, double& vz) const;
    double get_battery_voltage() const;
    void   get_motor_encoder(int& m1, int& m2, int& m3, int& m4) const;
    std::vector<double> get_motion_pid();
    std::pair<int,int>  get_uart_servo_value(int servo_id);
    int    get_uart_servo_angle(int s_id);
    std::vector<int> get_uart_servo_angle_array();
    int    get_akm_default_angle();

    // FIX #12 — matches Python which returns a float (e.g. 1.1).
    // Returns the version as a double (e.g. 1.1), or -1 on failure.
    double get_version();

    int    get_car_type_from_machine();

    // ── Function codes (public so callers can reference them) ─────────────────
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
    // ── Protocol constants ────────────────────────────────────────────────────
    static constexpr uint8_t HEAD       = 0xFF;
    static constexpr uint8_t DEVICE_ID  = 0xFC;
    static constexpr int     COMPLEMENT = 257 - DEVICE_ID; // = 5
    static constexpr uint8_t CAR_ADJUST = 0x80;
    static constexpr uint8_t AKM_SERVO_ID = 0x01;

    // ── Internal helpers ──────────────────────────────────────────────────────
    uint8_t checksum(const std::vector<uint8_t>& cmd) const;
    void    writeCmd(std::vector<uint8_t>& cmd);
    void    requestData(uint8_t function, uint8_t param = 0);
    void    parseData(uint8_t ext_type, const std::vector<uint8_t>& ext_data);
    void    receiveLoop();
    int8_t  limitMotorValue(double v) const;
    int     armConvertValue(int s_id, int s_angle) const;
    int     armConvertAngle(int s_id, int s_value) const;

    // ── Little-endian helpers ─────────────────────────────────────────────────
    static int16_t le16s(const std::vector<uint8_t>& d, size_t off) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off+1]) << 8));
    }
    static uint16_t le16u(const std::vector<uint8_t>& d, size_t off) {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(d[off]) |
            (static_cast<uint16_t>(d[off+1]) << 8));
    }
    static int32_t le32s(const std::vector<uint8_t>& d, size_t off) {
        return static_cast<int32_t>(
            static_cast<uint32_t>(d[off])         |
            (static_cast<uint32_t>(d[off+1]) << 8)  |
            (static_cast<uint32_t>(d[off+2]) << 16) |
            (static_cast<uint32_t>(d[off+3]) << 24));
    }
    static std::pair<uint8_t,uint8_t> packI16(int16_t v) {
        return { static_cast<uint8_t>(v & 0xff),
                 static_cast<uint8_t>((v >> 8) & 0xff) };
    }

    // ── Serial port ───────────────────────────────────────────────────────────
    SerialPort ser_;

    // ── Config ────────────────────────────────────────────────────────────────
    double  delay_time_;
    bool    debug_;
    uint8_t car_type_;

    // ── Background thread ─────────────────────────────────────────────────────
    // FIX #7 / FIX #13 — Use a joinable thread (not detached).
    // uart_running_ drives the loop; the destructor sets it false and joins.
    // create_receive_threading() checks uart_running_ so it can only be called
    // once — identical guard to Python's __uart_state == 0 check.
    std::thread       recv_thread_;
    std::atomic<bool> uart_running_{false};

    // ── Sensor cache ──────────────────────────────────────────────────────────
    std::atomic<double> ax_{0}, ay_{0}, az_{0};
    std::atomic<double> gx_{0}, gy_{0}, gz_{0};
    std::atomic<double> mx_{0}, my_{0}, mz_{0};
    std::atomic<double> vx_{0}, vy_{0}, vz_{0};
    std::atomic<double> roll_{0}, pitch_{0}, yaw_{0};
    std::atomic<int>    encoder_m1_{0}, encoder_m2_{0},
                        encoder_m3_{0}, encoder_m4_{0};
    std::atomic<int>    battery_voltage_{0};

    std::atomic<int>    read_id_{0}, read_val_{0};

    // FIX #2 (read_arm_) — protect the plain array with a mutex + a separate
    // release/acquire flag so the compiler cannot reorder stores past the flag.
    mutable std::mutex  arm_mutex_;
    std::atomic<int>    read_arm_ok_{0};
    int read_arm_[6] = {-1,-1,-1,-1,-1,-1};

    std::atomic<uint8_t> version_H_{0}, version_L_{0};
    double version_{0};   // written/read only from main thread after version_H_ is set

    std::atomic<int>    pid_index_{0};
    std::atomic<int16_t> kp1_{0}, ki1_{0}, kd1_{0};

    std::atomic<int>    arm_offset_id_{0}, arm_offset_state_{0};
    bool                arm_ctrl_enable_{true};

    std::atomic<int>    akm_def_angle_{100};
    std::atomic<bool>   akm_readed_angle_{false};

    std::atomic<int>    read_car_type_{0};
};

// ═══════════════════════════════════════════════════════════════════════════════
//  SerialPort implementation
// ═══════════════════════════════════════════════════════════════════════════════

#ifdef _WIN32
inline bool SerialPort::open(const std::string& port, int baud) {
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
    to.ReadIntervalTimeout         = 0;
    to.ReadTotalTimeoutConstant    = 5000;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.WriteTotalTimeoutConstant   = 1000;
    to.WriteTotalTimeoutMultiplier = 0;
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
inline int SerialPort::write(const std::vector<uint8_t>& data) {
    DWORD written = 0;
    WriteFile(hSerial_, data.data(), (DWORD)data.size(), &written, nullptr);
    return (int)written;
}
inline int SerialPort::readByte(uint8_t& out) {
    DWORD rd = 0;
    if (!ReadFile(hSerial_, &out, 1, &rd, nullptr) || rd == 0) return -1;
    return 1;
}
inline void SerialPort::flushInput() { PurgeComm(hSerial_, PURGE_RXCLEAR); }

#else // POSIX

inline bool SerialPort::open(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;

    termios tty{};
    tcgetattr(fd_, &tty);

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
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag  = IGNPAR;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;

    tty.c_cc[VMIN]  = 0;  // non-blocking so the receive loop can check uart_running_
    tty.c_cc[VTIME] = 5;  // 500 ms timeout per read() call

    tcsetattr(fd_, TCSANOW, &tty);
    return true;
}
inline void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}
inline bool SerialPort::isOpen() const { return fd_ >= 0; }
inline int SerialPort::write(const std::vector<uint8_t>& data) {
    return (int)::write(fd_, data.data(), data.size());
}
inline int SerialPort::readByte(uint8_t& out) {
    ssize_t n = ::read(fd_, &out, 1);
    return (int)n; // returns 0 on timeout, -1 on error, 1 on success
}
inline void SerialPort::flushInput() { tcflush(fd_, TCIFLUSH); }

#endif // platform serial

// ═══════════════════════════════════════════════════════════════════════════════
//  Rosmaster implementation
// ═══════════════════════════════════════════════════════════════════════════════

inline Rosmaster::Rosmaster(int car_type, const std::string& com,
                            double delay, bool debug)
    : delay_time_(delay), debug_(debug),
      car_type_(static_cast<uint8_t>(car_type))
{
    if (!ser_.open(com, 115200))
        throw std::runtime_error("Serial Open Failed: " + com);

    if (ser_.isOpen())
        std::cout << "Rosmaster Serial Opened! Baudrate=115200\n";
    else
        throw std::runtime_error("Serial Open Failed!");

    if (debug_)
        std::cout << "cmd_delay=" << delay_time_ << "s\n";

    set_uart_servo_torque(1);
    delay_ms(2);
}

inline Rosmaster::~Rosmaster() {
    // FIX #13 — signal the loop to stop, then JOIN (not detach).
    // The POSIX read() has VTIME=5 (500 ms) timeout so the thread will wake
    // and exit promptly after uart_running_ goes false.
    uart_running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    ser_.close();
    std::cout << "serial Close!\n";
}

// ── checksum ──────────────────────────────────────────────────────────────────
// Matches Python: sum(cmd, self.__COMPLEMENT) & 0xff
// COMPLEMENT = 257 - DEVICE_ID = 5; used as initial accumulator value.
inline uint8_t Rosmaster::checksum(const std::vector<uint8_t>& cmd) const {
    int s = COMPLEMENT;
    for (auto b : cmd) s += b;
    return static_cast<uint8_t>(s & 0xff);
}

// ── writeCmd ──────────────────────────────────────────────────────────────────
inline void Rosmaster::writeCmd(std::vector<uint8_t>& cmd) {
    uint8_t cs = checksum(cmd);
    cmd.push_back(cs);
    ser_.write(cmd);
    if (debug_) {
        std::cout << "cmd:";
        for (auto b : cmd) std::cout << " " << std::hex << (int)b;
        std::cout << std::dec << "\n";
    }
    // delay_time_ is in seconds; delay_ms() expects milliseconds.
    delay_ms(delay_time_ * 1000.0);
}

// ── requestData ───────────────────────────────────────────────────────────────
inline void Rosmaster::requestData(uint8_t function, uint8_t param) {
    std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05, FUNC_REQUEST_DATA,
                                 function, param};
    uint8_t cs = checksum(cmd);
    cmd.push_back(cs);
    ser_.write(cmd);
    if (debug_) {
        std::cout << "request:";
        for (auto b : cmd) std::cout << " " << std::hex << (int)b;
        std::cout << std::dec << "\n";
    }
    delay_ms(2);
}

// ── parseData ─────────────────────────────────────────────────────────────────
inline void Rosmaster::parseData(uint8_t ext_type,
                                  const std::vector<uint8_t>& d) {
    if (ext_type == FUNC_REPORT_SPEED) {
        vx_ = le16s(d, 0) / 1000.0;
        vy_ = le16s(d, 2) / 1000.0;
        vz_ = le16s(d, 4) / 1000.0;
        battery_voltage_ = d[6];
    }
    else if (ext_type == FUNC_REPORT_MPU_RAW) {
        constexpr double gyro_ratio  = 1.0 / 3754.9;
        constexpr double accel_ratio = 1.0 / 1671.84;
        constexpr double mag_ratio   = 1.0;
        gx_ =  le16s(d,  0) * gyro_ratio;
        gy_ =  le16s(d,  2) * (-gyro_ratio);
        gz_ =  le16s(d,  4) * (-gyro_ratio);
        ax_ =  le16s(d,  6) * accel_ratio;
        ay_ =  le16s(d,  8) * accel_ratio;
        az_ =  le16s(d, 10) * accel_ratio;
        mx_ =  le16s(d, 12) * mag_ratio;
        my_ =  le16s(d, 14) * mag_ratio;
        mz_ =  le16s(d, 16) * mag_ratio;
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
    }
    else if (ext_type == FUNC_UART_SERVO) {
        read_id_  = d[0];
        read_val_ = le16s(d, 1);
        if (debug_)
            std::cout << "FUNC_UART_SERVO: " << read_id_ << " " << read_val_ << "\n";
    }
    else if (ext_type == FUNC_ARM_CTRL) {
        // FIX #2 — use mutex + explicit release fence so the compiler cannot
        // reorder the array stores past the read_arm_ok_ store.
        {
            std::lock_guard<std::mutex> lk(arm_mutex_);
            for (int i = 0; i < 6; ++i)
                read_arm_[i] = le16s(d, i * 2);
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
        if (debug_)
            std::cout << "FUNC_VERSION: " << (int)version_H_
                      << " " << (int)version_L_ << "\n";
    }
    else if (ext_type == FUNC_SET_MOTOR_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_)
            std::cout << "FUNC_SET_MOTOR_PID: " << pid_index_
                      << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    else if (ext_type == FUNC_SET_YAW_PID) {
        pid_index_ = d[0];
        kp1_ = le16s(d, 1);
        ki1_ = le16s(d, 3);
        kd1_ = le16s(d, 5);
        if (debug_)
            std::cout << "FUNC_SET_YAW_PID: " << pid_index_
                      << " [" << kp1_ << "," << ki1_ << "," << kd1_ << "]\n";
    }
    else if (ext_type == FUNC_ARM_OFFSET) {
        arm_offset_id_    = d[0];
        arm_offset_state_ = d[1];
        if (debug_)
            std::cout << "FUNC_ARM_OFFSET: " << arm_offset_id_
                      << " " << arm_offset_state_ << "\n";
    }
    else if (ext_type == FUNC_AKM_DEF_ANGLE) {
        akm_def_angle_    = d[1]; // d[0] is the servo id, d[1] is the angle
        akm_readed_angle_ = true;
        if (debug_)
            std::cout << "FUNC_AKM_DEF_ANGLE: " << (int)d[0]
                      << " " << akm_def_angle_ << "\n";
    }
    else if (ext_type == FUNC_SET_CAR_TYPE) {
        read_car_type_ = d[0];
    }
}

// ── receiveLoop ───────────────────────────────────────────────────────────────
// FIX #3 — The incoming checksum is computed by the robot firmware using the
// same algorithm as our outgoing checksum: COMPLEMENT + sum(bytes) & 0xff.
// The original C++ omitted COMPLEMENT from the receive-side calculation.
// Python: check_sum = ext_len + ext_type, then accumulates all data bytes
// except the last one (which is rx_check_num), then checks check_sum % 256.
// The Python receiver does NOT add COMPLEMENT on receive — it only adds
// ext_len + ext_type + data[0..n-2].  We replicate that exactly.
//
// FIX #13 — Loop condition is uart_running_; read() has VTIME=5 so it
// returns at most every 500 ms even when no data arrives, letting the loop
// re-check uart_running_ promptly. The thread is joined, not detached.
inline void Rosmaster::receiveLoop() {
    ser_.flushInput();
    while (uart_running_.load(std::memory_order_relaxed)) {
        uint8_t head1 = 0;
        if (ser_.readByte(head1) <= 0) continue;
        if (head1 != HEAD) continue;

        uint8_t head2 = 0;
        if (ser_.readByte(head2) <= 0) continue;
        if (head2 != static_cast<uint8_t>(DEVICE_ID - 1)) continue;

        uint8_t ext_len = 0, ext_type = 0;
        if (ser_.readByte(ext_len)  <= 0) continue;
        if (ser_.readByte(ext_type) <= 0) continue;

        // FIX #3 — receive checksum accumulator: starts with ext_len + ext_type,
        // accumulates all payload bytes except the final one (the checksum byte).
        // This matches the Python __receive_data() logic exactly.
        int check_sum = ext_len + ext_type;
        int data_len  = ext_len - 2;
        if (data_len < 0) continue; // malformed frame

        std::vector<uint8_t> ext_data;
        ext_data.reserve(static_cast<size_t>(data_len));
        uint8_t rx_check_num = 0;

        while (static_cast<int>(ext_data.size()) < data_len) {
            uint8_t val = 0;
            if (ser_.readByte(val) <= 0) break;
            ext_data.push_back(val);
            if (static_cast<int>(ext_data.size()) == data_len) {
                // Last byte is the checksum, do NOT add it to check_sum
                rx_check_num = val;
            } else {
                check_sum += val;
            }
        }
        if (static_cast<int>(ext_data.size()) < data_len) continue;

        if ((check_sum % 256) == rx_check_num)
            parseData(ext_type, ext_data);
        else if (debug_)
            std::cout << "check sum error: len=" << (int)ext_len
                      << " type=" << std::hex << (int)ext_type
                      << " got=" << (int)rx_check_num
                      << " expected=" << (check_sum % 256)
                      << std::dec << "\n";
    }
}

// ── create_receive_threading ──────────────────────────────────────────────────
// FIX #7 — Guard with uart_running_ (identical to Python's __uart_state == 0).
// Thread is NOT detached; it is joined in the destructor.
inline void Rosmaster::create_receive_threading() {
    if (uart_running_.load()) return;   // already running
    uart_running_ = true;
    recv_thread_ = std::thread(&Rosmaster::receiveLoop, this);
    // Do NOT detach: the destructor sets uart_running_=false and joins.
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

// ── armConvertValue ───────────────────────────────────────────────────────────
inline int Rosmaster::armConvertValue(int s_id, int s_angle) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
            return static_cast<int>((3100.0 - 900.0) * (s_angle - 180.0) / (0.0 - 180.0) + 900.0);
        case 5:
            return static_cast<int>((3700.0 - 380.0) * (s_angle - 0.0)   / (270.0 - 0.0) + 380.0);
        case 6:
            return static_cast<int>((3100.0 - 900.0) * (s_angle - 0.0)   / (180.0 - 0.0) + 900.0);
        default: return -1;
    }
}

// ── armConvertAngle ───────────────────────────────────────────────────────────
inline int Rosmaster::armConvertAngle(int s_id, int s_value) const {
    switch (s_id) {
        case 1: case 2: case 3: case 4:
            return static_cast<int>((s_value - 900.0)*(0.0 - 180.0)/(3100.0 - 900.0) + 180.0 + 0.5);
        case 5:
            return static_cast<int>((270.0 - 0.0)*(s_value - 380.0)/(3700.0 - 380.0) + 0.0 + 0.5);
        case 6:
            return static_cast<int>((180.0 - 0.0)*(s_value - 900.0)/(3100.0 - 900.0) + 0.0 + 0.5);
        default: return -1;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Public setters
// ═══════════════════════════════════════════════════════════════════════════════

inline void Rosmaster::set_auto_report_state(bool enable, bool forever) {
    try {
        uint8_t state1 = enable  ? 1    : 0;
        uint8_t state2 = forever ? 0x5F : 0;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x05,
                                    FUNC_AUTO_REPORT, state1, state2};
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
        if (servo_id < 1 || servo_id > 4) {
            if (debug_) std::cerr << "set_pwm_servo input invalid\n";
            return;
        }
        angle = std::max(0, std::min(180, angle));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO,
                                    static_cast<uint8_t>(servo_id),
                                    static_cast<uint8_t>(angle)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "pwmServo: done\n";
    } catch (...) { std::cerr << "---set_pwm_servo error!---\n"; }
}

inline void Rosmaster::set_pwm_servo_all(int a1, int a2, int a3, int a4) {
    try {
        // Out-of-range angle → 255 (means "don't change this channel"), matching Python.
        auto fix = [](int a) -> uint8_t {
            return (a < 0 || a > 180) ? 255 : static_cast<uint8_t>(a);
        };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_PWM_SERVO_ALL,
                                    fix(a1), fix(a2), fix(a3), fix(a4)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "all Servo: done\n";
    } catch (...) { std::cerr << "---set_pwm_servo_all error!---\n"; }
}

inline void Rosmaster::set_colorful_lamps(int led_id, int red, int green, int blue) {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_RGB,
                                    static_cast<uint8_t>(led_id & 0xff),
                                    static_cast<uint8_t>(red   & 0xff),
                                    static_cast<uint8_t>(green & 0xff),
                                    static_cast<uint8_t>(blue  & 0xff)};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "rgb: done\n";
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
        if (debug_) std::cout << "rgb_effect: done\n";
    } catch (...) { std::cerr << "---set_colorful_effect error!---\n"; }
}

inline void Rosmaster::set_motor(double s1, double s2, double s3, double s4) {
    try {
        // Cast through uint8_t to preserve the bit-pattern of signed int8_t.
        auto pack = [](int8_t v) -> uint8_t { return static_cast<uint8_t>(v); };
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
                                    pack(limitMotorValue(s1)),
                                    pack(limitMotorValue(s2)),
                                    pack(limitMotorValue(s3)),
                                    pack(limitMotorValue(s4))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "motor: done\n";
    } catch (...) { std::cerr << "---set_motor error!---\n"; }
}

inline void Rosmaster::set_car_run(int state, int speed, bool adjust) {
    try {
        uint8_t ct = car_type_;
        if (adjust) ct = static_cast<uint8_t>(ct | CAR_ADJUST);
        auto [lo, hi] = packI16(static_cast<int16_t>(speed));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_CAR_RUN,
                                    ct,
                                    static_cast<uint8_t>(state & 0xff),
                                    lo, hi};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "car_run: done\n";
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
        if (debug_) std::cout << "motion: done\n";
    } catch (...) { std::cerr << "---set_car_motion error!---\n"; }
}

inline void Rosmaster::set_pid_param(double kp, double ki, double kd, bool forever) {
    try {
        if (kp > 10 || ki > 10 || kd > 10 || kp < 0 || ki < 0 || kd < 0) {
            std::cerr << "PID value must be:[0, 10.00]\n"; return;
        }
        uint8_t state = forever ? 0x5F : 0;
        auto [kpl, kph] = packI16(static_cast<int16_t>(kp * 1000.0));
        auto [kil, kih] = packI16(static_cast<int16_t>(ki * 1000.0));
        auto [kdl, kdh] = packI16(static_cast<int16_t>(kd * 1000.0));
        // Length byte (index 2) is hardcoded to 0x0A as in the Python source.
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x0A, FUNC_SET_MOTOR_PID,
                                    kpl, kph, kil, kih, kdl, kdh, state};
        writeCmd(cmd);
        if (debug_) std::cout << "pid: done\n";
        if (forever) delay_ms(100);
    } catch (...) { std::cerr << "---set_pid_param error!---\n"; }
}

inline void Rosmaster::set_car_type(int car_type) {
    // FIX — Python uses str(car_type).isdigit() which rejects negatives and
    // non-integers.  We mirror that by accepting [0, 255] integers only.
    if (car_type >= 0 && car_type <= 255) {
        car_type_ = static_cast<uint8_t>(car_type);
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_SET_CAR_TYPE,
                                    car_type_, 0x5F};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "car_type: done\n";
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
        if (debug_)
            std::cout << "uartServo: " << servo_id << " " << pulse_value << "\n";
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
    } catch (...) {
        std::cerr << "---set_uart_servo_angle error! ID=" << s_id << "---\n";
    }
}

inline void Rosmaster::set_uart_servo_id(int servo_id) {
    try {
        if (servo_id < 1 || servo_id > 250) {
            std::cerr << "servo id input error!\n"; return;
        }
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_UART_SERVO_ID,
                                    static_cast<uint8_t>(servo_id)};
        writeCmd(cmd);
        if (debug_) std::cout << "uartServo_id: done\n";
    } catch (...) { std::cerr << "---set_uart_servo_id error!---\n"; }
}

inline void Rosmaster::set_uart_servo_torque(int enable) {
    try {
        uint8_t on = (enable > 0) ? 1 : 0;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04,
                                    FUNC_UART_SERVO_TORQUE, on};
        writeCmd(cmd);
        if (debug_) std::cout << "uartServo_torque: done\n";
    } catch (...) { std::cerr << "---set_uart_servo_torque error!---\n"; }
}

inline void Rosmaster::set_uart_servo_ctrl_enable(bool enable) {
    arm_ctrl_enable_ = enable;
}

inline void Rosmaster::set_uart_servo_angle_array(std::vector<int> angle_s,
                                                   int run_time) {
    try {
        if (!arm_ctrl_enable_) return;
        if (angle_s.size() < 6) { std::cerr << "angle_s input error!\n"; return; }
        if (angle_s[0]<0||angle_s[0]>180||
            angle_s[1]<0||angle_s[1]>180||
            angle_s[2]<0||angle_s[2]>180||
            angle_s[3]<0||angle_s[3]>180||
            angle_s[4]<0||angle_s[4]>270||
            angle_s[5]<0||angle_s[5]>180) {
            std::cerr << "angle_s input error!\n"; return;
        }
        run_time = std::max(0, std::min(2000, run_time));

        int16_t vals[6];
        for (int i = 0; i < 6; ++i)
            vals[i] = static_cast<int16_t>(armConvertValue(i + 1, angle_s[i]));

        auto [rtl, rth] = packI16(static_cast<int16_t>(run_time));
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_ARM_CTRL};
        for (int i = 0; i < 6; ++i) {
            auto [lo, hi] = packI16(vals[i]);
            cmd.push_back(lo);
            cmd.push_back(hi);
        }
        cmd.push_back(rtl);
        cmd.push_back(rth);
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "arm: done\n";
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
        if (debug_) std::cout << "uartServo_offset: done\n";
        for (int i = 0; i < 200; ++i) {
            if (arm_offset_id_ == servo_id) {
                if (debug_) {
                    if (servo_id == 0)
                        std::cout << "Arm Reset Offset Value\n";
                    else
                        std::cout << "Arm Offset State: "
                                  << arm_offset_id_ << " "
                                  << arm_offset_state_ << " " << i << "\n";
                }
                return arm_offset_state_;
            }
            delay_ms(1);
        }
        return arm_offset_state_;
    } catch (...) {
        std::cerr << "---set_uart_servo_offset error!---\n";
        return 0;
    }
}

inline void Rosmaster::set_akm_default_angle(int angle, bool forever) {
    try {
        if (angle > 120 || angle < 60) return;
        uint8_t state = forever ? 0x5F : 0;
        if (forever) akm_def_angle_ = angle;
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_AKM_DEF_ANGLE,
                                    AKM_SERVO_ID,
                                    static_cast<uint8_t>(angle),
                                    state};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "akm set def angle: done\n";
        if (forever) delay_ms(100);
    } catch (...) { std::cerr << "---set_akm_default_angle error!---\n"; }
}

inline void Rosmaster::set_akm_steering_angle(int angle, bool ctrl_car) {
    try {
        if (angle > 45 || angle < -45) return;
        uint8_t id = ctrl_car ? static_cast<uint8_t>(AKM_SERVO_ID + 0x80)
                              : AKM_SERVO_ID;
        // Cast through int8_t first to preserve sign, then to uint8_t for the
        // packet — matches Python's int(angle) & 0xFF behaviour.
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_AKM_STEER_ANGLE,
                                    id,
                                    static_cast<uint8_t>(static_cast<int8_t>(angle))};
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);
        writeCmd(cmd);
        if (debug_) std::cout << "akm_steering_angle: done\n";
    } catch (...) { std::cerr << "---set_akm_steering_angle error!---\n"; }
}

inline void Rosmaster::reset_flash_value() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_FLASH, 0x5F};
        writeCmd(cmd);
        if (debug_) std::cout << "flash: done\n";
        delay_ms(100);
    } catch (...) { std::cerr << "---reset_flash_value error!---\n"; }
}

inline void Rosmaster::reset_car_state() {
    try {
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x04, FUNC_RESET_STATE, 0x5F};
        writeCmd(cmd);
        if (debug_) std::cout << "reset_car_state: done\n";
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

// ═══════════════════════════════════════════════════════════════════════════════
//  Public getters
// ═══════════════════════════════════════════════════════════════════════════════

inline void Rosmaster::get_accelerometer_data(double& ax, double& ay, double& az) const {
    ax = ax_; ay = ay_; az = az_;
}
inline void Rosmaster::get_gyroscope_data(double& gx, double& gy, double& gz) const {
    gx = gx_; gy = gy_; gz = gz_;
}
inline void Rosmaster::get_magnetometer_data(double& mx, double& my, double& mz) const {
    mx = mx_; my = my_; mz = mz_;
}
inline void Rosmaster::get_imu_attitude_data(double& roll, double& pitch,
                                              double& yaw, bool to_angle) const {
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
inline void Rosmaster::get_motion_data(double& vx, double& vy, double& vz) const {
    vx = vx_; vy = vy_; vz = vz_;
}
inline double Rosmaster::get_battery_voltage() const {
    return battery_voltage_ / 10.0;
}
inline void Rosmaster::get_motor_encoder(int& m1, int& m2, int& m3, int& m4) const {
    m1 = encoder_m1_; m2 = encoder_m2_;
    m3 = encoder_m3_; m4 = encoder_m4_;
}

inline std::vector<double> Rosmaster::get_motion_pid() {
    kp1_ = ki1_ = kd1_ = 0;
    pid_index_ = 0;
    requestData(FUNC_SET_MOTOR_PID, 1);
    for (int i = 0; i < 20; ++i) {
        if (pid_index_ > 0) {
            double kp = kp1_ / 1000.0;
            double ki = ki1_ / 1000.0;
            double kd = kd1_ / 1000.0;
            if (debug_)
                std::cout << "get_motion_pid: " << pid_index_
                          << " [" << kp << "," << ki << "," << kd << "] i=" << i << "\n";
            return {kp, ki, kd};
        }
        delay_ms(1);
    }
    return {-1, -1, -1};
}

inline std::pair<int,int> Rosmaster::get_uart_servo_value(int servo_id) {
    try {
        if (servo_id < 1 || servo_id > 250) {
            std::cerr << "get servo id input error!\n"; return {-1,-1};
        }
        read_id_ = 0; read_val_ = 0;
        requestData(FUNC_UART_SERVO, static_cast<uint8_t>(servo_id));
        for (int t = 30; t > 0; --t) {
            if (read_id_ > 0) return {static_cast<int>(read_id_),
                                      static_cast<int>(read_val_)};
            delay_ms(1);
        }
        return {-1, -1};
    } catch (...) {
        std::cerr << "---get_uart_servo_value error!---\n";
        return {-2, -2};
    }
}

inline int Rosmaster::get_uart_servo_angle(int s_id) {
    try {
        auto [read_id, value] = get_uart_servo_value(s_id);
        int max_a = (s_id == 5) ? 270 : 180;
        if (s_id >= 1 && s_id <= 6 && read_id == s_id) {
            int angle = armConvertAngle(s_id, value);
            if (angle < 0 || angle > max_a) {
                if (debug_) std::cerr << "read servo:" << s_id << " out of range!\n";
                return -1;
            }
            if (debug_)
                std::cout << "request angle " << s_id << ": "
                          << read_id << " " << value << "\n";
            return angle;
        }
        if (debug_) std::cerr << "read servo:" << s_id << " error!\n";
        return -1;
    } catch (...) {
        std::cerr << "---get_uart_servo_angle error!---\n";
        return -2;
    }
}

inline std::vector<int> Rosmaster::get_uart_servo_angle_array() {
    try {
        {
            std::lock_guard<std::mutex> lk(arm_mutex_);
            for (int i = 0; i < 6; ++i) read_arm_[i] = -1;
        }
        read_arm_ok_.store(0, std::memory_order_release);
        requestData(FUNC_ARM_CTRL, 1);

        std::vector<int> angle(6, -1);
        for (int t = 30; t > 0; --t) {
            if (read_arm_ok_.load(std::memory_order_acquire) == 1) {
                std::lock_guard<std::mutex> lk(arm_mutex_);
                for (int i = 0; i < 6; ++i)
                    if (read_arm_[i] > 0)
                        angle[i] = armConvertAngle(i + 1, read_arm_[i]);
                if (debug_) {
                    std::cout << "angle_array:";
                    for (int a : angle) std::cout << " " << a;
                    std::cout << "\n";
                }
                break;
            }
            delay_ms(1);
        }
        return angle;
    } catch (...) {
        std::cerr << "---get_uart_servo_angle_array error!---\n";
        return {-2,-2,-2,-2,-2,-2};
    }
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

// FIX #12 — Return type changed to double, matching Python's get_version()
// which returns a float like 1.1, not an int like 11.
inline double Rosmaster::get_version() {
    if (version_H_ == 0) {
        requestData(FUNC_VERSION);
        for (int i = 0; i < 20; ++i) {
            if (version_H_ != 0) {
                double val = static_cast<double>(version_H_.load());
                version_ = val + static_cast<double>(version_L_.load()) / 10.0;
                if (debug_)
                    std::cout << "get_version: V" << version_ << " i=" << i << "\n";
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
            int ct = read_car_type_;
            read_car_type_ = 0;
            return ct;
        }
        delay_ms(1);
    }
    return -1;
}
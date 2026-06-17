# Rosmaster C++ Driver — MecaMate

Driver C++17 pour le contrôleur Yahboom Rosmaster (X3/X3+/X1/R2), avec boucle
PID logicielle de vitesse par moteur, calibration par ratio inter-moteur, et
interface publique prête pour ros2_control.

---

## Table des matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture des threads](#2-architecture-des-threads)
3. [Protocole UART Yahboom](#3-protocole-uart-yahboom)
4. [Boucle PID logicielle](#4-boucle-pid-logicielle)
5. [Calibration](#5-calibration)
6. [API publique — référence](#6-api-publique--référence)
7. [Démarrage pas à pas](#7-démarrage-pas-à-pas)
8. [Intégration ros2_control](#8-intégration-ros2_control)
9. [Réglage des gains PID](#9-réglage-des-gains-pid)
10. [Diagnostics et dépannage](#10-diagnostics-et-dépannage)
11. [Paramètres à ajuster](#11-paramètres-à-ajuster)

---

## 1. Vue d'ensemble

`Rosmaster.hpp` est un header-only C++17 qui expose une unique classe `Rosmaster`.
Elle gère :

- La communication série POSIX avec le microcontrôleur Yahboom (CH340/CP210x).
- Le parsing des trames d'auto-report (encodeurs, IMU, batterie, vitesse odométrique).
- Une boucle PID logicielle tournant dans un thread dédié, avec fenêtre glissante
  anti-aliasing sur les encodeurs.
- Une procédure de calibration des échelles de vitesse par moteur.
- Un getter thread-safe `get_pid_measured()` pour exposer la mesure de vitesse
  aux couches supérieures (hardware interface ROS2).

`Rosmaster.cpp` contient uniquement le `main()` du banc de test. En intégration
ROS2, ce fichier est remplacé par le hardware interface.

### Dépendances

- C++17 (`std::optional`, structured bindings, `if constexpr` non utilisés, mais
  l'ensemble du projet cible C++17).
- POSIX (Linux). Stub Windows présent mais non testé.
- Aucune dépendance tierce. Pas de Boost, pas de libserial.

---

## 2. Architecture des threads

```
┌─────────────────────────────────────────────────────────────┐
│  Thread principal (ROS2 hardware interface ou main())        │
│                                                             │
│  bot.set_motor(s1, s2, s3, s4)  ──writes──▶  target_[i]   │
│  bot.get_pid_measured()          ──reads──▶  pid_measured_ │
│  bot.get_motor_encoder(...)      ──reads──▶  encoder_mX_   │
└──────────────────────────┬──────────────────────────────────┘
                           │  atomics (lock-free)
          ┌────────────────┴────────────────┐
          │                                 │
┌─────────▼──────────┐           ┌──────────▼──────────┐
│  receiveLoop()      │           │  pidLoop()           │
│  (recv_thread_)     │           │  (pid_thread_)       │
│                     │           │                      │
│  Lit UART → parse  │           │  Lit encoder_mX_     │
│  → encoder_mX_     │──atomic──▶│  → calcule measured  │
│  → battery, IMU... │           │  → PID → writeMotor  │
│                     │           │  → pid_measured_[i]  │
└─────────────────────┘           └──────────────────────┘
```

**Règles de concurrence :**

| Donnée | Producteur | Consommateur | Protection |
|---|---|---|---|
| `encoder_mX_` | `receiveLoop` | `pidLoop`, main | `std::atomic<int>` |
| `target_[i]` | main | `pidLoop` | `std::atomic<double>` |
| `pid_measured_[i]` | `pidLoop` | main | `std::atomic<double>` |
| `pid_gains_` | main (`set_pid_gains`) | `pidLoop` | `std::mutex pid_gains_mutex_` |
| `motor_state_[i]` | `pidLoop` | `pidLoop` | Exclusif au thread PID, pas de mutex |
| `enc_history_` | `pidLoop` | `pidLoop` | Exclusif au thread PID, pas de mutex |

**Ordre de démarrage garanti :**

```
Rosmaster()            → ouvre le port série
create_receive_threading() → démarre recv_thread_
set_auto_report_state(true) → active l'auto-report Yahboom
[attendre encoder_received_ == true]
calibrate_pid_scale_at()  → mesure les échelles (PID arrêté)
enable_pid_control()      → démarre pid_thread_
```

**Ordre d'arrêt garanti (destructeur) :**

```
disable_pid_control()  → pid_running_=false → join(pid_thread_)
uart_running_=false    → join(recv_thread_)
ser_.close()           → libère le fd
```

Le thread PID est joint **avant** le thread réception car `pidLoop()` lit les
atomics encodeur peuplés par `receiveLoop()`. Inverser l'ordre laisserait le
thread PID lire des valeurs périmées ou un fd fermé.

---

## 3. Protocole UART Yahboom

### Format de trame (auto-report entrant)

```
Byte 0 : 0xFF          (HEAD)
Byte 1 : 0xFB          (DEVICE_ID - 1 = 0xFC - 1)
Byte 2 : ext_len       (longueur à partir de ext_type, checksum inclus)
Byte 3 : ext_type      (fonction : encodeur=0x0D, IMU=0x0C, batterie dans 0x0A, ...)
Byte 4..N-2 : données
Byte N-1 : checksum    (= (ext_len + ext_type + somme(data[0..N-3])) % 256)
```

### Format de trame (commande sortante)

```
Byte 0 : 0xFF          (HEAD)
Byte 1 : 0xFC          (DEVICE_ID)
Byte 2 : longueur      (= taille totale - 1, checksum exclu)
Byte 3 : fonction
Byte 4..N-2 : paramètres
Byte N-1 : checksum    (= (COMPLEMENT + somme(bytes[0..N-2])) % 256)
             avec COMPLEMENT = 257 - DEVICE_ID = 5
```

### Trame encodeur (0x0D)

16 octets de données : quatre `int32_t` little-endian, un par moteur (FL, FR, RL, RR).
Les compteurs sont **cumulatifs** et **ne se remettent jamais à zéro** à chaud.
Le driver exploite l'arithmétique modulo-2³² pour gérer les wrap-arounds :

```cpp
const int32_t delta = static_cast<int32_t>(
    static_cast<uint32_t>(enc_now) - static_cast<uint32_t>(enc_old));
```

Cela est correct tant que le vrai déplacement entre deux lectures ne dépasse
pas 2³¹ ticks (impossible en pratique à ~10 000 ticks/s).

### Fréquence d'auto-report

Mesurée sur le Pi 4B avec le Yahboom X3 : **24.4 Hz** (intervalle médian 41 ms).
Cette fréquence est fixée par le firmware Yahboom et n'est pas configurable.

---

## 4. Boucle PID logicielle

### Pourquoi un PID logiciel ?

Le contrôleur Yahboom embarque son propre PID hardware. Cependant :

- Les gains hardware sont exposés via `set_pid_param()` mais leur effet réel
  sur le firmware est opaque.
- Le PID logiciel permet un contrôle total des gains, de la fréquence, de la
  fenêtre de mesure, et de l'anti-windup.
- Il est directement intégrable dans le hardware interface ros2_control sans
  dépendance au firmware.

### Loi de commande

```
velocity[i] = (enc_now[i] - enc_oldest[i]) / window_dt     [ticks/s]
measured[i] = clamp(velocity[i] / scale[i] * 100, -200, 200)  [%]

error[i]    = target[i] - measured[i]

d_filt[i]   = α * d_filt_prev[i] + (1-α) * (error[i] - error_prev[i]) / window_dt

raw_cmd[i]  = target[i]                   ← feedforward
            + kp * error[i]               ← proportionnel
            + ki * integral[i]            ← intégral
            + kd * d_filt[i]              ← dérivé filtré

if |raw_cmd[i]| < 100 :                   ← anti-windup pré-intégration
    integral[i] += error[i] * dt
    integral[i]  = clamp(integral[i], -80, 80)

cmd[i] = clamp(raw_cmd[i], -100, 100)
```

**Feedforward `target` dans `raw_cmd` :** la consigne est ajoutée directement à
la sortie. Cela assure qu'à erreur nulle, la commande correspond exactement à la
consigne en régime permanent, sans dépendre de l'intégrale pour compenser le terme
constant. L'intégrale ne compense que le biais résiduel (frottements, déséquilibre
inter-moteurs).

### Fenêtre glissante anti-aliasing

Les encodeurs arrivent à ~24.4 Hz. Le PID tourne à 25 Hz. Sans filtrage, la mesure
de vitesse sur un seul intervalle `dt` oscille fortement selon que l'itération
PID capture 0 ou 1 paquet encodeur (rapport 24.4/25 ≈ 0.976, non entier → battement).

La fenêtre glissante accumule les `kVelWindow=30` dernières valeurs d'encodeur
dans un ring buffer circulaire. La vitesse est calculée sur le delta entre la
valeur courante et la valeur la plus ancienne du buffer, couvrant environ
`30 × 40 ms = 1200 ms`. Cela est équivalent à une moyenne glissante sur ~29
périodes de paquet, ce qui supprime totalement le battement.

```
enc_history_ [ring buffer, taille kVelWindow]

oldest_idx = (enc_history_idx_ + 1) % kVelWindow   ← correct :
    après ecriture à enc_history_idx_ et incrément,
    enc_history_idx_ pointe sur la case écrite
    kVelWindow-1 itérations avant = la plus ancienne.

delta[i] = enc_now[i] - enc_history_[oldest_idx][i]
window_dt = n_samples * dt
```

**Note sur `oldest_idx` :** la formule `(idx+1)%N` (et non `idx`) est correcte
parce que l'écriture se fait à `idx` *avant* l'incrément. Après incrément,
`idx` désigne la case écrite au dernier tour le plus ancien, pas la case courante.
Une tentative de correction à `idx` (sans `+1`) a introduit une régression :
`delta` devenait nul au premier wrap du buffer, effondrant `measured` à 0.

### Anti-windup

L'intégrale est mise à jour **uniquement si `|raw_cmd| < 100`**, c'est-à-dire
uniquement si la sortie n'est pas saturée. C'est l'anti-windup pré-intégration
(standard industriel). Il n'y a pas de rollback post-calcul, ce qui évite toute
incohérence entre l'état intégrateur et la commande réellement envoyée.

### Filtre dérivé EMA

```
d_filt = α * d_filt_prev + (1-α) * raw_deriv
```

Avec `kDerivAlpha = 0.8` et `dt = 40 ms`, la constante de temps équivalente est :

```
τ = α / (1-α) * dt = 0.8 / 0.2 * 0.04 = 160 ms
```

Le terme dérivé utilise `window_dt` (≈1200 ms) et non `dt` (40 ms) comme
diviseur. Diviser par `dt` produirait un pic de dérivée ~30× trop grand à chaque
nouveau paquet encodeur.

---

## 5. Calibration

### Pourquoi calibrer ?

La conversion `%commande → ticks/s` dépend du variant de motoréducteur
(NFP-JGB37-520 existe en plusieurs réductions). Sans calibration, le feedforward
est faux et l'intégrale doit compenser seule.

### `calibrate_pid_scale_at()` — méthode recommandée

```cpp
double scale_global = bot.calibrate_pid_scale_at(
    /*throttle_pct=*/ 60,
    /*duration_ms=*/  800,
    /*use_per_motor=*/ true
);
```

**Principe :**

1. N runs à `throttle_pct%`, durée `duration_ms` chacun.
2. Pour chaque run, mesure des ticks par moteur (delta encodeur stable avant/après).
3. Calcule le ratio de chaque moteur par rapport à la moyenne du run (ex. M1=1.10, M2=0.92...).
4. Scale globale = moyenne tronquée (rejet min+max) sur les N runs.
5. Scale par moteur = scale_globale × ratio_moyen[i].

Les ratios inter-moteurs sont très stables run-to-run (CV < 1%) même si la
valeur absolue dérive thermiquement de ±5% entre runs. La moyenne tronquée
élimine les runs aberrants.

Si `use_per_motor=true`, `motor_scale_[i]` est peuplé et utilisé par `pidLoop()`.
Sinon, seule la scale globale est utilisée pour tous les moteurs.

**Préconditions :**

- PID désactivé (`pid_enabled_ == false`).
- Roues libres de tourner (robot levé ou zone dégagée).
- `set_auto_report_state(true)` déjà appelé.

### `set_motor_scales()` — injection directe

Si la calibration est faite en dehors du driver (ex. fichier de configuration) :

```cpp
std::array<double,4> scales = {10634.9, 8892.1, 9160.6, 9995.7};
double global = 9677.95;
bot.set_motor_scales(scales, global);
```

Cela peuple `motor_scale_[]`, `ticks_per_second_at_100pct_`, et lève le flag
`pid_scale_calibrated_` (supprime le warning à `enable_pid_control`).

---

## 6. API publique — référence

### Construction et cycle de vie

```cpp
// car_type : 1=X3, 2=X3_PLUS, 4=X1, 5=R2
Rosmaster bot(1, "/dev/ttyUSB0", /*cmd_delay_s=*/0.002, /*debug=*/false);

bot.create_receive_threading();       // démarre le thread de réception
bot.set_auto_report_state(true, true); // active l'auto-report (persistent=true)

// Attendre le premier paquet
while (bot.get_battery_voltage() == 0.0)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
```

### Contrôle moteur (sans PID)

```cpp
// Commande directe en % [-100, 100], envoyée immédiatement au hardware
bot.set_motor(40.0, 40.0, 40.0, 40.0);   // tous les moteurs à 40%
bot.set_motor(0.0, 0.0, 0.0, 0.0);       // arrêt
```

### Contrôle moteur (avec PID)

```cpp
// Après calibration et enable_pid_control(), set_motor() dépose des setpoints
// dans target_[]. Le thread PID les lit à kPidHz et envoie les commandes.
bot.set_motor(40.0, 40.0, 40.0, 40.0);   // consigne 40% sur les 4 moteurs
```

La distinction est automatique : `set_motor()` teste `pid_enabled_` et bascule
de comportement. **Il n'y a pas de deuxième fonction à appeler.**

### PID — lifecycle

```cpp
// Activation (après calibration)
bot.enable_pid_control(
    /*kp=*/0.6, /*ki=*/0.1, /*kd=*/0.0,
    /*ticks_per_sec=*/scale_global
);

// Modification des gains à chaud (thread-safe)
bot.set_pid_gains(0.6, 0.2, 0.0);

// Arrêt
bot.set_motor(0.0, 0.0, 0.0, 0.0);          // consigne zéro d'abord
std::this_thread::sleep_for(std::chrono::milliseconds(80)); // 2 cycles PID
bot.disable_pid_control();                    // join le thread PID
```

### Lecture des mesures

```cpp
// Encodeurs bruts (cumulatifs)
int m1, m2, m3, m4;
bot.get_motor_encoder(m1, m2, m3, m4);

// Vitesse mesurée par le PID (%) — lock-free, valide seulement si PID actif
std::array<double,4> vel = bot.get_pid_measured();
// vel[0]=M1%, vel[1]=M2%, vel[2]=M3%, vel[3]=M4%
// Retourne {0,0,0,0} si PID désactivé

// Batterie
double v = bot.get_battery_voltage();   // en Volts

// IMU attitude
double roll, pitch, yaw;
bot.get_imu_attitude_data(roll, pitch, yaw, /*to_angle=*/true);  // degrés

// Santé du port série
bool ok = bot.is_running();
```

### Calibration — séquence complète

```cpp
// 1. Warmup thermique recommandé (évite les runs à froid faussés)
bot.writeMotorRaw_public({60.0, 60.0, 60.0, 60.0});
std::this_thread::sleep_for(std::chrono::seconds(3));
bot.writeMotorRaw_public({0.0, 0.0, 0.0, 0.0});
std::this_thread::sleep_for(std::chrono::milliseconds(500));

// 2. Calibration par ratio inter-moteur
double scale_global = bot.calibrate_pid_scale_at(60, 800, true);

// 3. Activation PID
bot.enable_pid_control(0.6, 0.1, 0.0, scale_global);
```

---

## 7. Démarrage pas à pas

```cpp
#include "Rosmaster.hpp"
#include <thread>
#include <chrono>

int main() {
    // ── 1. Ouverture port ────────────────────────────────────────────────
    Rosmaster bot(1, "/dev/ttyUSB0");
    bot.create_receive_threading();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ── 2. Auto-report ────────────────────────────────────────────────────
    bot.set_auto_report_state(true, true);

    // ── 3. Attente premier paquet ─────────────────────────────────────────
    while (bot.get_battery_voltage() == 0.0)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // ── 4. Calibration ────────────────────────────────────────────────────
    // Warmup
    bot.writeMotorRaw_public({60.0, 60.0, 60.0, 60.0});
    std::this_thread::sleep_for(std::chrono::seconds(3));
    bot.writeMotorRaw_public({0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    double scale = bot.calibrate_pid_scale_at(60, 800, true);

    // ── 5. PID ────────────────────────────────────────────────────────────
    bot.enable_pid_control(0.6, 0.1, 0.0, scale);

    // ── 6. Commande ───────────────────────────────────────────────────────
    bot.set_motor(40.0, 40.0, 40.0, 40.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // ── 7. Arrêt propre ───────────────────────────────────────────────────
    bot.set_motor(0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bot.disable_pid_control();
    // Le destructeur ferme le port et joint les threads
    return 0;
}
```

Compilation :

```bash
g++ -std=c++17 -O2 -o mecamate main.cpp
```

---

## 8. Intégration ros2_control

### Vue d'ensemble

Dans l'architecture ros2_control, le hardware interface implémente
`hardware_interface::SystemInterface` et fait le pont entre ROS2 et le driver.

```
ros2_control_node
    └─ MecaMateSystemHardware  (SystemInterface)
           └─ Rosmaster bot_
                  ├─ recv_thread_    (encode → atomics)
                  └─ pid_thread_     (mesure → commande)
```

### State interfaces exposées

```
joint_0/velocity   ← bot_.get_pid_measured()[0] / 100.0 * max_vel_rad_s
joint_1/velocity   ← bot_.get_pid_measured()[1] / 100.0 * max_vel_rad_s
joint_2/velocity   ← bot_.get_pid_measured()[2] / 100.0 * max_vel_rad_s
joint_3/velocity   ← bot_.get_pid_measured()[3] / 100.0 * max_vel_rad_s
```

### Command interfaces consommées

```
joint_0/velocity   → commande_pct[0] = cmd_rad_s / max_vel_rad_s * 100.0
joint_1/velocity   → commande_pct[1] = ...
joint_2/velocity   → commande_pct[2] = ...
joint_3/velocity   → commande_pct[3] = ...
```

### Squelette d'implémentation

```cpp
// MecaMateSystemHardware.hpp
#pragma once
#include "hardware_interface/system_interface.hpp"
#include "Rosmaster.hpp"
#include <memory>

class MecaMateSystemHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    std::unique_ptr<Rosmaster> bot_;

    // Vitesses mesurées exposées via state interfaces [rad/s]
    std::array<double, 4> vel_state_{};

    // Commandes reçues depuis les command interfaces [rad/s]
    std::array<double, 4> vel_cmd_{};

    // Paramètres de conversion
    double max_vel_rad_s_{10.0};   // vitesse à 100% en rad/s — à calibrer
    double scale_global_{9677.0};  // ticks/s à 100% — issu de la calibration
};
```

```cpp
// MecaMateSystemHardware.cpp
#include "MecaMateSystemHardware.hpp"

hardware_interface::CallbackReturn
MecaMateSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // Lire les paramètres depuis le URDF / ros2_control xacro
    max_vel_rad_s_ = std::stod(info_.hardware_parameters.at("max_velocity_rad_s"));
    scale_global_  = std::stod(info_.hardware_parameters.at("ticks_per_sec_at_100pct"));

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MecaMateSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
    const std::string port = info_.hardware_parameters.at("serial_port");

    bot_ = std::make_unique<Rosmaster>(1, port, 0.002, false);
    bot_->create_receive_threading();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bot_->set_auto_report_state(true, true);

    // Attendre premier paquet encodeur
    auto t0 = std::chrono::steady_clock::now();
    while (bot_->get_battery_voltage() == 0.0) {
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(3)) {
            RCLCPP_ERROR(rclcpp::get_logger("MecaMate"),
                         "Timeout: aucun paquet Yahboom reçu");
            return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Injection des échelles pré-calibrées
    // (ou appel à calibrate_pid_scale_at() si les roues sont libres)
    std::array<double,4> per_motor = {
        std::stod(info_.hardware_parameters.at("scale_m1")),
        std::stod(info_.hardware_parameters.at("scale_m2")),
        std::stod(info_.hardware_parameters.at("scale_m3")),
        std::stod(info_.hardware_parameters.at("scale_m4"))
    };
    bot_->set_motor_scales(per_motor, scale_global_);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MecaMateSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
    bot_->enable_pid_control(0.6, 0.1, 0.0, scale_global_);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MecaMateSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    bot_->set_motor(0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bot_->disable_pid_control();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecaMateSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < 4; ++i)
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &vel_state_[i]);
    return si;
}

std::vector<hardware_interface::CommandInterface>
MecaMateSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < 4; ++i)
        ci.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &vel_cmd_[i]);
    return ci;
}

hardware_interface::return_type
MecaMateSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // Convertir pid_measured_ (%) → rad/s
    const auto meas = bot_->get_pid_measured();
    for (size_t i = 0; i < 4; ++i)
        vel_state_[i] = meas[i] / 100.0 * max_vel_rad_s_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MecaMateSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // Convertir rad/s → % et déposer dans les setpoints PID
    bot_->set_motor(
        vel_cmd_[0] / max_vel_rad_s_ * 100.0,
        vel_cmd_[1] / max_vel_rad_s_ * 100.0,
        vel_cmd_[2] / max_vel_rad_s_ * 100.0,
        vel_cmd_[3] / max_vel_rad_s_ * 100.0
    );
    return hardware_interface::return_type::OK;
}
```

### Description URDF (ros2_control xacro)

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
  <joint name="front_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="rear_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="rear_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Note sur `reconnect_rosmaster_if_needed()`

Le hardware interface peut détecter une déconnexion USB via :

```cpp
if (!bot_->is_running()) {
    // Le port a reçu EIO (Yahboom coupé électriquement)
    // Recréer bot_, refaire on_configure(), on_activate()
}
```

`is_running()` retourne `uart_running_.load(relaxed)`. `receiveLoop()` met
ce flag à `false` et ferme le fd proprement sur tout EIO/ENOTTY/EBADF, ce
qui déclenche la réénumération complète du CH340/CP210x au prochain `open()`.

---

## 9. Réglage des gains PID

### Valeurs actuelles (validées sur le banc)

```
kp = 0.6
ki = 0.1
kd = 0.0
```

Résultat observé : erreur statique < 2% sur M1, biais résiduel 2-3% sur M2/M3/M4
(compensable en augmentant `ki` ou en attendant plus longtemps le régime établi).

### Procédure de réglage sur robot au sol

**Étape 1 — Fréquence et fenêtre**

Ne pas changer `kPidHz=25` ni `kVelWindow=30` sans recalculer :

```
kVelWindow ≥ ceil(2 × période_paquets_ms / (1000 / kPidHz))
           = ceil(2 × 41 / 40)
           = ceil(2.05)
           = 3   (minimum théorique)
```

30 est volontairement grand pour lisser le jitter de scheduling. Le descendre
en-dessous de 8 risque de réintroduire de l'aliasing.

**Étape 2 — kp seul (ki=0, kd=0)**

Augmenter `kp` jusqu'à ce que la réponse au saut de consigne soit rapide sans
oscillation visible sur `get_pid_measured()`. Typiquement entre 0.4 et 1.0.

**Étape 3 — ki pour corriger le biais statique**

Avec `kp` fixé, augmenter `ki` lentement. Un `ki` trop élevé produit un dépassement
lent (l'intégrale met plusieurs secondes à saturer l'anti-windup). Valeur typique : 0.05–0.2.

**Étape 4 — kd si nécessaire**

Sur une commande mecanum sur surface lisse, `kd=0` est suffisant. Ajouter `kd`
uniquement si des perturbations impulsionnelles (choc, marche) doivent être
amorties plus vite. Rester en-dessous de 0.05 pour éviter l'amplification du
bruit quantification encodeur.

---

## 10. Diagnostics et dépannage

### `pid%` tombe à 0 après ~1200 ms

Symptôme : les colonnes `pid%` du banc sont non nulles pendant la montée (0–1200 ms)
puis tombent à 0 brutalement.

Cause probable : `oldest_idx` mal calculé dans `pidLoop()` — se produit au premier
wrap complet du ring buffer (exactement `kVelWindow × dt ≈ 1200 ms`). Vérifier que
le code utilise `(enc_history_idx_ + 1) % kVelWindow` et non `enc_history_idx_`.

### Oscillation 330/500 dans le banc de test

Symptôme : les colonnes `raw_M1` alternent fortement (~±25% autour de la cible).

Cause : aliasing entre la fenêtre d'affichage du banc (100 ms) et la période des
paquets encodeur (~41 ms). 100/41 ≈ 2.44 paquets/fenêtre — rapport non entier
→ battement. **Ce n'est pas une oscillation réelle du robot.**

Diagnostic : vérifier les colonnes `pid%` en parallèle. Si `pid%` est stable à
±2%, l'oscillation est un artefact d'affichage. Pour confirmer, passer la fenêtre
du banc à 820 ms (≈20 paquets, rapport quasi entier).

### Port série bloque au deuxième `open()` après coupure Yahboom

Symptôme : le processus gèle au `open()` sans timeout après une coupure électrique
du Yahboom suivie d'une reconnexion USB.

Cause : VHANGUP tty laissé par le fd non fermé de la session précédente bloque le
`open()` en mode bloquant. FIX-1 à FIX-7 dans le header adressent ce problème :
`O_NONBLOCK` sur `open()`, `TIOCEXCL/TIOCNXCL`, `HUPCL` désactivé, `tcflush`
avant `tcgetattr`, et `close()` explicite dans `receiveLoop()` sur EIO.

### `calibrate_pid_scale_at()` retourne des ticks insuffisants

Vérifier :
1. `set_auto_report_state(true)` a été appelé **avant** la calibration.
2. Les encodeurs répondent (`get_motor_encoder()` retourne des valeurs changeantes
   pendant que les moteurs tournent).
3. `duration_ms >= 200` (valeur minimale imposée).
4. Les roues sont libres de tourner.

### `enable_pid_control()` lève une exception

```
"no encoder packet received yet"
```

`set_auto_report_state(true)` n'a pas encore reçu de paquet. Attendre
`encoder_received_ == true` en polllant `get_battery_voltage() > 0` ou
`get_motor_encoder()` avec une valeur changeante, puis réessayer.

---

## 11. Paramètres à ajuster

| Paramètre | Emplacement | Valeur actuelle | Quand modifier |
|---|---|---|---|
| `kPidHz` | `Rosmaster.hpp` constexpr | 25 | Si la fréquence paquets encodeur change |
| `kVelWindow` | `Rosmaster.hpp` constexpr | 30 | Si `kPidHz` change — recalculer |
| `kDerivAlpha` | `Rosmaster.hpp` constexpr | 0.8 | Si `kPidHz` change — recalculer τ_eq |
| `kp, ki, kd` | `enable_pid_control()` appel | 0.6, 0.1, 0.0 | Réglage sur robot au sol |
| `scale_global` | calibration | ~9670 ticks/s | À recalibrer si motoréducteurs changent |
| `motor_scale_[i]` | calibration | per moteur | Idem |
| `max_vel_rad_s_` | URDF param | 10.0 | À mesurer sur le robot (vitesse max réelle) |
| `serial_port` | URDF param / constructor | `/dev/ttyUSB0` | Selon udev rules |

### Recalcul de `kVelWindow` si `kPidHz` change

```
kVelWindow ≥ ceil(2 × T_paquet_ms / (1000 / kPidHz))
```

Pour le Yahboom X3 (T_paquet ≈ 41 ms) :

| kPidHz | dt (ms) | kVelWindow minimum | Recommandé |
|---|---|---|---|
| 100 | 10 | 9 | 15 |
| 50 | 20 | 5 | 10 |
| 25 | 40 | 3 | 30 |
| 10 | 100 | 1 | 5 |

À 25 Hz, kVelWindow=30 couvre 1200 ms ≈ 29 périodes paquet — confortable.

#include "Rosmaster.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>

// =============================================================================
//  Banc de test MecaMate — main()
//
//  La calibration multi-runs est maintenant intégrée dans la classe via
//  Rosmaster::calibrate_motor_scales().  Ce fichier ne contient plus que le
//  séquenceur de test.
// =============================================================================
int main()
{
    {
    try {
        // ── 1. Ouverture port + thread réception ──────────────────────────
        Rosmaster bot(1, "/dev/ttyUSB0", 0.002, false);
        bot.create_receive_threading();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // ── 2. Auto-report ────────────────────────────────────────────────
        bot.set_auto_report_state(true, true);

        // ── 3. Attente robuste des premiers paquets ───────────────────────
        {
            auto t0 = std::chrono::steady_clock::now();
            while (bot.get_battery_voltage() == 0.0) {
                const auto elapsed =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                if (elapsed > 2000)
                    throw std::runtime_error(
                        "Timeout : aucun paquet Yahboom reçu après 2s — "
                        "vérifier alimentation et câble USB");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        std::cout << "Batterie : " << bot.get_battery_voltage() << " V\n";

        // ── 4. Calibration multi-runs avec warmup intégré ─────────────────
        // warmup_ms=3000 : 3 s à 60% pour stabiliser les bobines DC.
        // n_runs=5 : moyenne tronquée sur 3 runs intérieurs (min+max rejetés).
        const double scale_global = bot.calibrate_motor_scales(
            /*throttle_pct=*/ 60,
            /*duration_ms=*/  800,
            /*n_runs=*/        5,
            /*warmup_ms=*/  3000,
            /*use_per_motor=*/ true);

        // ── 5. Activation PID ─────────────────────────────────────────────
        // kd=0.0 confirmé sur banc (oscillation mecanum mécanique, pas logicielle).
        // ki=0.1 : correction statique lente mais stable.
        bot.enable_pid_control(0.6, 0.1, 0.0, scale_global);

        // ── 6. Test de convergence ────────────────────────────────────────
        std::cout << "\nTest convergence PID : avance 50 s @ -20%\n";
        std::cout << "  "
                << std::setw(6) << "t(ms)"
                << std::setw(8) << "raw_M1" << std::setw(8) << "raw_M2"
                << std::setw(8) << "raw_M3" << std::setw(8) << "raw_M4"
                << "  pid%: "
                << std::setw(7) << "M1" << std::setw(7) << "M2"
                << std::setw(7) << "M3" << std::setw(7) << "M4"
                << "\n";

        bot.set_motor(-20.0, -20.0, -20.0, -20.0);

        int prev[4] = {0, 0, 0, 0};
        bot.get_motor_encoder(prev[0], prev[1], prev[2], prev[3]);

        for (int i = 0; i < 500; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            int cur[4];
            bot.get_motor_encoder(cur[0], cur[1], cur[2], cur[3]);
            const auto meas = bot.get_pid_measured();

            std::cout << "  " << std::setw(6) << (i+1)*100;
            for (int j = 0; j < 4; ++j) {
                const int32_t delta = static_cast<int32_t>(
                    static_cast<uint32_t>(cur[j]) - static_cast<uint32_t>(prev[j]));
                std::cout << std::setw(8) << delta;
                prev[j] = cur[j];
            }
            std::cout << std::fixed << std::setprecision(1) << "  pid%:";
            for (int j = 0; j < 4; ++j)
                std::cout << std::setw(7) << meas[j];
            std::cout << "\n";
        }

        // ── 7. Arrêt propre ───────────────────────────────────────────────
        bot.set_motor(0.0, 0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));  // 2 cycles PID
        bot.disable_pid_control();

    } catch (const std::exception & e) {
        std::cerr << "Erreur fatale : " << e.what() << "\n";
        return 1;
    }
    }
    return 0;
}
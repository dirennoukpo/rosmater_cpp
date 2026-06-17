#include "Rosmaster.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────────
//  Calibration par ratio inter-moteur
//
//  Principe :
//    1. N runs à THROTTLE% → ticks mesurés par moteur
//    2. Ratio M[i] / moyenne_run  →  très stable run-to-run (CV < 0.5%)
//    3. Scale globale = moyenne tronquée (rejet min+max) des runs
//    4. Scale_i = scale_globale × ratio_moyen[i]
//
//  Avantage : la dérive thermique absolue affecte tous les moteurs
//  de façon identique → les ratios restent stables même si la valeur
//  absolue dérive de ±5% entre runs.
// ─────────────────────────────────────────────────────────────────────────────

static std::array<double, 4> calibrate(Rosmaster & bot,
                                        int throttle_pct,
                                        int duration_ms,
                                        int n_runs,
                                        double & scale_global_out)
{
    constexpr int STABLE_POLL_MS = 25;
    constexpr int STABLE_RETRIES = 30;

    // Snapshot stable : deux lectures identiques à STABLE_POLL_MS d'écart
    // → garantit qu'on ne lit pas un paquet encodeur en cours de MAJ (~20 ms)
    auto stable_read = [&](int m[4]) {
        int a[4], b[4];
        for (int i = 0; i < STABLE_RETRIES; ++i) {
            bot.get_motor_encoder(a[0], a[1], a[2], a[3]);
            std::this_thread::sleep_for(std::chrono::milliseconds(STABLE_POLL_MS));
            bot.get_motor_encoder(b[0], b[1], b[2], b[3]);
            if (a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]) {
                for (int j = 0; j < 4; ++j) m[j] = b[j];
                return;
            }
        }
        // Timeout : valeur la plus récente disponible
        for (int j = 0; j < 4; ++j) m[j] = b[j];
    };

    const double t   = static_cast<double>(throttle_pct);
    const double dt  = duration_ms / 1000.0;

    std::array<double, 4> ratio_sum  = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> ratio_sum2 = {0.0, 0.0, 0.0, 0.0};
    std::vector<double>   global_samples;
    int valid_runs = 0;

    std::cout << "\nCalibration @ " << throttle_pct << "% — "
              << n_runs << " runs de " << duration_ms << " ms\n";

    for (int run = 0; run < n_runs; ++run) {
        if (run > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int ma[4], mb[4];
        stable_read(ma);

        bot.writeMotorRaw_public({t, t, t, t});
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        bot.writeMotorRaw_public({0.0, 0.0, 0.0, 0.0});

        // Attente arrêt mécanique complet avant snapshot final
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        stable_read(mb);

        // Ticks par moteur (arithmétique uint32 modulo-2³²)
        std::array<double, 4> ticks{};
        bool ok = true;
        for (int i = 0; i < 4; ++i) {
            const int32_t d = static_cast<int32_t>(
                static_cast<uint32_t>(mb[i]) - static_cast<uint32_t>(ma[i]));
            ticks[i] = std::abs(static_cast<double>(d));
            if (ticks[i] < 10.0) { ok = false; }
        }

        if (!ok) {
            std::cerr << "  [WARN] run " << run
                      << " : ticks insuffisants — ignoré\n";
            continue;
        }

        // Ratio par rapport à la moyenne du run
        // (plus robuste que M1 seul : si M1 a une anomalie, ça ne fausse pas tout)
        const double run_mean = (ticks[0]+ticks[1]+ticks[2]+ticks[3]) / 4.0;
        for (int i = 0; i < 4; ++i) {
            const double r   = ticks[i] / run_mean;
            ratio_sum[i]    += r;
            ratio_sum2[i]   += r * r;
        }

        const double global = (run_mean / dt) * (100.0 / throttle_pct);
        global_samples.push_back(global);
        ++valid_runs;

        std::cout << "  Run " << run << " :";
        for (int i = 0; i < 4; ++i)
            std::cout << "  M" << (i+1) << "=" << static_cast<int>(ticks[i]);
        std::cout << "  global=" << static_cast<int>(global) << " ticks/s\n";
    }

    if (valid_runs < 3)
        throw std::runtime_error(
            "calibrate(): pas assez de runs valides (< 3) — "
            "vérifier câblage encodeurs et set_auto_report_state");

    // ── Scale globale : moyenne tronquée (rejet min + max) ────────────────
    std::sort(global_samples.begin(), global_samples.end());
    double global_total = 0.0;
    for (size_t i = 1; i < global_samples.size() - 1; ++i)
        global_total += global_samples[i];
    scale_global_out = global_total / static_cast<double>(global_samples.size() - 2);

    // ── Ratios moyens + CV ────────────────────────────────────────────────
    std::cout << "\n=== Ratios inter-moteurs ===\n";
    std::array<double, 4> ratio_mean{};
    for (int i = 0; i < 4; ++i) {
        ratio_mean[i]     = ratio_sum[i] / valid_runs;
        const double var  = ratio_sum2[i] / valid_runs
                          - ratio_mean[i] * ratio_mean[i];
        const double cv   = std::sqrt(std::max(var, 0.0))
                          / ratio_mean[i] * 100.0;
        std::cout << "  M" << (i+1)
                  << "  ratio=" << ratio_mean[i]
                  << "  CV="    << cv << "%";
        if (cv > 1.5)
            std::cout << "  [WARN: CV élevé — moteur irrégulier ?]";
        std::cout << "\n";
    }

    // ── Échelles finales par moteur ───────────────────────────────────────
    std::array<double, 4> final_scales{};
    std::cout << "\n=== Échelles finales ===\n";
    for (int i = 0; i < 4; ++i) {
        final_scales[i] = scale_global_out * ratio_mean[i];
        std::cout << "  M" << (i+1)
                  << " = " << final_scales[i] << " ticks/s"
                  << "  (ratio=" << ratio_mean[i] << ")\n";
    }
    std::cout << "  Global = " << scale_global_out << " ticks/s\n";

    return final_scales;
}

// =============================================================================
int main()
{
    try {
        // ── 1. Ouverture port + thread réception ──────────────────────────
        Rosmaster bot(1, "/dev/ttyUSB0", 0.002, false);
        bot.create_receive_threading();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // ── 2. Auto-report ────────────────────────────────────────────────
        bot.set_auto_report_state(true, true);

        // ── 3. Attente robuste des premiers paquets ───────────────────────
        // Utilise get_battery_voltage() comme proxy : > 0 ↔ paquets reçus
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

        // ── 4. Warmup thermique ───────────────────────────────────────────
        // 3 s à 60% stabilise les bobines brushed DC.
        // Sans warmup, le premier run de calibration est biaisé vers le bas
        // (inertie froide) et fausse le ratio M1 (le plus affecté).
        std::cout << "Warmup thermique (3 s @ 60%)...\n";
        bot.writeMotorRaw_public({60.0, 60.0, 60.0, 60.0});
        std::this_thread::sleep_for(std::chrono::seconds(3));
        bot.writeMotorRaw_public({0.0, 0.0, 0.0, 0.0});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ── 5. Calibration par ratio inter-moteur ─────────────────────────
        double scale_global = 1326.0;
        const auto final_scales = calibrate(bot,
                                             /*throttle_pct=*/ 60,
                                             /*duration_ms=*/  800,
                                             /*n_runs=*/        5,
                                             scale_global);

        // Injecter les échelles dans le PID
        bot.set_motor_scales(final_scales, scale_global);
        // ── Mesure fréquence réelle des paquets encodeurs ─────────────────────────
        std::cout << "\nMesure fréquence paquets encodeurs (3s)...\n";
        {
            int prev[4], cur[4];
            bot.get_motor_encoder(prev[0], prev[1], prev[2], prev[3]);

            int  n_changes = 0;
            auto t_start   = std::chrono::steady_clock::now();
            auto t_last    = t_start;

            // Robot immobile — on compte juste les changements d'encodeur
            // (même immobile les paquets arrivent avec la même valeur → on
            //  détecte les fronts de mise à jour via un flag dédié)

            // Méthode : poll à 2000 Hz, compter les transitions
            std::vector<double> intervals_ms;
            auto t_prev_change = t_start;
            bool first = true;

            for (int i = 0; i < 6000; ++i) {   // 6000 × 0.5 ms = 3 s
                std::this_thread::sleep_for(std::chrono::microseconds(500));
                bot.get_motor_encoder(cur[0], cur[1], cur[2], cur[3]);

                bool changed = false;
                for (int j = 0; j < 4; ++j)
                    if (cur[j] != prev[j]) { changed = true; break; }

                if (changed) {
                    auto now = std::chrono::steady_clock::now();
                    if (!first) {
                        const double ms = std::chrono::duration<double,std::milli>(
                            now - t_prev_change).count();
                        intervals_ms.push_back(ms);
                    }
                    t_prev_change = now;
                    first = false;
                    ++n_changes;
                    for (int j = 0; j < 4; ++j) prev[j] = cur[j];
                }
            }

            if (intervals_ms.size() < 5) {
                std::cout << "  [WARN] Pas assez de changements détectés\n";
            } else {
                double sum = 0.0;
                for (double v : intervals_ms) sum += v;
                const double mean_ms = sum / intervals_ms.size();

                // Médiane
                std::sort(intervals_ms.begin(), intervals_ms.end());
                const double median_ms = intervals_ms[intervals_ms.size()/2];

                std::cout << "  Paquets détectés : " << n_changes << " en 3s\n"
                        << "  Intervalle moyen : " << mean_ms   << " ms\n"
                        << "  Intervalle médian: " << median_ms << " ms\n"
                        << "  Fréquence réelle : " << 1000.0/median_ms << " Hz\n";
            }
        }
        // ── Test stabilité encodeurs immobile (2s) ───────────────────────────────
        std::cout << "\nTest stabilité encodeurs (robot immobile, 2s)...\n";
        {
            int drops[4] = {0,0,0,0};   // nombre de sauts anormaux
            int prev[4];
            bot.get_motor_encoder(prev[0], prev[1], prev[2], prev[3]);

            for (int i = 0; i < 200; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                int cur[4];
                bot.get_motor_encoder(cur[0], cur[1], cur[2], cur[3]);
                for (int j = 0; j < 4; ++j) {
                    const int32_t d = static_cast<int32_t>(
                        static_cast<uint32_t>(cur[j]) - static_cast<uint32_t>(prev[j]));
                    // Immobile → delta doit être 0. Toute variation = paquet corrompu.
                    if (std::abs(d) > 5) {
                        ++drops[j];
                        std::cout << "  [WARN] M" << (j+1) << " saut immobile : "
                                << d << " ticks @ " << i*10 << " ms\n";
                    }
                    prev[j] = cur[j];
                }
            }
            for (int j = 0; j < 4; ++j)
                std::cout << "  M" << (j+1) << " : " << drops[j]
                        << " anomalie(s) sur 200 polls\n";
        }
        // ── 6. Activation PID ─────────────────────────────────────────────
        // Gains originaux prévus pour 100 Hz sur Linux bare-metal — bon
        // point de départ sur Pi. À affiner après le premier test de
        // convergence (voir ordre de priorité : kp par pas de 0.2 si
        // oscillation, kd à la baisse si dépassement, ki par pas de 0.05
        // si erreur statique résiduelle).
        bot.enable_pid_control(1.8, 0.4, 0.05, scale_global);

        // ── 7. Test de mouvement ──────────────────────────────────────────
        // Remplacer le bloc "Test mouvement" dans main() par :

        std::cout << "\nTest convergence PID : avance 5 s @ 40%\n";
        std::cout << "  " 
                << std::setw(6) << "t(ms)"
                << std::setw(8) << "M1"
                << std::setw(8) << "M2"
                << std::setw(8) << "M3"
                << std::setw(8) << "M4"
                << "  (ticks/100ms)\n";

        bot.set_motor(40.0, 40.0, 40.0, 40.0);

        int prev[4] = {0, 0, 0, 0};
        bot.get_motor_encoder(prev[0], prev[1], prev[2], prev[3]);

        for (int i = 0; i < 50; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            int cur[4];
            bot.get_motor_encoder(cur[0], cur[1], cur[2], cur[3]);

            // Delta ticks sur 100 ms = vitesse instantanée
            const int dt_ms = 100;
            std::cout << "  " << std::setw(6) << (i+1)*dt_ms;
            for (int j = 0; j < 4; ++j) {
                const int32_t delta = static_cast<int32_t>(
                    static_cast<uint32_t>(cur[j]) - static_cast<uint32_t>(prev[j]));
                std::cout << std::setw(8) << delta;
                prev[j] = cur[j];
            }

            // Cible théorique : scale_i × 0.40 / 10  (ticks/100ms)
            std::cout << "  cible:"
                    << std::setw(5) << static_cast<int>(final_scales[0]*0.40/10)
                    << std::setw(5) << static_cast<int>(final_scales[1]*0.40/10)
                    << std::setw(5) << static_cast<int>(final_scales[2]*0.40/10)
                    << std::setw(5) << static_cast<int>(final_scales[3]*0.40/10)
                    << "\n";
        }
        // ── 8. Arrêt propre ───────────────────────────────────────────────
        bot.set_motor(0.0, 0.0, 0.0, 0.0);
        // Laisser le PID envoyer la commande zéro (~2 cycles @ 100 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        bot.disable_pid_control();

    } catch (const std::exception & e) {
        std::cerr << "Erreur fatale : " << e.what() << "\n";
        return 1;
    }
    return 0;
}
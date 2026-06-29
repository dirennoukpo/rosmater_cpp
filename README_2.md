# MecaMate — Rosmaster.hpp : contrôle de vitesse des roues

Documentation du driver C++ `Rosmaster.hpp` pour la plateforme Yahboom Rosmaster
(mecanum 4 roues), incluant l'historique complet de la boucle PID logicielle
(v2 → v8) et le fonctionnement du banc de test associé (`main.cpp`).

---

## Table des matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture du contrôle de vitesse](#2-architecture-du-contrôle-de-vitesse)
3. [Historique du PID logiciel (v2 → v8)](#3-historique-du-pid-logiciel-v2--v8)
4. [Le modèle moteur et ses limites physiques](#4-le-modèle-moteur-et-ses-limites-physiques)
5. [Le feedforward kS/kV (v8)](#5-le-feedforward-kskv-v8)
6. [Cycle de vie du port série](#6-cycle-de-vie-du-port-série)
7. [Procédure de calibration sur banc](#7-procédure-de-calibration-sur-banc)
8. [Utilisation du banc de test (`main.cpp`)](#8-utilisation-du-banc-de-test-maincpp)
9. [Réglage des gains PID](#9-réglage-des-gains-pid)
10. [Référence API](#10-référence-api)
11. [Limites connues et pistes futures](#11-limites-connues-et-pistes-futures)

---

## 1. Vue d'ensemble

`Rosmaster.hpp` est un portage C++17 (header-only) du driver Python Rosmaster
V3.3.9 de Yahboom, étendu avec :

- une couche **PID logiciel** par moteur (le firmware Yahboom ne gère que la
  commande PWM brute, pas de boucle vitesse fermée embarquée exploitable
  finement depuis ROS 2) ;
- un **feedforward moteur** (`kS + kV`) compensant la friction statique et la
  pente PWM→vitesse sous charge ;
- un cycle de vie de port série POSIX **robuste aux coupures
  d'alimentation/USB** (reconnexion sale du CH340/CP210x après e-stop) ;
- une **calibration automatique** des échelles de vitesse et des gains de
  feedforward, par moteur.

Le fichier est `#pragma once`, entièrement `inline`, à inclure directement
dans `MecaMateSystemHardware` (interface `ros2_control`) ou dans un banc de
test autonome comme `main.cpp`.

---

## 2. Architecture du contrôle de vitesse

```
                     ┌─────────────────────────────────────────┐
                     │              pidLoop() @ 25 Hz            │
                     │                                           │
  target_[i] ───────▶│  measured = ticks_delta / window_dt / scale │
  (set_motor)         │  error    = target − measured              │
                     │  ff_term  = kS·sign(target) + kV·target    │
                     │  raw_cmd  = ff_term + kp·error + ki·∫ + D   │
                     │  cmd_out  = clamp(raw_cmd, −100, 100)       │
                     └─────────────────┬─────────────────────────┘
                                       │
                                       ▼
                          writeMotorRaw() → UART → Yahboom firmware
                                       │
                                       ▼
                              moteur DC + réducteur
                                       │
                                       ▼
                          encodeur → FUNC_REPORT_ENCODER (~24.4 Hz)
                                       │
                                       ▼
                          receiveLoop() → encoder_m[1..4]_ (atomic)
```

Deux threads tournent en parallèle de l'application ROS 2 :

| Thread          | Rôle                                                          | Fréquence |
| --------------- | -------------------------------------------------------------- | --------- |
| `recv_thread_`  | Lit l'UART, parse les trames, met à jour les atomics capteurs   | event-driven (~24-25 Hz pour l'encodeur) |
| `pid_thread_`   | Boucle de contrôle vitesse, écrit les commandes PWM             | 25 Hz fixe (`kPidHz`) |

Le thread `pid_thread_` ne tourne que si `enable_pid_control()` a été appelé.
Sans PID actif, `set_motor()` écrit directement la commande PWM brute (mode
"legacy", identique au driver Python d'origine).

---

## 3. Historique du PID logiciel (v2 → v8)

### v2 → v3 — Fondations
- Séparation gains/état dans des structs dédiées.
- `dt` réel mesuré (pas supposé constant).
- Premier filtre D, gestion du wrap encodeur (compteur 32 bits qui boucle),
  vérification de l'état UART, saturation explicite de la commande.

### v3 → v4 — Anti-windup et garde-fous de démarrage
- **Anti-windup pré-intégration** : l'intégrale n'est mise à jour qu'*après*
  vérification de la saturation, supprimant une incohérence de rollback
  (l'ancienne version intégrait puis annulait après coup, ce qui créait des
  à-coups d'un cycle).
- **`encoder_received_`** : flag empêchant le PID de démarrer avant la
  réception du premier paquet encodeur valide — évite un calcul de vitesse
  sur des données capteur non initialisées (souvent `0`, donnant une fausse
  vitesse énorme au premier cycle).
- `calibrate_pid_scale()` rejette les durées `< 100 ms` (trop peu de ticks
  pour une estimation fiable).

### v4 → v5 — Corrections de robustesse
| Bug                                                         | Correction                                              |
| ------------------------------------------------------------ | -------------------------------------------------------- |
| Typo `deriv_filtered` → `derivative_filtered`                | Renommage                                               |
| `calibrate_pid_scale()` pouvait retourner ≤ 0 silencieusement | Garde explicite + exception                             |
| Vitesse mesurée non bornée (pic possible en cas de glitch)     | Clamp à `[-200, 200]` %                                  |
| `set_motor()` ne clampait pas les cibles quand le PID était actif | Clamp `[-100, 100]` avant stockage dans `target_[i]`     |
| Division par `scale` potentiellement nulle → `NaN`            | Garde `std::max(scale, 1.0)`                             |
| `encoder_received_` non réinitialisé après reconnexion UART    | Reset dans `receiveLoop()` à chaque session             |

### v5 → v6 — Précision temporelle et calibration multi-moteurs
- **Biais de fenêtre corrigé** : les paquets encodeur Yahboom arrivent à
  ~24.4 Hz, pas exactement à 25 Hz (fréquence du PID). L'ancienne méthode
  comptait `n_samples × dt_nominal`, introduisant un biais systématique
  d'environ 2.5 %. La v6 stocke un timestamp réel par échantillon
  (`EncSlot{enc, ts_us}`) et calcule `window_dt` à partir des horodatages
  réels — le biais disparaît, quel que soit le rapport entre fréquence
  encodeur et fréquence PID.
- **Terme D repensé** : EMA appliquée à la vitesse mesurée (`measured`)
  plutôt qu'à `(error - prev_error)/window_dt`, avec un diviseur `dt` réel
  (dimensionnellement correct). *(Cette approche sera elle-même corrigée en
  v7 — voir plus bas : c'est elle qui causait le "zailling".)*
- **`calibrate_motor_scales()`** : calibration multi-runs, basée sur les
  ratios inter-moteurs (voir [§7](#7-procédure-de-calibration-sur-banc)),
  promue depuis un `main()` autonome vers une méthode de classe.
  `calibrate_pid_scale_at()` est conservée comme wrapper de compatibilité
  (délègue à `calibrate_motor_scales(n_runs=1)`).
- `kVelWindow = 10` (fenêtre ≈ 400 ms) confirmé empiriquement sur banc.

### v6 → v7 — Correction de l'instabilité à l'inversion ("zailling")

Trois bugs liés au signe, découverts en testant une inversion de direction
(+20 % → −20 %) :

**Bug 1 — D de mauvais signe à l'inversion.**
La v6 appliquait l'EMA sur `measured` (la vitesse), pas sur `error`. À
l'inversion, `measured` chute brutalement → le terme D, soustrait dans
`raw_cmd = target + kp·error + ki·∫ − d_term`, devenait positif au moment où
il fallait freiner — il **poussait dans le même sens que le dépassement**
plutôt que de l'amortir.

*Correction v7* : l'EMA est appliquée sur `error` (`target − measured`), pas
sur `measured`. Le terme D devient `+d_term` (additif) :
```
err_filt = α·err_filt_prev + (1−α)·error
d_term   = kd · (err_filt − err_filt_prev) / dt
raw_cmd  = target + kp·error + ki·∫ + d_term
```
Le signe de `d_term` est maintenant correct dans les deux sens de rotation,
et un step de consigne produit une impulsion D positive qui *aide*
l'accélération vers le nouveau setpoint au lieu de s'y opposer.

**Bug 2 — Intégrale bloquée pendant l'inversion.**
L'anti-windup d'origine n'intégrait que si `|raw_cmd| < 100`. À l'inversion,
la commande sature (`raw_cmd = -100`) pendant toute la phase de
décélération — l'intégrale chargée en marche avant restait figée à sa valeur
positive et continuait à pousser dans le mauvais sens pendant des centaines
de millisecondes.

*Correction v7* : l'anti-windup autorise aussi l'intégration quand l'erreur
et l'intégrale ont des signes opposés (l'intégrale "décharge" vers zéro — ce
cas est toujours sûr) :
```cpp
const bool not_saturated = std::abs(raw_cmd) < 100.0;
const bool winding_down  = (error * integral) < 0.0;
if (not_saturated || winding_down) { integral += error * dt; }
```

**Bug 3 — Nettoyage du struct `MotorPidState`.**
Le champ `prev_error` (v5) devenu obsolète a été retiré ; `MotorPidState`
contient désormais uniquement `integral`, `measured_filtered` (affichage via
`get_pid_measured()`) et `error_filtered` (terme D).

> **Note empirique** : avec `kd = 0.0` (réglage utilisé sur banc jusqu'à
> récemment), les corrections 1 et 3 sont transparentes — seule la
> correction 2 (anti-windup) avait un effet visible. Le terme D corrigé
> devient exploitable (`kd ≈ 0.02–0.05`) pour amortir les à-coups mécaniques
> propres aux roues mecanum, sans réintroduire d'instabilité à l'inversion.

### v7 → v8 — Feedforward moteur (kS + kV)

**Constat déclencheur** : la fiche moteur Yahboom mesurée donne :

| Condition          | Vitesse        | Courant    | Couple      |
| ------------------- | -------------: | ---------: | ----------: |
| À vide              | 333 tr/min     | 0,15 A     | —           |
| Charge nominale     | 250 tr/min     | 0,65 A     | 1 kg·cm     |
| Blocage (stall)     | 0 tr/min       | 2,4 A      | 4 kg·cm     |

Le moteur perd ~25 % de vitesse sous charge nominale (physique normale d'un
petit DC à balais), et possède une zone morte avant démarrage (friction
statique). Le PID v7 traitait `target` comme un feedforward 1:1 implicite
(`raw_cmd = target + kp·error + ...`), ce qui suppose un moteur parfaitement
linéaire et sans frottement — hypothèse fausse en pratique, en particulier à
basse vitesse.

**Modèle ajouté (v8)** :
```
pwm_ff = kS[i]·sign(target) + kV[i]·target
raw_cmd = pwm_ff + kp·error + ki·∫ + d_term
```
- `kS` (par moteur) : PWM % minimal nécessaire pour vaincre la friction
  statique et démarrer le mouvement (zone morte).
- `kV` (par moteur) : pente PWM-par-%-de-vitesse dans la région linéaire,
  mesurée sous la charge réelle présente pendant la calibration.
- Le PID ne corrige plus que le **résidu** autour de la prédiction
  feedforward, et non plus la commande entière — les gains `kp/ki/kd` tunés
  pour le v7 doivent être revus (généralement réduits).
- À `target == 0` exactement, `ff_term = 0` (pas de `sign(0)·kS`) : le robot
  ne "rampe" pas contre sa propre friction quand on demande l'arrêt.
- Rétrocompatible : tant que `calibrate_feedforward()` n'a pas été appelée
  ou que `enable_feedforward(false)`, `ff_gains_[i] = {kS=0, kV=1}` →
  comportement strictement identique au v7.

Voir [§5](#5-le-feedforward-kskv-v8) pour le détail de la calibration.

---

## 4. Le modèle moteur et ses limites physiques

Avant de blâmer le logiciel pour un manque de précision, il est utile de
savoir ce qui est **physiquement** limitant sur cette plateforme :

1. **Le moteur DC à balais lui-même** est la limite dominante à
   vitesse moyenne/haute : perte de vitesse sous charge (~25 %), sensibilité
   du couple au courant, non-linéarité PWM→vitesse (zone morte + friction).
   Aucun raffinement logiciel ne fera se comporter ce moteur comme un servo
   industriel (boucle de courant matérielle, contrôle FOC, etc.) — c'est un
   changement de matériel, pas de code.

2. **La résolution encodeur** devient le facteur limitant à **très basse
   vitesse** (manœuvres fines, juste après une inversion, approche d'un
   setpoint proche de zéro) : avec peu de ticks par fenêtre de mesure
   (~400 ms), le signal `measured` devient une fonction en escalier
   grossière, indépendamment de la qualité du moteur ou du PID.

3. **La boucle de courant** est gérée par le firmware Yahboom, hors de
   contrôle du code ROS 2 — le PID logiciel à 25 Hz hérite de tout le bruit
   et les à-coups produits en dessous, sans visibilité sur cette couche.

**Conclusion pratique** : le feedforward (v8) cible le point 1. Si tu
constates encore de l'imprécision en dessous d'une certaine vitesse de
consigne, le point 2 (résolution encodeur / mesure par temps-entre-impulsions
plutôt que ticks-par-fenêtre) reste la piste à explorer ensuite — voir
[§11](#11-limites-connues-et-pistes-futures).

---

## 5. Le feedforward kS/kV (v8)

### Principe

```
pwm_ff = kS[i] · sign(target) + kV[i] · target
```

- Si `target == 0` → `pwm_ff = 0` exactement (pas de terme `sign(0)`).
- Sinon, `sign(target)` vaut `+1` ou `−1`, et `kS` ajoute un "coup de pouce"
  constant pour vaincre la friction statique, dans le bon sens.
- `kV` est la pente linéaire au-delà de la zone morte.

### Calibration automatique — `calibrate_feedforward()`

```cpp
double calibrate_feedforward(int throttle_min_pct = 5,
                             int throttle_max_pct  = 70,
                             int step_pct          = 3,
                             int settle_ms         = 250,
                             int sample_ms         = 300);
```

**Algorithme**, par moteur indépendamment (mais les 4 roues sont pilotées
simultanément pour simplifier le séquençage matériel — chaque moteur garde
sa propre mesure encodeur) :

1. Balayage PWM de `throttle_min_pct` à `throttle_max_pct`, pas de
   `step_pct`.
2. À chaque palier : attente `settle_ms` (transitoire mécanique), puis
   mesure des ticks sur `sample_ms`.
3. **`kS[i]`** = premier palier PWM où le moteur dépasse un seuil de
   mouvement (8 ticks sur la fenêtre `sample_ms` — au-delà du bruit
   électrique/encodeur).
4. **`kV[i]`** = régression linéaire (moindres carrés) de `PWM = kS + kV·vitesse`
   sur les points de la région linéaire (au-delà de la zone morte
   observée). Garde-fous :
   - si la matrice est dégénérée (tous les points à la même vitesse, ou un
     seul point) → repli sur un ratio simple plutôt qu'un calcul instable ;
   - si l'ordonnée à l'origine du fit (`kS` théorique) est très inférieure à
     la zone morte *observée* directement → on garde la valeur observée
     (plus fiable qu'une extrapolation du fit) ;
   - `kV` est toujours clampé `≥ 0.1` (refuse une pente négative ou nulle,
     qui produirait un feedforward s'opposant au mouvement) ;
   - `kS` est clampé dans `[0, 50]` %.

**Préconditions** : PID désactivé, roues libres de tourner, auto-report
actif (mêmes préconditions que `calibrate_motor_scales()`).

**Après calibration**, il faut explicitement activer le feedforward :
```cpp
bot.calibrate_feedforward();
bot.enable_feedforward(true);   // sans ce flag, ff_term = target (comportement v7)
```

### Régler manuellement (sans calibration auto)

```cpp
bot.set_feedforward_gains(/*motor_index=*/0, /*kS=*/8.0, /*kV=*/0.6);
bot.get_feedforward_gains(0, kS, kV);   // relecture
```

---

## 6. Cycle de vie du port série

Contexte historique : un bug nécessitait de **rebooter le Raspberry Pi**
après chaque e-stop suivi d'un cycle d'alimentation Yahboom. Cause racine :
le contrôleur tty noyau (CH340/CP210x) garde un état interne par descripteur
de fichier ; sur perte d'alimentation USB, le noyau envoie un signal
`VHANGUP`, déclenchant une cascade d'échecs si le code ne le gère pas
explicitement.

| # | Problème                                                      | Correction                                                              |
| - | --------------------------------------------------------------- | ------------------------------------------------------------------------ |
| 1 | `open()` bloquant indéfiniment sur une session VHANGUP           | `O_NONBLOCK` à l'ouverture, retiré immédiatement après via `fcntl`        |
| 2 | `tcsetattr()` silencieusement ignoré sur un fd qui vient de réapparaître | `tcflush(TCIOFLUSH)` **avant** `tcgetattr()`                              |
| 3 | Un autre processus (udev, ModemManager) vole le port             | `TIOCEXCL` immédiatement après `open()`                                  |
| 4 | Verrou exclusif conservé par le noyau après un `close()` mal formé | `TIOCNXCL` avant `close()`                                               |
| 5 | Race thread/fd dans le destructeur (`EBADF` si le port se ferme pendant un `read()`) | Ordre strict : `uart_running_=false` → `join()` → `ser_.close()`         |
| 6 | `EIO` non géré dans `receiveLoop()` laissait le fd ouvert côté noyau | `ser_.close()` explicite sur toute erreur fatale (`EIO`/`ENOTTY`/`EBADF`) |
| 7 | DTR/RTS coupé au `close()`, faisant disparaître le `/dev/ttyUSBx` | `HUPCL` désactivé dans `c_cflag`                                          |

Un **watchdog de silence** (1500 ms sans trame valide) déclenche également
une fermeture propre — utile si l'appareil se déconnecte sans générer
d'erreur `EIO` explicite (certains hubs USB).

---

## 7. Procédure de calibration sur banc

Ordre obligatoire (chaque étape suppose le PID désactivé et les roues libres
de tourner) :

```
1. create_receive_threading()
2. set_auto_report_state(true, true)
3. (attendre les premiers paquets — battery_voltage != 0)
4. calibrate_motor_scales(...)      ── échelles ticks/s par moteur
5. calibrate_feedforward(...)       ── kS/kV par moteur
6. enable_feedforward(true)
7. enable_pid_control(kp, ki, kd, scale_global)
```

### `calibrate_motor_scales()` — détail de l'algorithme

1. Warmup thermique optionnel (60 % pendant `warmup_ms`, défaut 3 s) — les
   bobines DC chauffent et leur résistance change les premières secondes.
2. `n_runs` mesures indépendantes à `throttle_pct` % (défaut 60 %, 800 ms).
3. Pour chaque run : lecture stable avant (deux lectures identiques à 25 ms
   d'écart) → impulsion moteur → lecture stable après.
4. **Ratios inter-moteurs** (`ticks[i] / moyenne_run`) calculés par run, puis
   moyennés sur tous les runs — robuste à une dérive thermique globale
   (±5 %) car les ratios restent stables même si l'absolu varie.
5. **Échelle globale** = moyenne tronquée des échantillons globaux par run
   (rejette min et max si `n_runs ≥ 3`).
6. **Échelle par moteur** = échelle globale × ratio moyen du moteur.
7. Un coefficient de variation (`CV`) est affiché par moteur ; `CV > 1.5 %`
   déclenche un avertissement (moteur potentiellement irrégulier — roulement
   sec, câblage défaillant).

### `calibrate_feedforward()` — voir [§5](#5-le-feedforward-kskv-v8)

---

## 8. Utilisation du banc de test (`main.cpp`)

Le fichier `main.cpp` exécute la séquence complète **en un seul run
automatique**, dans l'ordre :

```
1. Ouverture port (/dev/ttyUSB0) + thread de réception
2. set_auto_report_state(true, true)
3. Attente du premier paquet valide (timeout 2 s)
4. calibrate_motor_scales(60%, 800ms, 5 runs, warmup 3s)
5. calibrate_feedforward(5%→70%, pas 3%, settle 250ms, sample 300ms)
   → affichage kS/kV par moteur
6. enable_feedforward(true)
7. enable_pid_control(kp=0.3, ki=0.05, kd=0.0, scale_global)
8. Test de convergence : avance à −20% pendant 50 s, log toutes les 100 ms
   → colonnes : ticks bruts par fenêtre de 100 ms + vitesse PID en % par moteur
9. Arrêt moteur, attente 2 cycles PID, disable_pid_control()
```

### Compiler et lancer

```bash
g++ -std=c++17 -Wall -Wextra -pthread main.cpp -o main_test
./main_test
```

### Lire les logs

```
  t(ms)  raw_M1  raw_M2  raw_M3  raw_M4  pid%:     M1     M2     M3     M4
    100    -130    -128    -131    -129       -19.8  -19.6  -19.9  -19.7
```
- `raw_Mx` : delta de ticks encodeur brut sur la fenêtre de 100 ms (signé).
- `pid%` (`get_pid_measured()`) : vitesse mesurée par le PID, en % de
  l'échelle calibrée — c'est la valeur que voit réellement la boucle de
  contrôle, à comparer à la cible (`-20.0` dans ce test).

**Préconditions matérielles** : roues libres de tourner (calibration), câble
USB et alimentation Yahboom stables, `/dev/ttyUSB0` accessible (vérifier les
permissions udev si l'ouverture échoue avec `Permission denied`).

---

## 9. Réglage des gains PID

### Avant le feedforward (v7 et antérieur)

`raw_cmd = target + kp·error + ki·∫ + d_term` — le terme `target` agissait
déjà comme feedforward 1:1 implicite, donc `kp/ki` ne corrigeaient que
l'écart résiduel par rapport à un modèle linéaire parfait. Valeurs utilisées
sur banc : `kp=0.6, ki=0.1, kd=0.0`.

### Avec le feedforward actif (v8)

`raw_cmd = ff_term + kp·error + ki·∫ + d_term`, où `ff_term` porte
maintenant l'essentiel de la commande prédictive. Le PID ne corrige plus que
le **résidu** (perturbations, imprécision du fit, variations de charge en
temps réel) — il est normal et attendu que `kp/ki` puissent être réduits par
rapport au v7. Point de départ utilisé dans `main.cpp` : `kp=0.3, ki=0.05,
kd=0.0` — **à valider et affiner sur banc**, ce ne sont pas des valeurs
calibrées.

### `kd` — terme dérivé

Resté à `0.0` historiquement car l'oscillation observée sur les roues
mecanum était mécanique (jeu, accroche), pas une instabilité logicielle.
Depuis la correction v7 (D appliqué sur `error_filtered`, pas sur
`measured_filtered`), le terme D est sign-correct dans les deux sens de
rotation et devient exploitable : `kd ≈ 0.02–0.05` est une fourchette de
départ raisonnable pour amortir les à-coups mécaniques, à valider
empiriquement.

### Réglage par moteur

Si un moteur diverge en comportement des trois autres (CV élevé en
calibration, dead zone très différente) :
```cpp
bot.set_motor_pid_gains(/*motor_index=*/2, kp, ki, kd, /*override=*/true);
bot.reset_motor_pid_gains(2);   // retour aux gains globaux
```

---

## 10. Référence API

### Cycle de vie
| Méthode                                    | Rôle                                                    |
| -------------------------------------------- | --------------------------------------------------------- |
| `create_receive_threading()`                 | Démarre le thread de réception UART                       |
| `set_auto_report_state(enable, forever)`      | Active le rapport automatique de capteurs/encodeurs        |
| `is_running()`                                | État du thread de réception                                |

### Calibration
| Méthode                                                                  | Retour                          |
| --------------------------------------------------------------------------- | ---------------------------------- |
| `calibrate_motor_scales(throttle_pct, duration_ms, n_runs, warmup_ms, use_per_motor)` | échelle globale (ticks/s)         |
| `calibrate_pid_scale_at(throttle_pct, duration_ms, use_per_motor)`            | wrapper compat. (`n_runs=1`)       |
| `calibrate_pid_scale(duration_ms)`                                           | échelle globale, méthode historique simple |
| `calibrate_feedforward(min_pct, max_pct, step_pct, settle_ms, sample_ms)`     | zone morte moyenne (%)            |
| `set_motor_scales(scales, global)`                                           | injection manuelle des échelles    |
| `set_feedforward_gains(motor_index, kS, kV)`                                  | injection manuelle kS/kV           |
| `get_feedforward_gains(motor_index, kS&, kV&)`                                | lecture kS/kV courants             |

### Contrôle PID
| Méthode                                                  | Rôle                                            |
| ----------------------------------------------------------- | -------------------------------------------------- |
| `enable_pid_control(kp, ki, kd, ticks_per_sec)`              | démarre le thread PID @ 25 Hz                      |
| `disable_pid_control()`                                      | arrête le thread PID, reset l'état                 |
| `set_pid_gains(kp, ki, kd)`                                  | gains globaux (tous moteurs sans override)         |
| `set_motor_pid_gains(motor_index, kp, ki, kd, override)`     | gains spécifiques à un moteur                       |
| `reset_motor_pid_gains(motor_index)`                         | retour aux gains globaux pour ce moteur            |
| `enable_feedforward(enable)`                                  | active/désactive le terme feedforward              |
| `get_pid_measured()`                                          | vitesse mesurée actuelle (%) par moteur            |

### Commande moteur
| Méthode                                          | Comportement                                              |
| --------------------------------------------------- | -------------------------------------------------------------- |
| `set_motor(s1, s2, s3, s4)`                          | si PID actif → dépose les cibles ; sinon → PWM direct           |
| `set_motor_with_compensation(s1, s2, s3, s4)`        | applique la compensation de pente avant `set_motor`             |
| `set_car_motion(vx, vy, vz)`                         | commande cinématique haut niveau (firmware Yahboom)             |

---

## 11. Limites connues et pistes futures

- **Résolution encodeur à basse vitesse** : la mesure par fenêtre fixe
  (`kVelWindow = 10`, ≈400 ms) quantifie grossièrement `measured` quand peu
  de ticks tombent dans la fenêtre. Une mesure par **temps-entre-impulsions**
  serait plus précise à basse vitesse, mais nécessite de connaître le nombre
  exact de ticks par tour de roue (après réducteur) pour juger si le gain en
  vaut l'implémentation.
- **Fréquence de boucle PID** : `kPidHz = 25` est bas comparé à des
  plateformes industrielles (boucle vitesse en centaines de Hz à quelques
  kHz). Une augmentation est possible si la latence UART/firmware Yahboom le
  permet, mais n'a pas été testée.
- **Feedforward non testé en charge réelle** : la calibration roues libres
  donne un `kV` valide à vide ; sous charge (robot au sol, irrégularités de
  terrain), la pente réelle sera différente (cf. [§4](#4-le-modèle-moteur-et-ses-limites-physiques),
  perte de ~25 % sous charge nominale). Le PID doit absorber cet écart via
  le terme résiduel — à valider sur le robot complet, pas seulement sur
  banc roues libres.
- **Gains v8 non finalisés** : `kp=0.3, ki=0.05, kd=0.0` dans `main.cpp`
  sont un point de départ, pas un réglage validé. À itérer sur le test de
  convergence du banc.
- **Boucle de courant** : hors de portée du code ROS 2 — gérée
  exclusivement par le firmware Yahboom, sans visibilité ni contrôle direct.
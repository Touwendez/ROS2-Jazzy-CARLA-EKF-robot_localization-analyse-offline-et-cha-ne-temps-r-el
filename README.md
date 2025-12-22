#  EKF (robot_localization) : analyse offline chaîne temps réel

## Pourquoi ce dépôt existe
Ce dépôt regroupe notre travail autour de la localisation d’un véhicule (CARLA) avec **ROS2 Jazzy** et un **EKF** (package `robot_localization`).

On a commencé par une analyse **hors-ligne** (export CSV + notebook) pour valider nos métriques et comparer plusieurs configurations (odom seule, IMU seule, fusion).  
Ensuite, on a construit la version **temps réel** : on s’abonne aux topics ROS2, on calcule les métriques au fil de l’eau, et on les affiche en direct avec PlotJuggler.

---

## Étape A1 : Analyse hors-ligne (CSV + notebook)
Dans l’étape A1, on a exporté les runs en **CSV** et on a construit un notebook qui :
- charge les runs,
- calcule les erreurs \(e_x\), \(e_y\),
- calcule la norme 2D \(e_{2d}=\sqrt{e_x^2+e_y^2}\),
- calcule le **RMSE**,
- affiche les courbes pour comparer les 3 configurations EKF.

Cette partie est utile pour analyser calmement les performances et garder une base reproductible.

---

## Étape A2 : Chaîne temps réel ROS2 (topics, métriques live, visualisation)
L’objectif de l’étape A2 est d’avoir la même analyse, mais **en direct** :
- l’EKF tourne dans ROS2,
- un node calcule les métriques en continu,
- PlotJuggler affiche les courbes en temps réel.

### Un point important : les timestamps doivent être cohérents
On a rencontré (et corrigé) un problème classique :  
- la vérité terrain CARLA (ex. `/carla/hero/odometry`) est horodatée en **temps simulé** (ex. ~180 s),
- la sortie EKF pouvait être horodatée en **temps système (epoch)**.

Dans ce cas, comparer les messages “instant par instant” n’a plus de sens.  
La solution propre consiste à :
- jouer le rosbag avec `--clock` (publication de `/clock`),
- activer `use_sim_time:=true` sur l’EKF et sur le node de métriques.

---

## Ce qu’on mesure en temps réel (topics `/metrics/`)
Le script `online_compare.py` publie en live :
- `/metrics/ex` et `/metrics/ey` : erreurs en x/y,
- `/metrics/e2d` : norme 2D de l’erreur,
- `/metrics/rmse2d_window` : RMSE 2D sur une fenêtre glissante (ex. 5 s),
- `/metrics/compute_ms` : temps CPU réel pour calculer les métriques (preuve “temps réel”),
- `/metrics/offset_ts` : décalage entre timestamps GT et EKF (contrôle d’alignement temporel).

---

## Vidéo PlotJuggler (démo)


Une vidéo de démonstration est incluse dans le dépôt.


https://github.com/user-attachments/assets/5962419e-e050-4794-9170-d4bd04219640


Elle montre l’évolution **en temps réel** des courbes PlotJuggler :
- `e2d` (erreur 2D instantanée),
- `rmse2d_window` (RMSE sur fenêtre glissante),
- `compute_ms` (coût de calcul du node).

Quelques remarques pour lire correctement la vidéo :
- `e2d` peut varier en fonction de la dynamique (virages/accélérations) et de l’action de correction du filtre.
- `rmse2d_window` est plus lisse car elle moyenne sur une fenêtre temporelle.
- `compute_ms` reste généralement faible (sub-millisecondes à quelques ms), ce qui valide la compatibilité temps réel du calcul.
- Dans la vidéo, le node de métriques est lancé après quelques secondes : avant cela, PlotJuggler n’affiche rien car aucun topic `/metrics/*` n’est publié.
- Si le rosbag est joué en boucle (`--loop`), un “saut en arrière du temps” peut survenir au rebouclage ; notre script gère ce cas en réinitialisant la fenêtre RMSE pour limiter les artefacts.

---

# Comment reproduire la démo (pas à pas)

## 1) Rejouer le rosbag (en boucle + horloge simulée)
On démarre par le rosbag. C’est lui qui fournit les données (odom/IMU/etc.) et l’horloge `/clock`.

```bash
ros2 bag play ~/Téléchargements/carla_data --loop --clock


## Étape A2 — Visualisation live (PlotJuggler) et vérifications

Cette partie décrit la procédure que nous utilisons pour reproduire la démo “temps réel” à partir d’un rosbag : lecture des données, lancement de l’EKF `robot_localization`, calcul des métriques en ligne, puis affichage dans PlotJuggler. L’objectif est d’avoir une chaîne cohérente en temps simulé, afin de comparer correctement la sortie filtrée et la vérité terrain.

### 1) Vérifier que l’horloge simulée est disponible
Avant de lancer l’EKF, on s’assure que `/clock` existe et avance. C’est indispensable dès qu’on utilise `use_sim_time=true`.

```bash
ros2 topic list | grep clock
ros2 topic echo /clock --once

# Procédure complète (A2) — Lancer le rosbag, l’EKF, les métriques et PlotJuggler

Ce document décrit la procédure que nous utilisons pour reproduire la chaîne “temps réel” :
rosbag → EKF (`robot_localization`) → calcul des métriques → visualisation PlotJuggler.

L’idée est simple : on rejoue un scénario enregistré, on filtre avec l’EKF, puis on compare en ligne la sortie filtrée à la vérité terrain.

---

## Pré-requis
- ROS2 Jazzy installé et sourcé (`source /opt/ros/jazzy/setup.bash`)
- Workspace sourcé si besoin (`source ~/ros2_ws/install/setup.bash`)
- Le rosbag CARLA (dossier contenant `metadata.yaml` + `.db3`)
- Les scripts et la config EKF présents dans le repo :
  - `config/ekf_imu_odom.yaml`
  - `scripts/online_compare.py`

---

## Organisation conseillée (4 terminaux)
- Terminal 1 : rosbag
- Terminal 2 : EKF
- Terminal 3 : métriques (online_compare)
- Terminal 4 : PlotJuggler + vérifications

---

## Étape 0 — (Optionnel mais recommandé) Sourcing
À faire dans chaque terminal si nécessaire :

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash



# A1 — Analyse temps réel basée sur des topics ROS 2

## Objectif de l’étape A1

L’objectif est de se rapprocher d’un fonctionnement réaliste : les métriques de performance du filtre **EKF** sont calculées **au fil de l’arrivée des données**, sans étape intermédiaire d’enregistrement.

L’étape A1 consiste donc à :
- comparer en temps réel la sortie du filtre EKF à une référence,
- calculer des métriques d’erreur et de performance,
- publier ces métriques sous forme de topics ROS 2 exploitables par des outils de visualisation.

---

## Chaîne ROS 2 mise en place

### Données d’entrée
Deux sources principales sont utilisées :

1) **Référence (ground truth)**  
Topic : `/carla/hero/odometry`  
Ce topic fournit la pose du véhicule issue du simulateur **CARLA**, considérée comme référence dans ce contexte.

2) **Estimation EKF**  
Topic : `/odometry/filtered`  
Ce topic correspond à la sortie du filtre EKF du package `robot_localization`, configuré à partir des capteurs disponibles (odométrie et/ou IMU selon les tests).

> Les données sont rejouées à partir d’un **rosbag en boucle**, ce qui permet de tester le système sur un scénario identique et répétable.

---

## Synchronisation des messages
Les messages provenant des deux topics ne sont pas publiés exactement au même instant.  
Pour comparer des états correspondant au même moment de la trajectoire, une synchronisation approximative est utilisée :

- **Approximate Time Synchronizer**

Cette méthode associe deux messages dont les timestamps sont proches, tout en restant tolérante à de légers décalages temporels.

---

## Node d’analyse temps réel

Un node ROS 2 Python (`online_compare.py`) a été développé pour réaliser l’analyse en ligne.

### Rôle du node
Le node :
- s’abonne aux topics de référence et de sortie EKF,
- associe les messages synchronisés,
- calcule les métriques d’erreur et de performance,
- publie ces métriques sur de nouveaux topics ROS 2.

---

## Métriques calculées

À chaque paire de messages synchronisés, les métriques suivantes sont calculées :

### Métriques calculées

- **Erreur en x**  
  `e_x = x_EKF - x_ref`

- **Erreur en y**  
  `e_y = y_EKF - y_ref`

- **Erreur de position 2D**  
  `e_2D = sqrt(e_x^2 + e_y^2)`

- **Latence temporelle**  
  Différence entre le timestamp de la sortie EKF et celui de la référence.

- **RMSE 2D glissant**  
  Calculé sur une fenêtre temporelle glissante (quelques secondes), permettant de suivre l’évolution récente de l’erreur.

---

## Topics de sortie (métriques publiées)

Les métriques sont publiées sous forme de topics ROS 2 :

- `/metrics/ex`
- `/metrics/ey`
- `/metrics/e2d`
- `/metrics/rmse2d_window`
- `/metrics/offset_ts`
- `/metrics/lag_sim`
- `/metrics/compute_ms`

Cette architecture rend le système **modulaire** et permet d’utiliser différents outils de visualisation/analyse sans modifier le calcul des métriques.

---

## Résultats obtenus

### Erreur de position
La valeur instantanée observée pour l’erreur 2D est de l’ordre de :

`e_2D ≈ 1 × 10^-8 m`

Cette valeur est négligeable et indique une superposition quasi parfaite entre la trajectoire estimée par l’EKF et la trajectoire de référence fournie par CARLA (dans cette configuration).

### Fréquence de publication
Les métriques sont publiées à une fréquence moyenne d’environ **6 à 7 Hz**, correspondant au rythme effectif des paires de messages synchronisées (référence + sortie EKF).

La fréquence est stable, ce qui confirme le bon fonctionnement :
- du mécanisme de synchronisation,
- du calcul en temps réel.

### Cohérence avec l’analyse hors-ligne
Les résultats en temps réel sont cohérents avec l’analyse hors-ligne (CSV) :
- RMSE quasi nul pour les configurations utilisant l’odométrie,
- absence de latence significative.

Cela valide la cohérence entre les deux approches (offline et online).


---

## Étape A2 : Chaîne temps réel ROS2 (topics, métriques live, visualisation)
L’objectif de l’étape A2 est d’avoir la même analyse, mais **en direct** :
- l’EKF tourne dans ROS2,
- un node calcule les métriques en continu,
- PlotJuggler affiche les courbes en temps réel.
### Cohérence des timestamps (temps simulé vs temps système)

Un point critique a été identifié puis corrigé : **les deux sources comparées doivent partager la même base de temps**.

Dans notre cas :
- la vérité terrain CARLA (ex. `/carla/hero/odometry`) est horodatée en **temps simulé** (ex. ~180 s),
- la sortie du filtre EKF pouvait être horodatée en **temps système** (type epoch).

Cette divergence rend la synchronisation invalide : les messages ne correspondent plus au même instant, et la comparaison “point à point” (ainsi que les métriques) n’a plus de sens.

#### Correction appliquée (configuration recommandée)
Pour garantir une comparaison correcte, toute la chaîne est forcée en **temps simulé** :

- jouer le rosbag en publiant l’horloge :
  ```bash
  ros2 bag play <bag> --loop --clock

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


---

## Discussion et limites
Dans la configuration actuelle, l’odométrie issue du simulateur CARLA est **très précise**.  
Par conséquent :
- l’erreur mesurée est extrêmement faible,
- le filtre EKF ne montre pas de gain visible par rapport à la référence.

C’est toutefois normal dans un environnement de simulation idéal.  
Cela justifie la mise en place de scénarios plus réalistes dans les étapes suivantes, notamment via :
- ajout de bruit,
- capteurs dégradés.

---

## Conclusion
L’étape A1 valide la mise en place d’une chaîne ROS 2 temps réel complète, allant :
- de la souscription aux topics de capteurs et d’estimation,
- au calcul en ligne des métriques d’erreur et de performance,
- jusqu’à la publication de ces métriques sous forme de nouveaux topics.

Cette étape constitue une base solide pour la suite du projet, en particulier pour :
- la visualisation temps réel des performances,
- l’étude de scénarios plus complexes,
- l’analyse de l’impact des réglages du filtre EKF.

---




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


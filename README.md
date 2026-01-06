## 🔗 Projets liés

* **Évaluation expérimentale – Fusion multi-capteurs**
* 
  👉 [https://github.com/Touwendez/Evaluation-exp-rimentale-Fusion-multi-capteurs](https://github.com/Touwendez/Evaluation-exp-rimentale-Fusion-multi-capteurs)

* **Localisation multi-capteurs – ZOE / ROS2 / CARLA**
* 
  👉 [https://github.com/Touwendez/localisation-multicapteurs-zoe-ros2-carla](https://github.com/Touwendez/localisation-multicapteurs-zoe-ros2-carla)

* **Évaluation expérimentale – Fusion multi-capteurs (Suite 2)**
* 
  👉 [https://github.com/Touwendez/Evaluation-exp-rimentale-Fusion-multi-capteurs-suite-2](https://github.com/Touwendez/Evaluation-exp-rimentale-Fusion-multi-capteurs-suite-2)



# A1 : Analyse temps réel basée sur des topics ROS 2 

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
  <img width="159" height="67" alt="Screenshot from 2025-12-26 17-51-23" src="https://github.com/user-attachments/assets/5b993842-879d-42a2-a581-57c035862629" />


- **Latence temporelle**  
  Différence entre le timestamp de la sortie EKF et celui de la référence.

- **RMSE 2D glissant**  
  Calculé sur une fenêtre temporelle glissante (quelques secondes), permettant de suivre l’évolution récente de l’erreur.

---

## Topics de sortie (métriques publiées)

<img width="425" height="395" alt="Screenshot from 2025-12-26 16-16-03" src="https://github.com/user-attachments/assets/3a5f4a55-d3f4-43ee-8631-223d87fce86f" />


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


<img width="153" height="39" alt="Screenshot from 2025-12-26 17-52-41" src="https://github.com/user-attachments/assets/028fabf2-6900-4e32-a9e8-2dd42c0f28b8" />


---

<img width="662" height="74" alt="Screenshot from 2025-12-26 18-31-06" src="https://github.com/user-attachments/assets/15658051-5d62-4fff-a08e-9b912a57e9b8" />


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


# C : Étude paramétrique des incertitudes EKF (Q et P0) avec `robot_localization`

## C.1 Objectif

L’objectif de cette partie est d’étudier l’influence des **paramètres d’incertitude** du filtre EKF (`robot_localization`) sur les performances d’estimation de pose dans un environnement simulé CARLA.

Plus précisément, on cherche à comprendre l’effet de :

- **Q : process_noise_covariance** : incertitude associée au **modèle de prédiction** (processus).
  → Q élevé : on considère que le modèle est moins fiable.  
  → Q faible : on considère que le modèle est plus fiable.

- **P0 : initial_estimate_covariance** : incertitude sur **l’état initial** au démarrage.
  → P0 élevé : le filtre “admet” qu’il ne connait pas bien l’état initial et corrige plus fortement au début.

Cette étude s’inscrit dans le choix de paramètres (Q/P0) aux résultats observables (erreurs, stabilité, réactivité, etc.).

---

## C.2 Configurations testées

Trois configurations EKF ont été créées dans le répertoire :

`~/ros2_ws/src/my_py_pkg/config/test_compare/`

1. **Qlow** : configuration avec **Q faible**
2. **Qhigh** : configuration avec **Q élevée**
3. **P0high** : configuration avec **P0 élevée** (Q conservée “moyenne” pour isoler l’effet de P0)

Remarque technique importante (ROS2 / YAML) : lors de la création des fichiers, ROS2 refuse une liste
qui mélange des entiers (`0`) et des flottants (`0.0`, `1e-3`, etc.).
Il a donc fallu écrire **uniquement des flottants** dans les matrices (utiliser `0.0` au lieu de `0`), sinon
erreur du type :

`Sequence should be of same type. Value type 'integer' do not belong ...`

---

## C.3 Protocole expérimental

L’expérimentation a été menée de manière reproductible en rejouant les **mêmes données** CARLA pour les
trois configurations.

### C.3.1 Données et topics utilisés

- **Données** : rosbag CARLA rejoué en boucle
- **Référence (ground truth simulation)** : `/carla/hero/odometry`
- **Sortie du filtre EKF** : `/odometry/filtered`

### C.3.2 Étapes d’un run (valable pour chaque config)

Pour chaque configuration (Qlow, Qhigh, P0high), la procédure est identique :

1. Lecture du rosbag CARLA en boucle :

   `ros2 bag play <bag> --loop`

2. Lancement du filtre EKF :

   `ros2 run robot_localization ekf_node --ros-args --params-file <config.yaml>`

3. Enregistrement comparatif des données via `compare_logger.py` :
   - Synchronisation approx des messages référence (hero) et EKF (filtered)
   - Génération d’un CSV contenant : timestamps, positions, orientations, et delta temps

4. Renommage du CSV de façon explicite :
   - `run_Qlow.csv`
   - `run_Qhigh.csv`
   - `run_P0high.csv`

---

## C.3.3 Extraction des métriques et création du tableau

Un script Python `summarize_runs.py` lit les trois fichiers CSV et calcule automatiquement :

- **RMSE_x**, **RMSE_y**, **RMSE_2D**
- **RMSE_yaw**
- **lat_mean**, **lat_max**
- durée analysée et fréquence nominale estimée

Le résultat est sauvegardé dans `runs_summary.csv`.

---

## C.4 Métriques retenues

Les métriques utilisées sont :

- **RMSE_x [m]** : écart quadratique moyen sur l’axe x entre EKF et référence
- **RMSE_y [m]** : écart quadratique moyen sur l’axe y
- **RMSE_2D [m]** : RMSE sur la norme 2D = √ ( (x_f − x_h)² + (y_f − y_h)² )
- **RMSE_yaw [rad]** : RMSE sur l’angle yaw (en tenant compte du wrap [−π, π])
- **lat_mean [s] / lat_max [s]** : différence temporelle moyenne / max entre les timestamps appariés
- **dt_nom_s / hz_ref** : pas de temps nominal et fréquence de référence estimée

---
## 7) Resultats (runs_summary.csv`)

| Config | Fichiers          | N   | Duration (s) | dt_nom (s) | Hz ref | RMSE_x (m)     | RMSE_y (m)     | RMSE_2D (m)    | RMSE_yaw (rad) | lat_mean (s) | lat_max (s) |
|--------|---------------|-----|--------------|------------|--------|----------------|----------------|----------------|----------------|--------------|-------------|
| Qlow   | run_Qlow.csv   | 194 | 9.65         | 0.05       | 20.0   | 3.628353e-06   | 3.554437e-06   | 5.079269e-06   | 3.799077e-07   | 0.0          | 0.0         |
| Qhigh  | run_Qhigh.csv  | 189 | 9.40         | 0.05       | 20.0   | 1.313342e-06   | 8.758971e-07   | 1.578627e-06   | 0.000000e+00   | 0.0          | 0.0         |
| P0high | run_P0high.csv | 191 | 9.50         | 0.05       | 20.0   | 5.267703e-07   | 5.167356e-07   | 7.379042e-07   | 1.023289e-07   | 0.0          | 0.0         |


---

## C.6 Analyse et interprétation des résultats

### C.6.1 Observations générales

Les RMSE obtenus sont extrêmement faibles (ordre de grandeur 10⁻⁶ m, soit quelques micromètres).
De même, la latence mesurée est nulle (`lat_mean = lat_max = 0.0 s`).

Ces résultats indiquent que, dans ce scénario, l’estimation EKF est **quasi identique** à la référence CARLA.

**Explication principale** : les données simulées CARLA (en particulier l’odométrie) sont très propres et
cohérentes, ce qui conduit naturellement à une superposition quasi parfaite entre référence et sortie EKF.
Dans ce contexte, l’impact de Q et P0 sur le RMSE global est faible, car la correction du filtre repose sur une
mesure déjà très précise.

### C.6.2 Effet de Q (Qlow vs Qhigh)

Théoriquement :

- **Q faible (Qlow)** : le filtre considère que le modèle de prédiction est fiable → comportement attendu :
  estimation plus “rigide” et plus lissée.
- **Q élevé (Qhigh)** : le filtre considère que le modèle est incertain → comportement attendu : estimation
  potentiellement plus réactive (et plus sensible aux mesures).

Dans nos résultats, l’erreur reste quasi nulle dans les deux cas. Les différences observées (de l’ordre du
micro-mètre) restent très faibles et peuvent être influencées par :

- la précision numérique,
- la synchronisation approx des messages,
- la durée légèrement différente des runs.

On peut donc conclure que **dans ce scénario CARLA**, la différence entre Qlow et Qhigh ne se traduit pas
par un gain significatif en RMSE global.

### C.6.3 Effet de P0 (P0high)

Théoriquement, P0 agit surtout au démarrage du filtre :

- **P0 élevé** : le filtre part avec une forte incertitude → il accepte plus facilement les corrections initiales
  (convergence potentiellement plus rapide).
- **P0 faible** : le filtre est “confiant” dès le début → corrections initiales plus faibles.

Dans ce test, l’erreur globale étant déjà extrêmement faible, l’effet de P0 n’apparaît pas fortement dans les
RMSE finaux. Cependant, P0high donne ici les plus faibles RMSE (toujours dans des valeurs très petites),
ce qui reste cohérent avec une convergence initiale légèrement plus “souple”.

### C.6.4 Latence et cohérence temporelle

Les valeurs `lat_mean` et `lat_max` sont à 0.0 s, ce qui signifie que les couples de messages (référence vs
EKF) ont été synchronisés sans décalage notable.

Cela valide la cohérence de la méthode de comparaison.

---

## C.7 Conclusion de la partie C

Cette partie a permis de :

1. Mettre en place une **méthode reproductible** de test paramétrique (Q/P0) :
   - rosbag identique,
   - exécution EKF avec paramètres contrôlés,
   - génération de CSV,
   - extraction automatique des métriques,
   - construction d’un tableau comparatif.

2. Observer que, sur ce scénario CARLA, les performances mesurées sont quasi parfaites (RMSE ~ 10⁻⁶
   m), ce qui masque en grande partie l’impact des variations de Q et P0 sur l’erreur globale.

Ainsi, la campagne C valide surtout la **démarche** et l’outil de comparaison.

Pour rendre l’impact des covariances plus visible et interprétable, la suite logique est de travailler sur des
scénarios plus “difficiles” (bruit artificiel, changements de conduite, perturbations), ce qui permettra de relier
plus clairement les paramètres Q/R/P0 aux courbes et aux métriques (RMSE, stabilité, réactivité).



# Rapport — Tests EKF avec odométrie bruitée (Qlow / Qhigh / P0high)

Ce document décrit la démarche et les premiers résultats obtenus pour évaluer un **EKF (robot_localization)** en présence d’une **odométrie bruitée artificiellement**.  
L’objectif est de comprendre l’impact des paramètres de covariance (**Q**, **P0**) sur les performances (RMSE, stabilité).

---

## 1) Objectif

Nous voulons observer comment le filtre EKF réagit lorsque l’odométrie est dégradée (bruit ajouté sur la pose), et comparer trois configurations :

- **Qlow_noisy** : covariance de processus faible → filtre plus “rigide”
- **Qhigh_noisy** : covariance de processus forte → filtre plus “souple”
- **P0high_noisy** : covariance initiale élevée → effet sur la convergence au démarrage

Le but est de relier :
- **choix de covariances (Q/P0)**  
avec  
- **résultats observés (RMSE / stabilité)**

---

## 2) Environnement et données

- ROS2 **Jazzy** (Linux)
- Rosbag CARLA : `~/Téléchargements/carla_data/carla_data_0.db3`
- EKF : `robot_localization` (`ekf_node`)
- Scripts :
  - `odom_noiser.py` : publie une odométrie bruitée
  - `compare_logger.py` : enregistre un CSV (référence vs filtre)
  - `summarize_runs.py` : résume les métriques à partir des CSV

---

## 3) Méthodologie

### 3.1 Création d’une odométrie bruitée

Un node Python (`odom_noiser.py`) :

- s’abonne à : `/carla/hero/odometry`
- publie : `/carla/hero/odometry_noisy`

Le bruit ajouté est gaussien (position x/y et yaw) et la **covariance** est renseignée dans le message `Odometry` afin que le filtre puisse interpréter l’incertitude des mesures.

---

### 3.2 Pipeline expérimental (1 run)

Pour chaque configuration EKF :

1. Lecture du rosbag CARLA  
2. Lancement du node de bruit (`odom_noiser.py`)  
3. Lancement de l’EKF avec un YAML spécifique (basé sur `odometry_noisy`)  
4. Lancement du logger (`compare_logger.py`) qui écrit un CSV contenant des paires synchronisées :
   - référence : `/carla/hero/odometry`
   - sortie EKF : `/odometry/filtered`

Les 3 fichiers CSV générés sont :

- `run_noisy_Qlow.csv`
- `run_noisy_Qhigh.csv`
- `run_noisy_P0high.csv`

---

## 4) Résultats (tableau de synthèse)

Les métriques suivantes ont été extraites avec `summarize_runs.py`.

| Config | N (points) | Durée simulée (s) | RMSE_x (m) | RMSE_y (m) | RMSE_2D (m) | lat_mean (s) | lat_max (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Qlow_noisy | ~51 | ~2.50 | 1.681 | 2.303 | 2.851 | 0.0 | 0.0 |
| Qhigh_noisy | ~54 | ~2.65 | 5.108 | 3.310 | 6.087 | 0.0 | 0.0 |
| P0high_noisy | ~48 | ~2.35 | 0.028 | 0.033 | 0.043 | 0.0 | 0.0 |

### Interprétation
- **Qhigh_noisy** présente l’erreur la plus grande (RMSE_2D ~6 m) : un **Q trop élevé** rend le filtre plus permissif et il peut davantage suivre une mesure bruitée.
- **Qlow_noisy** est plus stable (RMSE_2D ~2.85 m) : le filtre “résiste” mieux au bruit car il fait plus confiance au modèle.
- **P0high_noisy** donne une erreur très faible (~4 cm) sur cette fenêtre courte, mais ce résultat doit être confirmé sur des runs plus longs (P0 influence surtout la phase de démarrage).

---

## 5) Discussion

### 5.1 Qlow vs Qhigh
On observe une différence nette entre les deux réglages :

- **Q faible** → filtre plus “rigide”, trajectoire plus stable, erreurs plus faibles
- **Q fort** → filtre plus “souple”, suit davantage les variations, et en présence de bruit cela dégrade l’estimation

Ce comportement est cohérent avec le rôle de **Q (process noise covariance)** : plus Q est grand, plus le filtre considère le modèle de mouvement incertain.

---

### 5.2 Effet de P0 (initial_estimate_covariance)
Le paramètre **P0** agit principalement sur la **convergence initiale**.  
Comme nos runs actuels couvrent seulement ~2–3 secondes de temps simulé, **P0high** peut être avantagé par l’effet “début de run”.  
Il faudra une durée plus longue pour valider la tendance.

---

### 5.3 Durée et fréquence des topics
Les runs bruités ne couvrent actuellement que **~2.3 à 2.7 secondes de temps simulé**, ce qui donne ~50 échantillons.

Ce résultat correspond à la fréquence observée en temps réel :

- `/carla/hero/odometry` ≈ 6.7 Hz  
- `/odometry/filtered` ≈ 5–6 Hz  

Donc avec un enregistrement basé sur un `timeout` réel, on obtient naturellement ~50–70 points sur une fenêtre courte de temps simulé.

> **Remarque** : Pour des métriques plus robustes, il faudra augmenter la durée d’acquisition (ex : 30–35 s en temps réel) afin de couvrir davantage de temps simulé.

---

### 5.4 Synchronisation (tolérance)
Le logger utilise une synchronisation approximative (ApproximateTimeSynchronizer).  
Il peut arriver qu’une ou deux paires soient appariées “au mauvais voisin”, ce qui se voit par un `dt` négatif ou des pics isolés.

Améliorations possibles :
- réduire la tolérance `--tol`
- filtrer les lignes où `dt < 0` dans l’analyse

---

## 6) Conclusion (état actuel)

À ce stade :

- La chaîne complète **rosbag → bruit → EKF → CSV → métriques** fonctionne.
- Les réglages de covariance (**Q**, **P0**) influencent clairement les performances.
- Les résultats sont exploitables mais encore limités par la courte durée de temps simulé.

---

## 7) Prochaines étapes recommandées

Pour renforcer l’analyse :

1. **Augmenter la durée de logging** (ex : 30–35 s réel) pour obtenir plus de temps simulé et plus de points.
2. Standardiser les runs : même bruit (seed), même durée, même tolérance.
3. Ajouter des visualisations comparatives :
   - trajectoires 2D superposées
   - erreurs temporelles (ex, ey, e2d)
4. Étendre l’étude paramétrique :
   - tester aussi R (covariance de mesure) en plus de Q
   - relier clairement les choix Q/R aux courbes (RMSE / stabilité / réactivité)

---

## Commande utilisée pour le tableau

Exemple :

```bash
python3 ~/ros2_ws/src/my_py_pkg/scripts/summarize_runs.py \
  Qlow_noisy  ~/ros2_ws/logs/run_noisy_Qlow.csv \
  Qhigh_noisy ~/ros2_ws/logs/run_noisy_Qhigh.csv \
  P0high_noisy ~/ros2_ws/logs/run_noisy_P0high.csv


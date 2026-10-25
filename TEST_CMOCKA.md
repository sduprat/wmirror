# wmirror — notes de développement et de validation

Ce document rassemble les informations utiles pour travailler sur le projet `wmirror` : logique de géométrie, tests CMocka, commandes série, et visualisation 3D de la zone atteignable.

## 1) Objectif du projet

Le miroir doit suivre le soleil en calculant une normale de réflexion à partir du soleil et d’une cible de pièce. La logique est divisée entre :

- la géométrie et les conversions angle / vecteur dans `wmirror_logic.c` / `wmirror_logic.h`
- la commande des servos dans `wmirror.ino`
- les validations de logique dans `tests/cmocka_wmirror/`
- un outil visuel 3D dans `web3d/`

Le but est de garantir que la cible calculée reste dans les limites mécaniques des servos, sans mouvements brusques.

## 2) Points clés de la logique

### 2.1 Normal de réflexion

La normale est calculée à partir :

- de l’azimut/altitude du soleil
- de l’azimut/altitude de la cible

Le calcul se fait via la moyenne des vecteurs incident et réfléchi, puis conversion sphérique pour obtenir `norm_azimuth` et `norm_altitude`.

### 2.2 Conversion vers servo

La commande des servos est calculée avec :

- `H = 270 - norm_azimuth`
- `V = 90 - norm_altitude`

Ensuite, on applique des bornes mécaniques :

- `H` dans `[0, 180]`
- `V` dans `[0, 90]`

C’est ce point qui explique qu’une cible “valides” en géométrie peut être impossible à atteindre physiquement.

### 2.3 Limites mécaniques

Le système ne doit pas seulement vérifier un angle théorique, il doit vérifier l’angle réel des servos avant de décider qu’une cible est atteignable.

Le mauvais précédent comportement était :

- calculer un angle clamped
- le considérer comme validé
- ce qui faisait passer en vert des points réellement saturés à 0° ou 180°

La logique correcte est maintenant :

- calculer l’angle brut
- vérifier le brut dans les bornes
- afficher l’angle réel clamped
- marquer `reachable` selon le brut, pas selon la valeur forcée

## 3) Boucles de contrôle

Le firmware a été séparé en deux queues de traitement :

### 3.1 Boucle lente de suivi du soleil

- intervalle : environ 30 s
- calcule la position du soleil
- recalcule la normale et la cible de miroir
- mets à jour la cible des servos

### 3.2 Boucle rapide d’asservissement servo

- intervalle : ~30 ms
- applique le lissage des servos
- évite les changements trop brusques
- utilise un pas limité : `SERVO_STEP_H`, `SERVO_STEP_V`

Cela permet d’avoir un suivi stable du soleil sans “saccade” mécanique.

## 4) Commandes série

Le système accepte des commandes série en mode débug et réglage manuel.

Commandes principales :

- `A` : +5° sur l’azimut
- `E` : -5° sur l’azimut
- `Z` : +4° sur l’altitude
- `S` : -5° sur l’altitude
- `H` : retour à la position initiale
- `P` : activation du mode automatique
- `O` : arrêt du mode automatique
- `T1` / `T2` / `T3` : points de test
- `?` : affiche les valeurs géo et la normale
- `LIST` : affiche la liste des commandes
- `R` : redo de la dernière commande

### Exemple

```text
A
A
Z
S
T2
T3
?
R
```

## 5) Validation CMocka

Le projet contient un premier banc de test CMocka dans `tests/cmocka_first/` ainsi qu’un banc `cmocka_wmirror/` dédié à la logique du miroir.

### 5.1 Construire le test

```bash
cd /Users/maison/WORK/wmirror/tests/cmocka_first
export PATH="/Users/admin/Tools/cmake-3.30.0-macos-universal/CMake.app/Contents/bin:$PATH"
cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

### 5.2 Cas couverts

- normalisation des angles sur 360°
- limites de servo
- bornes de positions
- calcul de la normale
- cas limites autour de 0° / 360°
- valeurs géométriques stables en bord de domaine

## 6) Visualisation 3D de la zone atteignable

Un outil visuel est présent dans `web3d/` pour valider les zones atteignables sans matériel.

### 6.1 Lancer le visualiseur

```bash
cd /Users/maison/WORK/wmirror/web3d
python3 -m http.server 8000
```

Puis ouvrir :

```text
http://localhost:8000
```

### 6.2 Ce que l’on voit

- points gris foncé au sol : zone de scan globale
- points jaunes : case considérée comme atteignable au plafond
- points rouges : case non atteignable
- vecteurs de couleur :
  - bleu = direction du soleil
  - jaune = cible
  - vert = normale
- bouton `Rotation: on/off` pour inspecter la scène sans “tourniquet”

Le visualiseur est utile pour vérifier la cohérence du repère de la pièce et la présence/absence de zones mécaniquement atteignables.

## 7) Fichiers importants

- `wmirror.ino` : firmware principal
- `wmirror_logic.c` : géométrie
- `wmirror_logic.h` : API partagée
- `tests/cmocka_wmirror/test_wmirror_logic.c` : validation logique
- `web3d/index.html` : interface de debug 3D
- `web3d/logic.js` : logique JS reproduisant la géométrie
- `web3d/app.js` : rendu visuel et interaction

## 8) À retenir

Le cœur du projet n’est pas seulement le calcul de la position du soleil, mais surtout la vérification que la cible demandée reste dans les bornes mécaniques et le lissage des déplacements pour conserver un comportement fluide.

La logique de géométrie est stable ; les réglages finaux viennent ensuite du calibrage miroir / servos / pièce réelle.

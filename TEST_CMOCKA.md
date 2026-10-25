# Test CMocka pour le projet `wmirror`

Ce document explique comment lancer un premier test CMocka sur macOS, sans Homebrew, en gardant tout dans un dossier local du projet.

## 1) Prérequis

On a déjà vérifié que :
- le système est macOS Intel (`x86_64`)
- Homebrew ne fonctionne pas sur cette machine
- `cmake` a été installé localement
- CMocka a été compilé et installé localement

Le but est d’avoir un environnement de test portable et reproductible, sans dépendre d’un package manager global.

## 2) Structure recommandée

À la racine du projet :

```text
wmirror/
├── install/
│   ├── cmake-.../
│   └── cmocka-2.0.2/
├── tests/
│   └── cmocka_first/
├── wmirror.ino
└── ...
```

## 3) Vérifier l’emplacement de CMake

Si CMake a été installé dans un dossier local :

```bash
ls /Users/admin/Tools/cmake-3.30.0-macos-universal/CMake.app/Contents/bin
```

Tu dois voir un binaire `cmake`.

## 4) Vérifier l’emplacement de CMocka

Le package CMocka installé est à l’emplacement :

```bash
ls /Users/admin/Tools/cmocka-2.0.2/build/install
```

Tu dois voir au moins :
- `include/`
- `lib/`

## 5) Construire le projet de test CMocka

Dans le dossier du test :

```bash
cd /Users/maison/WORK/wmirror/tests/cmocka_first
```

Ensuite, ajoute le binaire CMake au `PATH` :

```bash
export PATH="/Users/admin/Tools/cmake-3.30.0-macos-universal/CMake.app/Contents/bin:$PATH"
```

Puis configure le projet :

```bash
cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build
```

Compiler :

```bash
cmake --build build
```

Lancer les tests :

```bash
ctest --test-dir build --output-on-failure
```

## 6) Exemple de test minimal déjà validé

Le projet de test contient un fichier `test_basic.c` avec des tests simples sur :
- normalisation d’un angle autour de 360°
- borne de l’altitude
- borne d’un entier servo
- vérification d’une valeur finie

Le fichier `CMakeLists.txt` pointe explicitement vers l’installation locale de CMocka :

```cmake
set(CMOCKA_ROOT "/Users/admin/Tools/cmocka-2.0.2/build/install")
```

C’est une solution simple pour un environnement sans package manager global.

## 7) Commande unique pour tout lancer

Tu peux lancer en une ligne :

```bash
export PATH="/Users/admin/Tools/cmake-3.30.0-macos-universal/CMake.app/Contents/bin:$PATH" && \
cd /Users/maison/WORK/wmirror/tests/cmocka_first && \
cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build && \
cmake --build build && \
ctest --test-dir build --output-on-failure
```

## 8) Résultat attendu

Le test doit passer avec un message du type :

```text
100% tests passed, 0 tests failed out of 1
```

## 9) À quoi sert ce test

Cette première étape permet de valider le framework de test sans sortir le matériel :
- CMake fonctionne
- CMocka fonctionne
- les tests de bornes et de logique pure peuvent être exécutés sur machine
- on a une base de non-régression avant d’évaluer le comportement sur le vrai système

## 10) Prochaine étape prévue

Ensuite, on créera des tests plus spécifiques à `wmirror` :
- normalisation des angles azimut
- bornes de `positionH` et `positionV`
- cas limite autour de 0° / 360°
- cas d’altitude proche de 0° ou 90°
- cas où la cible est au soleil / opposée au soleil

Ces tests permettront de valider la logique sans manipuler le matériel réel.

# Commandes de mise au point série pour `wmirror`

Ce document décrit le mode de réglage du système sans télécommande IR. Il est utilisé pour la mise au point et le débogage en laboratoire.

## Commandes série disponibles

Sur le moniteur série Arduino, tu peux envoyer les commandes suivantes :

- `A` : +5° sur l’azimut
- `E` : -5° sur l’azimut
- `Z` : +4° sur l’altitude
- `S` : -5° sur l’altitude
- `H` : retour à la position d’origine des servos
- `P` : active le mode automatique
- `O` : stoppe le mode automatique
- `T1` : test de continuité, cible `az=179`, `alt=99`
- `T2` : cible `az=0`, `alt=47`
- `T3` : cible égale au soleil courant
- `?` : affiche les valeurs géographiques et la normale calculée

## Exemple de séquence de test

```text
A
A
Z
S
T2
T3
T1
?
```

Ce mode permet de faire des réglages très rapides en AZERTY, sans avoir à utiliser la télécommande.

## Remarque importante sur `T1`

`T1` est un test de bord de domaine, pas un réglage de fonctionnement normal. `alt=99` est un cas extrême utilisé pour vérifier la continuité du calcul autour des limites physiques du système. Il ne représente pas un point “réel” du ciel dans le cadre d’un usage nominal.

## Remarque sur les mesures

Dans les logs série, on observe une évolution continue de la normale quand la cible est déplacée progressivement, ce qui est cohérent avec un calcul stable. Les valeurs ne sautent pas brutalement tant que le point cible reste dans le domaine exploitable.

## Utilisation recommandée

Pour la mise au point du système, la procédure la plus simple est :

1. démarrer le système
2. envoyer `?` pour lire la position initiale
3. utiliser `A`, `E`, `Z`, `S` pour ajuster la cible
4. valider la normale calculée et les positions des servos
5. utiliser `T2` ou `T3` pour aller sur des points de référence connus

## Remarque technique

Le code Arduino garde la logique de calcul dans `wmirror_logic.c` et délègue le pilotage des servos et la commande série dans `wmirror.ino`.

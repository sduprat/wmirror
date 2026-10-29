# 🪞 wmirror - Miroir orientable intelligent

Un projet low-tech pour orienter automatiquement un miroir afin de rediriger la lumière solaire ou un faisceau lumineux vers une direction souhaitée dans une pièce.

## 📋 À propos

**wmirror** est un système DIY permettant de contrôler précisément l'orientation d'un miroir à l'aide de deux servomoteurs. Contrairement à un simple suivi solaire, ce projet permet de **réfléchir la lumière vers un point spécifique** dans une pièce (canapé, table à manger, plan de travail, etc.).

### Fonctionnalités clés

| Fonction | Description |
|----------|-------------|
| **Suivi solaire intelligent** | Le miroir suit le soleil pour maintenir la réflexion vers votre cible |
| **Ciblage de pièce** | Définissez des points spécifiques dans votre pièce (coordonnées X,Y) |
| **Présélections** | Accès rapide à des positions prédéfinies (Salon, Salle à manger, Cuisine) |
| **Contrôle manuel** | Ajustement précis via télécommande IR ou série |
| **Mode automatique** | Suivi continu avec calculs en temps réel |
| **Lissage des mouvements** | Déplacements fluides sans à-coups |
| **Visualisation 3D** | Outil web pour valider la zone atteignable |

## 📦 Matériel requis

### Composants électroniques
- 1 × Arduino Uno (ou compatible)
- 2 × Servomoteurs (testé avec SG90 ou équivalent)
- 1 × Module RTC DS3231 (horloge temps réel)
- 1 × Récepteur infrarouge (ex: VS1838B)
- 1 × Télécommande IR standard
- 1 × Miroir (taille adaptée à vos servomoteurs)
- Câbles de connexion (mâle-femelle)
- Alimentation stable 5V (2A recommandé pour les servos)

### Outillage recommandé
- Fer à souder et étain
- Pince à dénuder
- Support pour miroir
- Imprimante 3D (optionnelle, pour le boîtier)

## 🔧 Montage

### Schéma de câblage

```
Arduino Uno
├── Pin 9   → Servomoteur Vertical (PWM) - contrôle l'inclinaison
├── Pin 10  → Servomoteur Horizontal (PWM) - contrôle la rotation
├── Pin 7   → Récepteur IR (DATA)
├── Pin A4/A5 → RTC DS3231 (SDA/SCL)
├── 5V      → Alimentation servos, récepteur IR, RTC
└── GND     → Masse commune
```

### Instructions de montage
1. **Fixation des servomoteurs** : Montez les deux servomoteurs de manière à permettre un mouvement sur deux axes (vertical et horizontal)
2. **Fixation du miroir** : Attachez solidement le miroir au servomoteur vertical
3. **Câblage électrique** : Suivez le schéma ci-dessus
4. **Module RTC** : Connectez le DS3231 sur les broches I2C (A4=SDA, A5=SCL)
5. **Alimentation** : Utilisez une source externe 5V pour les servos (ne pas alimenter uniquement via Arduino)

> ⚠️ **Attention** : Les servomoteurs consomment beaucoup de courant. Une alimentation externe est **fortement recommandée** pour éviter de surcharger l'Arduino.

## 🚀 Installation

### Prérequis logiciels
- [Arduino IDE](https://www.arduino.cc/en/software) ou [PlatformIO](https://platformio.org/)
- Bibliothèques Arduino requises :
  - TimeLib
  - SunPosition (par Andrew Moore)
  - Servo (inclus avec Arduino IDE)
  - IRremote (par shirriff)
  - RTClib (par Adafruit)
  - Wire (inclus avec Arduino IDE)

### Installation avec PlatformIO
1. Cloner ce dépôt
2. Ouvrir le projet avec PlatformIO
3. Attendre l'installation automatique des dépendances
4. Sélectionner la carte Arduino Uno
5. Compiler et téléverser

### Installation avec Arduino IDE
1. Installer les bibliothèques via le gestionnaire :
   - `Time` par Michael Margolis
   - `SunPosition` par Andrew Moore
   - `IRremote` par shirriff
   - `RTClib` par Adafruit
2. Ouvrir le fichier `wmirror.ino`
3. Sélectionner le bon port série et la carte Arduino Uno
4. Téléverser le code

### Configuration initiale du RTC
À la première utilisation, le RTC sera initialisé avec l'heure de compilation. Pour une heure précise :
1. Téléversez le code avec l'heure correcte dans `setTime()`
2. Ou utilisez un outil de configuration RTC

## ⚙️ Configuration

### Paramètres géographiques
Modifiez ces constantes pour adapter à votre localisation :

```cpp
// Dans wmirror.ino
#define LATITUDE 43.6    // Latitude en degrés décimaux (ex: 43.6 pour Toulouse)
#define LONGITUDE 1.8    // Longitude en degrés décimaux
```

### Configuration de la pièce
Le système utilise un repère 3D pour cibler des points dans votre pièce :

```cpp
// Hauteur du plafond (en mètres)
#define ROOM_CEILING_Z_M 2.5

// Points prédéfinis (coordonnées X,Y en mètres depuis le miroir)
#define ROOM_POINT_SALON_X_M (1.5f)      // 1.5m devant
#define ROOM_POINT_SALON_Y_M (-0.5f)     // 0.5m à gauche
#define ROOM_POINT_SALLE_A_MANGER_X_M (3.5f)
#define ROOM_POINT_SALLE_A_MANGER_Y_M (0.5f)
#define ROOM_POINT_CUISINE_X_M (6.0f)
#define ROOM_POINT_CUISINE_Y_M (3.0f)
```

> 💡 **Astuce** : Utilisez la commande `POS x y` via le port série pour tester différentes positions avant de modifier le code.

### Codes télécommande IR
Les codes sont préconfigurés. Si votre télécommande utilise des codes différents :

1. Ouvrez le moniteur série (9600 bauds)
2. Appuyez sur les touches de votre télécommande
3. Notez les codes affichés (format `0xXXXX`)
4. Mettez à jour les définitions dans `wmirror.ino`

```cpp
// Exemple pour une télécommande standard
#define CODE_HAUT    0x0058    // Flèche haut
#define CODE_BAS     0x0059    // Flèche bas
#define CODE_GAUCHE  0x005A    // Flèche gauche
#define CODE_DROITE  0x005B    // Flèche droite
#define CODE_HOME    0x0054    // Bouton Home
#define CODE_PLAY    0x002C    // Bouton Play (mode auto)
#define CODE_STOP    0x0031    // Bouton Stop
// Boutons numériques pour les présélections
#define CODE_1       0x0001    // T1: Test 1
#define CODE_2       0x0002    // T2: Test 2
#define CODE_3       0x0003    // T3: Suivre le soleil
#define CODE_4       0x0004    // Salon
#define CODE_5       0x0005    // Salle à manger
#define CODE_6       0x0006    // Cuisine
```

### Lissage des servos
Ajustez ces valeurs pour modifier la fluidité des mouvements :

```cpp
// Pas de déplacement maximum à chaque itération
#define SERVO_STEP_V 2    // Degrés par étape (vertical)
#define SERVO_STEP_H 2    // Degrés par étape (horizontal)

// Intervalles de mise à jour
#define SUN_UPDATE_INTERVAL_MS (30000)    // 30 secondes pour le soleil
#define SERVO_REFRESH_INTERVAL_MS (30)   // 30 ms pour les servos
```

## 🎮 Utilisation

### Télécommande IR

| Bouton | Action |
|--------|--------|
| **↑ Haut** | Déplace la cible vers l'avant (Y-0.20m) |
| **↓ Bas** | Déplace la cible vers l'arrière (Y+0.20m) |
| **← Gauche** | Déplace la cible vers la droite (X+0.20m) |
| **→ Droite** | Déplace la cible vers la gauche (X-0.20m) |
| **HOME** | Retour à la position initiale |
| **PLAY** | Active le mode automatique |
| **STOP** | Désactive le mode automatique |
| **1** | Test 1: Azimut=179°, Altitude=99° |
| **2** | Test 2: Azimut=0°, Altitude=47° |
| **3** | Test 3: Cible = position du soleil |
| **4** | Présélection: Salon |
| **5** | Présélection: Salle à manger |
| **6** | Présélection: Cuisine |

### Commandes série (9600 bauds)

Le système accepte des commandes texte via le port série pour un contrôle précis :

#### Commandes de déplacement
| Commande | Action |
|----------|--------|
| `A` | +5° sur l'azimut |
| `E` | -5° sur l'azimut |
| `Z` | +4° sur l'altitude |
| `S` | -5° sur l'altitude |

#### Commandes de position pièce
| Commande | Action |
|----------|--------|
| `X+` | +0.20m sur X |
| `X-` | -0.20m sur X |
| `Y+` | +0.20m sur Y |
| `Y-` | -0.20m sur Y |
| `POS x y` | Positionne la cible aux coordonnées (x,y) |

#### Présélections
| Commande | Action |
|----------|--------|
| `SALON` | Cible le point Salon |
| `SALLE` | Cible le point Salle à manger |
| `CUISINE` | Cible le point Cuisine |
| `LIST` | Affiche la liste des présélections |

#### Commandes système
| Commande | Action |
|----------|--------|
| `HOME` | Retour à la position initiale |
| `P` | Active le mode automatique |
| `O` | Désactive le mode automatique |
| `T1` | Test 1 (continuité) |
| `T2` | Test 2 |
| `T3` | Cible = soleil |
| `?` | Affiche les valeurs géométriques actuelles |
| `R` | Répète la dernière commande |
| `?` | Affiche l'aide |

### Exemple de session série
```
wmirror ready
Commands (AZERTY / room):
  A  : +5 on azimuth
  E  : -5 on azimuth
  ...

SALON
PRESET SALON => ROOM target: X=-1.500 m  Y=-0.500 m  => AZ=180.00 deg  EL=45.00 deg

Y+
ROOM target: X=-1.500 m  Y=-0.300 m  => AZ=180.00 deg  EL=41.11 deg

?
2024/5/15 14:30:45
SAzimuth: 195.2560 degrees  SAltitude: 45.6789 degrees
TAzimuth: 180.0000 degrees  TAltitude: 41.1100 degrees
NAzimuth: 187.6280 degrees  NAltitude: 43.3944 degrees
```

### Mode automatique
Lorsque le mode automatique est activé :
- Le système calcule la position du soleil toutes les 30 secondes
- La normale du miroir est recalculée pour maintenir la réflexion vers la cible
- Les servos se déplacent progressivement vers la nouvelle position
- La LED intégrée s'allume pour indiquer le mode actif

## 📊 Comprendre les angles et coordonnées

### Système de coordonnées

#### Coordonnées sphériques (angles)
- **Azimut** : Angle horizontal par rapport au Nord (0° = Nord, 90° = Est, 180° = Sud, 270° = Ouest)
- **Altitude** : Angle vertical par rapport à l'horizon (0° = horizon, 90° = zénith)

#### Coordonnées cartésiennes (pièce)
- **X** : Distance Est-Ouest (positif = Est)
- **Y** : Distance Nord-Sud (positif = Sud)
- **Z** : Hauteur (par défaut = hauteur du plafond = 2.5m)

### Principe de réflexion
Le système calcule la **normale** du miroir comme la bissectrice entre :
1. Le vecteur **soleil → miroir** (incident)
2. Le vecteur **miroir → cible** (réfléchi)

La normale est ensuite convertie en angles de servo :
- `PositionH = 270° - norm_azimuth`
- `PositionV = 90° - norm_altitude`

## 🎯 Visualisation 3D

Un outil de visualisation interactif est disponible pour valider la zone atteignable :

1. Lancez un serveur web local :
   ```bash
   cd web3d
   python3 -m http.server 8000
   ```

2. Ouvrez votre navigateur à l'adresse :
   ```
   http://localhost:8000
   ```

3. Utilisez l'interface pour :
   - Visualiser la zone de couverture du miroir
   - Voir les vecteurs soleil/cible/normale
   - Tester différentes configurations

### Ce que vous voyez
- **Points gris** : Zone de scan globale au sol
- **Points jaunes** : Positions atteignables au plafond
- **Points rouges** : Positions non atteignables
- **Vecteurs** :
  - 🔵 Bleu = direction du soleil
  - 🟡 Jaune = cible
  - 🟢 Vert = normale du miroir

## 🔍 Dépannage

### Problèmes courants

| Problème | Solution |
|----------|----------|
| Les servos ne bougent pas | Vérifiez le câblage et l'alimentation externe |
| La télécommande IR ne répond pas | Vérifiez le récepteur IR (LED clignote à la réception) |
| Mouvements saccadés | Augmentez `SERVO_STEP_V/H` ou vérifiez l'alimentation |
| Heure incorrecte | Vérifiez le module RTC et sa batterie |
| Cible non atteignable | La position est peut-être hors des limites mécaniques |
| Valeurs d'angles incohérentes | Vérifiez la configuration géographique |

### Vérification du câblage
1. Testez chaque servo individuellement avec un code simple
2. Vérifiez que le récepteur IR reçoit bien les signaux
3. Assurez-vous que la masse est bien commune à tous les composants
4. Vérifiez les connexions I2C du RTC (SDA=A4, SCL=A5)

### Diagnostics via port série
Ouvrez le moniteur série (9600 bauds) pour voir :
- Les codes IR reçus
- Les positions actuelles des servos
- Les angles soleil/cible/normale
- Les coordonnées de la cible dans la pièce

## 🧪 Tests et validation

Le projet inclut une suite de tests complets pour valider la logique géométrique :

### Exécuter les tests
```bash
# Installer les dépendances (Ubuntu/Debian)
sudo apt-get install libcmocka-dev gcc make

# Compiler et exécuter les tests
make test
```

### Tests inclus
- Normalisation des angles sur 360°
- Limites des servos (clamping)
- Calcul de la normale de réflexion
- Conversions sphérique ↔ cartésien
- Cas limites et valeurs aux bornes

## 🌍 Personnalisation avancée

### Ajouter une présélection
1. Définissez les coordonnées dans `wmirror.ino` :
   ```cpp
   #define ROOM_POINT_MON_BUREAU_X_M (2.0f)
   #define ROOM_POINT_MON_BUREAU_Y_M (-1.0f)
   ```

2. Ajoutez un code IR ou une commande série :
   ```cpp
   if (upper == "BUREAU") {
       goToRoomPreset(ROOM_POINT_MON_BUREAU_X_M, ROOM_POINT_MON_BUREAU_Y_M, F("BUREAU"));
       return;
   }
   ```

### Modifier les limites mécaniques
Les limites par défaut des servos peuvent être ajustées :

```cpp
// Dans wmirror_logic.c
double normalize_servo_h(double value) {
    return clamp_double(value, 0.0, 180.0);  // Limites horizontales
}

double normalize_servo_v(double value) {
    return clamp_double(value, 0.0, 90.0);   // Limites verticales
}
```

### Changer la fréquence de suivi
```cpp
#define SUN_UPDATE_INTERVAL_MS (30000)    // Mise à jour soleil (30s)
#define SERVO_REFRESH_INTERVAL_MS (30)   // Rafraîchissement servos (30ms)
```

## 💡 Idées d'amélioration

- [ ] Ajouter un écran LCD/OLED pour afficher l'état
- [ ] Intégrer un capteur de lumière pour calibration automatique
- [ ] Ajouter un module WiFi/Bluetooth pour contrôle mobile
- [ ] Créer une interface web pour le contrôle à distance
- [ ] Implémenter un système de calibration automatique
- [ ] Ajouter des fins de course physiques pour la sécurité
- [ ] Intégrer une prédiction météo pour optimiser le suivi
- [ ] Ajouter un mode "balayage" pour couvrir une zone

## 📚 Ressources

### Bibliothèques utilisées
- [SunPosition](https://github.com/andrewmoore/sunposition) - Calcul de la position du soleil
- [IRremote](https://github.com/z3t0/Arduino-IRremote) - Gestion des télécommandes IR
- [RTClib](https://github.com/adafruit/RTClib) - Gestion du RTC
- [CMocka](https://cmocka.org/) - Framework de tests unitaires

### Outils
- [Wokwi](https://wokwi.com/) - Simulateur Arduino en ligne
- [PlatformIO](https://platformio.org/) - Environnement de développement
- [GitHub Actions](https://github.com/features/actions) - CI/CD

## 🤝 Contribution

Ce projet est open source et les contributions sont les bienvenues !

### Comment contribuer
- **Signaler un bug** : Ouvrez une issue avec une description claire
- **Proposer une amélioration** : Ouvrez une issue pour discuter de la feature
- **Soumettre du code** : Fork le projet, créez une branche, et envoyez une PR
- **Améliorer la documentation** : Les corrections et ajouts sont toujours utiles

### Structure du code
```
wmirror/
├── wmirror.ino           # Firmware principal
├── wmirror_logic.c/h     # Logique géométrique (C)
├── Makefile              # Build des tests
├── tests/                # Tests unitaires CMocka
│   ├── test_basic.c
│   ├── test_wmirror_logic.c
│   └── test_limits_bounds.c
├── web3d/                # Visualiseur 3D
│   ├── index.html
│   ├── app.js
│   └── logic.js
├── .github/workflows/    # CI/CD
└── diagram.json          # Schéma Wokwi
```

## 📜 Licence

Ce projet est sous licence [MIT](LICENSE). Vous êtes libre d'utiliser, modifier et distribuer ce code.

---

*Projet personnel low-tech par Stéphane Duprat*
*Version : feature/rework*

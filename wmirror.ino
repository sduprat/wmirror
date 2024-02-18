#include <TimeLib.h>
#include <SunPosition.h>
#include <Servo.h>
#include <math.h>
#include <Servo.h>
#include <IRremote.h>

// Pins de réception infrarouge
const int RECV_PIN = 7;

// Objets Servo
Servo servoV;
Servo servoH;

// Positions initiales des servomoteurs
#define POSITION_INITIALE_V 30
#define POSITION_INITIALE_H 90

// Variables de position des servomoteurs
int positionV = POSITION_INITIALE_V;
int positionH = POSITION_INITIALE_H;

// Codes de télécommande

#define DECODE_NEC          // for arduino rc and wokwi

#ifdef DECODE_NEC
#define CODE_HAUT 0x0002    // Code de la flèche haut
#define CODE_BAS 0x0098     // Code de la flèche bas
#define CODE_GAUCHE 0x00E0  // Code de la flèche gauche
#define CODE_DROITE 0x0090  // Code de la flèche droite
#define CODE_HOME 0x00E2    // Code de la touche home (test)
#define CODE_PLAY 0x00A8    // Code de la touche play
#define CODE_STOP 0x0068    // Code de la touche play
#else
#define CODE_HAUT 0x0058    // Code de la flèche haut
#define CODE_BAS 0x0059     // Code de la flèche bas
#define CODE_GAUCHE 0x005A  // Code de la flèche gauche
#define CODE_DROITE 0x005B  // Code de la flèche droite
#define CODE_HOME 0x0054    // Code de la touche home
#define CODE_PLAY 0x002C    // Code de la touche play
#define CODE_STOP 0x0031    // Code de la touche play
#endif

IRrecv irrecv(RECV_PIN);
decode_results results;

// 1 si play
int mode_play = 1;
unsigned long current_millis =0;
unsigned long last_millis =0;

 // soit toutes les 4" pour un degre
#define PAS_MILLIS (4*60)

void updateServoPositions() {
  // Contraintes pour la positionV
  positionH = constrain(positionH, -180, 180);

  // Contraintes pour la positionH
  positionV = constrain(positionV, 0, 90);

  // Mettre à jour la position des servomoteurs
  servoV.write(positionV);
  servoH.write(positionH);

  // Afficher la position des servomoteurs sur la sortie série
  Serial.println(mode_play);
  Serial.println(mode_play);
  Serial.println(positionV);
  Serial.print("Position H : ");
  Serial.println(positionH);
}
void setup() {
  // Initialisation des servomoteurs
  servoV.attach(9);
  servoH.attach(10);

  updateServoPositions();

  // Initialisation de la réception infrarouge
  irrecv.enableIRIn();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //mode_play = 0;
  
  // Initialisation de la communication série
  Serial.begin(9600);
}

void loop() {
  if (irrecv.decode()) {
    unsigned long code = irrecv.decodedIRData.command;

    // Afficher le code reçu en hexadécimal sur la sortie série
    Serial.print("Code reçu : 0x");
    Serial.println(code, HEX);

    // Vérification du code reçu
    switch (code) {
      case CODE_HAUT:
        // Modifier la position verticale du servomoteur
        positionV += 1;
        break;
      case CODE_BAS:
        // Modifier la position verticale du servomoteur
        positionV -= 1;
        break;
      case CODE_GAUCHE:
        // Modifier la position horizontale du servomoteur
        positionH += 1;
        break;
      case CODE_DROITE:
        // Modifier la position horizontale du servomoteur
        positionH -= 1;
        break;
      case CODE_HOME:
        // Revenir à la position initiale des servomoteurs
        positionV = POSITION_INITIALE_V;
        positionH = POSITION_INITIALE_H;
        break;
      case CODE_PLAY:
        // Faire quelque chose pour le code play
        mode_play = 1;
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        last_millis = millis();
        break;
      case CODE_STOP:
        // Faire quelque chose pour le code play
        mode_play = 0;
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        break;
    }


    updateServoPositions();

    irrecv.resume(); // Réactiver la réception infrarouge
  }

  if (mode_play) {
    current_millis = millis();
    if ((current_millis - last_millis)/1000 >= PAS_MILLIS) {
      Serial.println(current_millis - last_millis);
      positionH++;
      last_millis = current_millis;
      updateServoPositions();

    }
  
  delay(100);                       // wait for a 100ms

  }

}

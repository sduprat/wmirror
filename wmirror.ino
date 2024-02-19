#include <TimeLib.h>
#include <SunPosition.h>
#include <Servo.h>
#include <math.h>
#include <Servo.h>
#include <IRremote.h>
#include <Wire.h>
#include <RTClib.h>

//to comment in real
#define WOKWI

// RTC
#ifdef WOKWI
RTC_DS1307 rtc;
#else
RTC_DS3231 rtc;
#endif


// Servo Config
Servo servoV;
Servo servoH;

// Positions initiales des servomoteurs
#define POSITION_INITIALE_V 30
#define POSITION_INITIALE_H 90

// Variables de position des servomoteurs
int positionV = POSITION_INITIALE_V;
int positionH = POSITION_INITIALE_H;

// IR Config
// Pins de réception infrarouge
const int RECV_PIN = 7;

// Codes de télécommande

//#define DECODE_NEC          // for arduino rc and wokwi

#ifdef WOKWI
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



// Geo config
#define LATITUDE 43.6 // Latitude de Lavaur en degrés décimaux
#define LONGITUDE 1.8 // Longitude de Lavaur en degrés décimaux

float norm_azimuth = 0.0; // Polaris est située approximativement au pôle nord céleste, son azimut est donc 0 degrés
float norm_altitude = 90.0 - LATITUDE; // L'élévation de Polaris est égale à 90 degrés moins la latitude de l'observateur

float target_azimuth = 180;
float target_altitude = (180 - 55);

float sun_azimuth;
float sun_altitude;

// Time Config

// 1 si play
int mode_play = 1;
unsigned long current_millis =0;
unsigned long last_millis =0;

 // update pos every ...
#define PAS_MILLIS (30)



void updateServoPositions() {
  // Contraintes pour la positionV
  positionH = constrain(positionH, -180, 180);

  // Contraintes pour la positionH
  positionV = constrain(positionV, 0, 90);

  // Mettre à jour la position des servomoteurs
  servoV.write(positionV);
  servoH.write(positionH);

  // Afficher la position des servomoteurs sur la sortie série
  Serial.print("Servos H: ");
  Serial.print(positionH);
  Serial.print(" V: ");
  Serial.println(positionV);
}

float angle_normal(float angle1, float angle2) {

  float result;

  if ((angle1-angle2)>180)
    result = ((angle1 + angle2)/2+180);
    else {
      if ((angle1-angle2)<-180)
        result = ((angle1 + angle2)/2-180);
      else
        result = (angle1 + angle2)/2;}
  return result;
}

void printGeo(){

  Serial.print(year(), DEC);
  Serial.print('/');
  Serial.print(month(), DEC);
  Serial.print('/');
  Serial.print(day(), DEC);
  Serial.print(' ');
  Serial.print(hour(), DEC);
  Serial.print(':');
  Serial.print(minute(), DEC);
  Serial.print(':');
  Serial.print(second(), DEC);
  Serial.println();


  Serial.print("SAzimuth: ");
  Serial.print(sun_azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tSAltitude: ");
  Serial.print(sun_altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
  Serial.print("TAzimuth: ");
  Serial.print(target_azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tTAltitude: ");
  Serial.print(target_altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
  Serial.print("NAzimuth: ");
  Serial.print(norm_azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tNAltitude: ");
  Serial.print(norm_altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
}


//compute new norm from sun
void updateNorm() {
  Serial.print("updateNorm");

  SunPosition sun(LATITUDE, LONGITUDE, now()); // Créer un objet SunPosition pour la latitude, la longitude et l'heure actuelles
  sun_azimuth = sun.azimuth(); // Obtenir l'azimut du soleil
  sun_altitude = sun.altitude(); // Obtenir l'élévation du soleil

  // deduce norm from servo pos
  norm_azimuth  = angle_normal(sun_azimuth, target_azimuth);
  norm_altitude = angle_normal(sun_altitude, target_altitude);

  // update servo pos from norm
  positionH = 270 - norm_azimuth;
  positionV = 90 - norm_altitude ;

  printGeo();
}

//compute new target from servos
void updateTarget() {
  Serial.print("updateTarget");
  SunPosition sun(LATITUDE, LONGITUDE, now()); // Créer un objet SunPosition pour la latitude, la longitude et l'heure actuelles
  sun_azimuth = sun.azimuth(); // Obtenir l'azimut du soleil
  sun_altitude = sun.altitude(); // Obtenir l'élévation du soleil

  // deduce norm from servo pos
  norm_azimuth  = 270.0 - positionH;
  norm_altitude = 90.0 - positionV;

  // compute norm from sun and target
  target_azimuth = fmod(2.0 * norm_azimuth - sun_azimuth,360);
  target_altitude = fmod(2.0 * norm_altitude - sun_altitude,360);
  
  printGeo();
}

void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //mode_play = 0;
  
  // Initialisation de la communication série
  Serial.begin(9600);

  // Initialisation des servomoteurs
  servoV.attach(9);
  servoH.attach(10);

  // Initialisation de la réception infrarouge
  irrecv.enableIRIn();

  // RTC
  Wire.begin();
  rtc.begin();
  
  // Uncomment the following line to set the time
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Time init
  //setTime(16, 0, 0, 13, 2, 2024); // Définir l'heure et la date actuelles (heure:minute:seconde, jour:mois:année)
  setTime(rtc.now().unixtime());


  updateNorm();
  updateServoPositions();

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

    switch (code) {
      case CODE_HAUT:
      case CODE_BAS:
      case CODE_GAUCHE:
      case CODE_DROITE:
      case CODE_HOME:
        updateServoPositions();
        updateTarget();
    }

    irrecv.resume(); // Réactiver la réception infrarouge
  }

  if (mode_play) {
    current_millis = millis();
    if ((current_millis - last_millis)/1000 >= PAS_MILLIS) {
      Serial.println(current_millis - last_millis);
      positionH++;
      last_millis = current_millis;
      updateNorm();
      updateServoPositions();

    }
  
  delay(100);                       // wait for a 100ms

  }

}

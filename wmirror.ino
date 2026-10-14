#include <TimeLib.h>
#include <SunPosition.h>
#include <Servo.h>
#include <math.h>
#include <Servo.h>
#include <IRremote.h>
#include <Wire.h>
#include <RTClib.h>

#include "wmirror_logic.h"

//to comment in real
//#define WOKWI

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
#define CODE_1 0x0030    // Code de la touche play
#define CODE_2 0x0018    // Code de la touche play
#define CODE_3 0x007A    // Code de la touche play
#else
#define CODE_HAUT 0x0058    // Code de la flèche haut
#define CODE_BAS 0x0059     // Code de la flèche bas
#define CODE_GAUCHE 0x005A  // Code de la flèche gauche
#define CODE_DROITE 0x005B  // Code de la flèche droite
#define CODE_HOME 0x0054    // Code de la touche home
#define CODE_PLAY 0x002C    // Code de la touche play
#define CODE_STOP 0x0031    // Code de la touche play
#define CODE_1 0x0001    // Code de la touche play
#define CODE_2 0x0002    // Code de la touche play
#define CODE_3 0x0003    // Code de la touche play
#endif

IRrecv irrecv(RECV_PIN);
decode_results results;



// Geo config
#define LATITUDE 43.6 // Latitude de Lavaur en degrés décimaux
#define LONGITUDE 1.8 // Longitude de Lavaur en degrés décimaux

float norm_azimuth = 0.0; // Polaris est située approximativement au pôle nord céleste, son azimut est donc 0 degrés
float norm_altitude = 90.0 - LATITUDE; // L'élévation de Polaris est égale à 90 degrés moins la latitude de l'observateur

float target_azimuth = 30;
float target_altitude = 45;

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
  positionH = (int)normalize_servo_h(positionH);
  positionV = (int)normalize_servo_v(positionV);

  servoV.write(positionV);
  servoH.write(positionH);

  Serial.print("Servos H: ");
  Serial.print(positionH);
  Serial.print(" V: ");
  Serial.println(positionV);
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
  Serial.print(sun_azimuth, 4);
  Serial.print(" degrees");
  Serial.print("\tSAltitude: ");
  Serial.print(sun_altitude, 4);
  Serial.println(" degrees");
  Serial.print("TAzimuth: ");
  Serial.print(target_azimuth, 4);
  Serial.print(" degrees");
  Serial.print("\tTAltitude: ");
  Serial.print(target_altitude, 4);
  Serial.println(" degrees");
  Serial.print("NAzimuth: ");
  Serial.print(norm_azimuth, 4);
  Serial.print(" degrees");
  Serial.print("\tNAltitude: ");
  Serial.print(norm_altitude, 4);
  Serial.println(" degrees");
}

void updateNorm() {
  Serial.println("updateNorm");

  SunPosition sun(LATITUDE, LONGITUDE, now());
  sun_azimuth = sun.azimuth();
  sun_altitude = sun.altitude();

  double computed_azimuth = 0.0;
  double computed_altitude = 0.0;

  compute_norm_from_angles(sun_azimuth,
                          sun_altitude,
                          target_azimuth,
                          target_altitude,
                          &computed_azimuth,
                          &computed_altitude);

  norm_azimuth = (float)computed_azimuth;
  norm_altitude = (float)computed_altitude;

  positionH = (int)(270.0 - norm_azimuth);
  positionV = (int)(90.0 - norm_altitude);

  positionH = (int)normalize_servo_h(positionH);
  positionV = (int)normalize_servo_v(positionV);

  printGeo();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  servoV.attach(9);
  servoH.attach(10);

  irrecv.enableIRIn();

  Wire.begin();
  rtc.begin();

  setTime(rtc.now().unixtime()-3600);

  updateNorm();
  updateServoPositions();
}

void loop() {
  if (irrecv.decode()) {
    unsigned long code = irrecv.decodedIRData.command;

    Serial.print("Code reçu : 0x");
    Serial.println(code, HEX);

    switch (code) {
      case CODE_HAUT:
        target_altitude += 1;
        break;
      case CODE_BAS:
        target_altitude -= 1;
        break;
      case CODE_GAUCHE:
        target_azimuth += 1;
        break;
      case CODE_DROITE:
        target_azimuth -= 1;
        break;
      case CODE_HOME:
        positionV = POSITION_INITIALE_V;
        positionH = POSITION_INITIALE_H;
        break;
      case CODE_1:
        target_azimuth = 179;
        target_altitude = 99;
        break;
      case CODE_2:
        target_azimuth = 0;
        target_altitude = 47;
        break;
      case CODE_3:
        target_azimuth = sun_azimuth;
        target_altitude = sun_altitude;
        break;
      case CODE_PLAY:
        mode_play = 1;
        digitalWrite(LED_BUILTIN, HIGH);
        last_millis = millis();
        break;
      case CODE_STOP:
        mode_play = 0;
        digitalWrite(LED_BUILTIN, LOW);
        break;
    }

    switch (code) {
      case CODE_HAUT:
      case CODE_BAS:
      case CODE_GAUCHE:
      case CODE_DROITE:
      case CODE_HOME:
      case CODE_1:
      case CODE_2:
      case CODE_3:
        updateNorm();
        updateServoPositions();
    }

    irrecv.resume();
  }

  if (mode_play) {
    current_millis = millis();
    if ((current_millis - last_millis)/1000 >= PAS_MILLIS) {
      positionH++;
      last_millis = current_millis;
      updateNorm();
      updateServoPositions();
    }

    delay(100);
  }
}

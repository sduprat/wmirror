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

// Room target config
#define ROOM_CEILING_Z_M 2.5
float target_x_m = 0.0f;
float target_y_m = 0.0f;

// Room preset points
#define ROOM_POINT_SALON_X_M (1.5f)
#define ROOM_POINT_SALON_Y_M (-0.5f)
#define ROOM_POINT_SALLE_A_MANGER_X_M (3.5f)
#define ROOM_POINT_SALLE_A_MANGER_Y_M (0.5f)
#define ROOM_POINT_CUISINE_X_M (6.0f)
#define ROOM_POINT_CUISINE_Y_M (3.0f)

String lastCommand = "";

void rememberLastCommand(const String &command) {
  String trimmed = command;
  trimmed.trim();

  if (trimmed.length() == 0) {
    return;
  }

  String upper = trimmed;
  upper.toUpperCase();

  if (upper == "R" || upper == "?" || upper == "LIST") {
    return;
  }

  lastCommand = trimmed;
}

void redoLastCommand() {
  if (lastCommand.length() == 0) {
    Serial.println(F("No command to redo"));
    return;
  }

  Serial.print(F("REDO: "));
  Serial.println(lastCommand);
  handleSerialCommand(lastCommand);
}

void syncTargetFromRoomXY() {
  double az_deg = 0.0;
  double el_deg = 90.0;

  room_xy_to_az_el((double)target_x_m, (double)target_y_m, ROOM_CEILING_Z_M, &az_deg, &el_deg);
  target_azimuth = (float)az_deg;
  target_altitude = (float)el_deg;
}

void printRoomTarget() {
  Serial.print(F("ROOM target: X="));
  Serial.print(target_x_m, 3);
  Serial.print(F(" m  Y="));
  Serial.print(target_y_m, 3);
  Serial.print(F(" m  => AZ="));
  Serial.print(target_azimuth, 2);
  Serial.print(F(" deg  EL="));
  Serial.println(target_altitude, 2);
}

void setRoomTarget(float x_m, float y_m) {
  target_x_m = x_m;
  target_y_m = y_m;
  syncTargetFromRoomXY();
}

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

void printSerialHelp() {
  Serial.println(F("Commands (AZERTY / room):"));
  Serial.println(F("  A  : +5 on azimuth"));
  Serial.println(F("  E  : -5 on azimuth"));
  Serial.println(F("  Z  : +4 on altitude"));
  Serial.println(F("  S  : -5 on altitude"));
  Serial.println(F("  X+ : +0.20 m on X"));
  Serial.println(F("  X- : -0.20 m on X"));
  Serial.println(F("  Y+ : +0.20 m on Y"));
  Serial.println(F("  Y- : -0.20 m on Y"));
  Serial.println(F("  POS x y : set target position in room coordinates"));
  Serial.println(F("  SALON : preset point (-1.5,-0.5)"));
  Serial.println(F("  SALLE : preset point (-3.5,0.5)"));
  Serial.println(F("  CUISINE : preset point (-6.0,3.0)"));
  Serial.println(F("  LIST : show preset room points"));
  Serial.println(F("  HOME     : reset default servo positions"));
  Serial.println(F("  P        : play"));
  Serial.println(F("  O        : stop"));
  Serial.println(F("  T1       : az=179 alt=99 (continuity test)"));
  Serial.println(F("  T2       : az=0 alt=47"));
  Serial.println(F("  T3       : target = sun"));
  Serial.println(F("  R        : redo last command"));
  Serial.println(F("  ?        : print current geo values"));
}

void goToRoomPreset(float x_m, float y_m, const __FlashStringHelper *label);

void applyTargetUpdate() {
  updateNorm();
  updateServoPositions();
}

void handleSerialCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  String upper = command;
  upper.toUpperCase();

  if (upper == "R") {
    redoLastCommand();
    return;
  }

  if (upper == "?") {
    printGeo();
    return;
  }

  if (upper == "HOME") {
    positionV = POSITION_INITIALE_V;
    positionH = POSITION_INITIALE_H;
    target_x_m = 0.0f;
    target_y_m = 0.0f;
    syncTargetFromRoomXY();
    applyTargetUpdate();
    Serial.println(F("HOME"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "PLAY") {
    mode_play = 1;
    digitalWrite(LED_BUILTIN, HIGH);
    last_millis = millis();
    Serial.println(F("PLAY"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "STOP") {
    mode_play = 0;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(F("STOP"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "T1") {
    target_azimuth = 179.0;
    target_altitude = 99.0;
    target_x_m = 0.0f;
    target_y_m = 0.0f;
    applyTargetUpdate();
    Serial.println(F("TEST1"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "T2") {
    target_azimuth = 0.0;
    target_altitude = 47.0;
    target_x_m = 0.0f;
    target_y_m = 0.0f;
    applyTargetUpdate();
    Serial.println(F("T2"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "T3") {
    target_azimuth = sun_azimuth;
    target_altitude = sun_altitude;
    target_x_m = 0.0f;
    target_y_m = 0.0f;
    applyTargetUpdate();
    Serial.println(F("TEST3"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "A") {
    target_azimuth += 5.0;
    applyTargetUpdate();
    Serial.print(F("AZ="));
    Serial.println(target_azimuth, 2);
    rememberLastCommand(command);
    return;
  }

  if (upper == "E") {
    target_azimuth -= 5.0;
    applyTargetUpdate();
    Serial.print(F("AZ="));
    Serial.println(target_azimuth, 2);
    rememberLastCommand(command);
    return;
  }

  if (upper == "Z") {
    target_altitude += 4.0;
    applyTargetUpdate();
    Serial.print(F("EL="));
    Serial.println(target_altitude, 2);
    rememberLastCommand(command);
    return;
  }

  if (upper == "S") {
    target_altitude -= 5.0;
    applyTargetUpdate();
    Serial.print(F("EL="));
    Serial.println(target_altitude, 2);
    rememberLastCommand(command);
    return;
  }

  if (upper == "X+") {
    target_x_m += 0.20f;
    syncTargetFromRoomXY();
    applyTargetUpdate();
    printRoomTarget();
    rememberLastCommand(command);
    return;
  }

  if (upper == "X-") {
    target_x_m -= 0.20f;
    syncTargetFromRoomXY();
    applyTargetUpdate();
    printRoomTarget();
    rememberLastCommand(command);
    return;
  }

  if (upper == "Y+") {
    target_y_m += 0.20f;
    syncTargetFromRoomXY();
    applyTargetUpdate();
    printRoomTarget();
    rememberLastCommand(command);
    return;
  }

  if (upper == "Y-") {
    target_y_m -= 0.20f;
    syncTargetFromRoomXY();
    applyTargetUpdate();
    printRoomTarget();
    rememberLastCommand(command);
    return;
  }

  if (upper.startsWith("POS ")) {
    String rest = command.substring(4);
    int separator = rest.indexOf(' ');
    if (separator > 0) {
      String xStr = rest.substring(0, separator);
      String yStr = rest.substring(separator + 1);
      target_x_m = xStr.toFloat();
      target_y_m = yStr.toFloat();
      syncTargetFromRoomXY();
      applyTargetUpdate();
      printRoomTarget();
      rememberLastCommand(command);
      return;
    }
  }

  if (upper == "SALON") {
    goToRoomPreset(ROOM_POINT_SALON_X_M, ROOM_POINT_SALON_Y_M, F("SALON"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "SALLE") {
    goToRoomPreset(ROOM_POINT_SALLE_A_MANGER_X_M, ROOM_POINT_SALLE_A_MANGER_Y_M, F("SALLE"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "CUISINE") {
    goToRoomPreset(ROOM_POINT_CUISINE_X_M, ROOM_POINT_CUISINE_Y_M, F("CUISINE"));
    rememberLastCommand(command);
    return;
  }

  if (upper == "LIST") {
    Serial.println(F("Preset points: SALON (-1.5,-0.5), SALLE (-3.5,0.5), CUISINE (-6.0,3.0)"));
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(command);
  printSerialHelp();
}

void goToRoomPreset(float x_m, float y_m, const __FlashStringHelper *label) {
  target_x_m = x_m;
  target_y_m = y_m;
  syncTargetFromRoomXY();
  applyTargetUpdate();
  Serial.print(F("PRESET "));
  Serial.print(label);
  Serial.print(F(" => "));
  printRoomTarget();
}

void handleSerialInput() {
  static String inputBuffer = "";

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleSerialCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  delay(1000);
  Serial.println(F("wmirror ready"));
  printSerialHelp();

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
  handleSerialInput();

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
        applyTargetUpdate();
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

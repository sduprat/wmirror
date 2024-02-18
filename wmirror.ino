#include <TimeLib.h>
#include <SunPosition.h>
#include <Servo.h>
#include <math.h>

#define LATITUDE 43.6 // Latitude de Lavaur en degrés décimaux
#define LONGITUDE 1.8 // Longitude de Lavaur en degrés décimaux

float polaris_azimuth = 0.0; // Polaris est située approximativement au pôle nord céleste, son azimut est donc 0 degrés
float polaris_altitude = 90.0 - LATITUDE; // L'élévation de Polaris est égale à 90 degrés moins la latitude de l'observateur

//float norm_azimuth = 180.0;
//float norm_alttude = 30.0;

Servo servoH;

void setup() {
  Serial.begin(9600);
  setTime(21, 0, 0, 13, 2, 2024); // Définir l'heure et la date actuelles (heure:minute:seconde, jour:mois:année)
  servoH.attach(7);
}

double angle_normal(double angle1, double angle2) {

  float result;
//  double result = atan2(
//    (sin(angle1 * M_PI / 180.0) + sin(angle2 * M_PI / 180.0))/2,
//    (cos(angle1 * M_PI / 180.0) + cos(angle2 * M_PI / 180.0))/2
//    ) * 180.0 / M_PI;
//  if (result < 0.0) {
//    result += 360.0;
//  }
  if ((angle1-angle2)>180)
    result = ((angle1 + angle2)/2+180);
    else {
      if ((angle1-angle2)<-180)
        result = ((angle1 + angle2)/2-180);
      else
        result = (angle1 + angle2)/2;}
  return result;
}

void loop() {
  SunPosition sun(LATITUDE, LONGITUDE, now()); // Créer un objet SunPosition pour la latitude, la longitude et l'heure actuelles
  float azimuth = sun.azimuth(); // Obtenir l'azimut du soleil
  float altitude = sun.altitude(); // Obtenir l'élévation du soleil

  float norm_azimuth  = angle_normal(azimuth, polaris_azimuth);
  float norm_altitude = angle_normal(altitude, polaris_altitude);
  
  servoH.write( norm_azimuth);

  Serial.print("SAzimuth: ");
  Serial.print(azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tSAltitude: ");
  Serial.print(altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
  Serial.print("PAzimuth: ");
  Serial.print(polaris_azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tPAltitude: ");
  Serial.print(polaris_altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
  Serial.print("NAzimuth: ");
  Serial.print(norm_azimuth, 4); // Afficher l'azimut avec une précision de 4 décimales
  Serial.print(" degrees");
  Serial.print("\tNAltitude: ");
  Serial.print(norm_altitude, 4); // Afficher l'élévation avec une précision de 4 décimales
  Serial.println(" degrees");
  delay(4000); // Attendre une seconde avant la prochaine itération de la boucle
}
#include <SPI.h>
#include <pgmspace.h>
#include "PMW3389.h"

#define SS1  9
#define SS2  10
PMW3389 sensor1, sensor2;
int xydat_1[2] = {0, 0};
int xydat_2[2] = {0, 0};
unsigned long currTime;
unsigned long pollTimer = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting dual PMW3389 sensor initialization...");
  sensor1.begin(SS1);
  sensor2.begin(SS2);
  delay(250);
  if (sensor1.begin(SS1))
    Serial.println("Sensor1 initialization successed");
  else
    Serial.println("Sensor1 initialization failed");
  if (sensor2.begin(SS2))
    Serial.println("Sensor2 initialization successed");
  else
    Serial.println("Sensor2 initialization failed");
  sensor1.setCPI(1600);
  sensor2.setCPI(1600);
  int cpi1 = sensor1.getCPI();
  int cpi2 = sensor2.getCPI();
  Serial.print("Sensor 1 CPI: ");
  Serial.println(cpi1);
  Serial.print("Sensor 2 CPI: ");
  Serial.println(cpi2);
  if (cpi1 != cpi2 || cpi1 != 1600)
    Serial.println("WARNING: CPI initialization failed");
  else
    Serial.println("Both sensors detected successfully! Starting polling mode...");
  sensor1.readBurst();
  sensor2.readBurst();
}



void loop() {
  currTime = millis();
  if(currTime > pollTimer) {
    PMW3389_DATA data1 = sensor1.readBurst();
    PMW3389_DATA data2 = sensor2.readBurst();
    xydat_1[0] = data1.dx;
    xydat_1[1] = data1.dy;
    xydat_2[0] = data2.dx;
    xydat_2[1] = data2.dy;
    Serial.print("Sensor1 x=");
    Serial.print(xydat_1[0]);
    Serial.print(" y=");
    Serial.print(xydat_1[1]);
    Serial.print(" | Sensor2 x=");
    Serial.print(xydat_2[0]);
    Serial.print(" y=");
    Serial.println(xydat_2[1]);
    pollTimer = currTime + 100;
  }
}
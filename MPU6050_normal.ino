
#include <Wire.h>
#include <SD.h>
#include "MPU6050.h"

MPU6050 mpu;
float filteracc(float acc, int i, int j);

float varVolt[2][3] = {{9.65, 9.08, 8.29}, {0.92, 1.45, 0.73}}; // disp
float varProcess = 0.05; //speed of reaction
float acl = 0.061;
float gyr = 0.00875;
float Pc [5][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
float G [5][6] = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
float P [5][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
float Xp [5][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
float Zp [5][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
float Xe [5][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

float pitch[5];
float roll[5];
float yaw[5];

char report[512];
char file_out[512];

float nowYaw[5][50] = {0};
float nowPitch[5][50] = {0};
float nowRoll[5][50] = {0};

float exYaw[5][50] = {0};
float exPitch[5][50] = {0};
float exRoll[5][50] = {0};

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(50000);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");
  for (int i = 0; i <= 4; i++) {
    if (i == 4) {
      digitalWrite(10, LOW);
      mpu.setWire(1);
      while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
      {
        Serial.println("Could not find a valid 555MPU6050 sensor, check wiring!");
        delay(10);
      }
      digitalWrite(10, HIGH);
    }
    else {
      digitalWrite(i + 6, LOW);
      mpu.setWire(0);
      while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
      {
        Serial.println("Could not find a valid [i] MPU6050 sensor, check wiring!");
        delay(10);
      }
      digitalWrite(i + 6, HIGH);
    }
  }

}

void loop()
{
  float acc[5][3] = {0};
  float gyr[5][3] = {0};
  int fil_acc[5][3] = {0};
  for (int i = 0; i <= 4; i++) {
    digitalWrite(i + 6, LOW);
    Vector normAccel = mpu.readNormalizeAccel();
    for (int j = 0; j <= 2; j++) {

      acc[i][j] = normAccel.XAxis;
      acc[i][j] = normAccel.YAxis;
      acc[i][j] = normAccel.ZAxis;
      fil_acc[i][j] = filteracc(acc[i][j], i, j);
      fil_acc[i][j] = filteracc(acc[i][j], i, j);
      fil_acc[i][j] = filteracc(acc[i][j], i, j);
    }
    digitalWrite(i + 6, HIGH);
    pitch[i] = 180 * atan (fil_acc[i][1] / sqrt(fil_acc[i][2] * fil_acc[i][2] + fil_acc[i][3] * fil_acc[i][3])) / M_PI;
    roll[i] = 180 * atan (fil_acc[i][2] / sqrt(fil_acc[i][1] * fil_acc[i][1] + fil_acc[i][3] * fil_acc[i][3])) / M_PI;
    yaw[i] = 180 * atan (fil_acc[i][3] / sqrt(fil_acc[i][1] * fil_acc[i][1] + fil_acc[i][3] * fil_acc[i][3])) / M_PI;
  }
  Serial.print("YPR: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(yaw[i]);
    Serial.print(" ");
    Serial.print(pitch[i]);
    Serial.print(" ");
    Serial.print(roll[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  for (int j = 0; j < 5; j++) {
    for (int i = 48; i >= 0; i--) {
      nowYaw[j][i + 1] = nowYaw[j][i];
      nowPitch[j][i + 1] = nowPitch[j][i];
      nowRoll[j][i + 1] = nowRoll[j][i];
    }
  }

  // korrelator i sr znach
  float srznachYaw [2][5] = {0};
  float srznachPitch [2][5] = {0};
  float srznachRoll [2][5] = {0};

  for (int i = 0; i <= 4; i++ ) {
    for (int j = 0; j <= 49; j++) {
      srznachYaw [1][i] += nowYaw[i][j];
      srznachPitch [1][i] += nowPitch[i][j];
      srznachRoll [1][i] += nowRoll[i][j];
      srznachYaw [2][i] += exYaw[i][j];
      srznachPitch [2][i] += exPitch[i][j];
      srznachRoll [2][i] += exRoll[i][j];
    }
  }
  float korrelYaw [5] = {0};
  float korrelPitch [5] = {0};
  float korrelRoll [5] = {0};
  float summYaw [5][3] = {0};
  float summPitch [5][3] = {0};
  float summRoll [5][3] = {0};
  for (int i = 0; i <= 4; i++) {
    for (int j = 0; j <= 49; j++) {
      summYaw [i][1] += (nowYaw[i][j] - srznachYaw [1][i]) * (exYaw[i][j] - srznachYaw [2][i]);
      summYaw [i][2] += (nowYaw[i][j] - srznachYaw [1][i]) * (nowYaw[i][j] - srznachYaw [1][i]);
      summYaw [i][3] += (exYaw[i][j] - srznachYaw [2][i]) * (exYaw[i][j] - srznachYaw [2][i]);
    }
    korrelYaw[i] = summYaw [i][1] / sqrt(summYaw [i][2] * summYaw [i][3]);
    Serial.println(korrelYaw[i]);
  }
  delay(10);
}
float filteracc(float acc, int i, int j) { //функция фильтрации
  Pc[i][j] = P[i][j] + varProcess;
  G[i][j] = Pc[i][j] / (Pc[i][j] + varVolt[0][j]);
  P[i][j] = (1 - G[i][j]) * Pc[i][j];
  Xp[i][j] = Xe[i][j];
  Zp[i][j] = Xp[i][j];
  Xe[i][j] = G[i][j] * (acc - Zp[i][j]) + Xp[i][j]; // "фильтрованное" значение
  return (Xe[i][j]);
}

#define SOP 255
#define Ts 10000
#define Ts_sec 0.01

#define algoritmoVelocidade 0

//float Ts_sec;
uint32_t Tant;
int acaoA = 0;
int acaoB = 0;
float correnteA = 0;
float correnteB = 0;
int pwmA = 17;
int pwmB = 17;
int acaoMedia;
float acaoDelta, acaoInst;
float setpoint;
float erro[3]={0,0,0};
float Kp, Ki, Kd, Ti, Td;
float acaoP, acaoI, acaoD;
float q0, q1, q2;

int acaoTeste = 0;
/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>

//Variáveis para controlar os motores
Servo myservoA;
Servo myservoB; 
//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
//double anguloTeste = 10.5;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  Serial.begin(115200);
  myservoA.attach(9);
  myservoB.attach(10);
  myservoA.write(17);
  myservoB.write(17); 
  initializingPID();
  Serial.println(q0);
  Serial.println(q1);
  Serial.println(q2);
  Serial.println(Ti);
  Serial.println(Td);
  Serial.println(Ts_sec);
  while(millis()<5000);
  digitalWrite(13,HIGH);
  
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  Tant = micros();
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  //Serial.print(accX); //Serial.print("\t");
  //Serial.print(accY); //Serial.print("\t");
  //Serial.print(accZ); //Serial.print("\t");

  //Serial.print(gyroX); //Serial.print("\t");
  //Serial.print(gyroY); //Serial.print("\t");
  //Serial.print(gyroZ); //Serial.print("\t");

  //Serial.print("\t");
#endif

  //Serial.print(roll); //Serial.print("\t");
  ////Serial.print(gyroXangle); //Serial.print("\t");
  ////Serial.print(compAngleX); //Serial.print("\t");
  //Serial.print(kalAngleX); //Serial.print("\t");

  //Serial.print("\t");

  ////Serial.print(pitch); //Serial.print("\t");
  ////Serial.print(gyroYangle); //Serial.print("\t");
  ////Serial.print(compAngleY); //Serial.print("\t");
  ////Serial.print(kalAngleY); //Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  //Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  //Serial.print(temperature); //Serial.print("\t");
#endif

  //Serial.print("\r\n");
  delay(2);
  if(micros() - Tant < Ts){
    Tant = micros();
    Serial.print(getKalmanAngle()); Serial.print("\t");
    Serial.print(acaoA); Serial.print("\t");
    Serial.print(acaoP); Serial.print("\t");
    Serial.print(acaoI); Serial.print("\t");
    Serial.println(acaoD); Serial.print("\t");
    /*Serial.print(pwmA); Serial.print("\t");
    Serial.print(pwmB); Serial.print("\t");
    Serial.print(correnteA); Serial.print("\t");
    Serial.println(correnteB);*/
    
    acaoMedia = 50;
    if(((millis()/10000)%4) == 0){
      setpoint = -12;
    } else if(((millis()/10000)%4) == 1) {
      setpoint = 0;
    } else if(((millis()/10000)%4) == 2) {
      setpoint = 12;
    } else if(((millis()/10000)%4) == 3) {
      setpoint = 0;
    }
    setpoint = 0;
    
    controlAngle();
    
    /*Serial.print(getKalmanAngle()); Serial.print("\t");
    Serial.print(acaoA); Serial.print("\t");
    Serial.println(acaoB);*/
  }
  //myservoA.write(acaoA);
  //myservoB.write(acaoB); 
  setMotor();
}

void controlAngle(){
  //input = ângulo
  //output = porcentagem da corrente para os motores
  
  erro[2] = erro[1];
  erro[1] = erro[0];
  erro[0] = setpoint - getKalmanAngle();
  
  
  //acaoDelta = Kp*erro[0];
  #if algoritmoVelocidade
  acaoInst = q0*erro[0] + q1*erro[1] + q2*erro[2];
  acaoDelta = acaoDelta + acaoInst;
  #else
  acaoP = Kp * erro[0];
  acaoI += Ki *erro[0];
  acaoD = Kd *(erro[0]-erro[1]);
  
  int Imax = 15;
  if (acaoI > Imax){
    acaoI = Imax;
  } else if (acaoI < ((-1)*Imax)){
    acaoI = ((-1)*Imax);
  }
  if (erro[0]*erro[1]<0){ //Anti-windup / Troca de sinal do erro
    acaoI = 0;
  }

  acaoDelta = acaoP + acaoI + acaoD;
  #endif
  int saturacao = 30;
  if (acaoDelta > saturacao){
    acaoDelta = saturacao;
  } else if (acaoDelta < ((-1)*saturacao)){
    acaoDelta = ((-1)*saturacao);
  }
  
  acaoA = acaoMedia + acaoDelta;
  acaoB = acaoMedia - acaoDelta;

  if(acaoA > 100){
    acaoA = 100;
  } else if(acaoA<0){
    acaoA = 0;
  }

  if(acaoB > 100){
    acaoB = 100;
  } else if(acaoB<0) {
    acaoB = 0;
  }
}

void initializingPID(){
  //Ts_sec = Ts/1000000;
  //Ts_sec = 0.02;

  //1ª sequencia - 20ms ok
  //Kp = 0.6;
  //Ki = 0.4;
  //Kd = 0.7;

  //1ª sequencia - 10ms ok
  /*Kp = 0.9; // laboratorio
  Ki = 0.05;// laboratorio
  Kd = 0.35;// laboratorio*/
  Kp = 0.9; //casa
  Ki = 0.8; //casa
  Kd = 0.5; //casa
  /*Kp = 2.0;
  Ki = 0.7;
  Kd = 1.0;*/

  #if algoritmoVelocidade
  Td = Kd/Kp; //Se Kd = 0 -> Td = 0
  //Conta própria
  if(Ki==0){
    Ti = 999999; //Número grande -> infinito
    q0 = Kp*(1 + (Td/Ts_sec));
    q1 = -1*Kp* (1 + (2*Td/Ts_sec));
    q2 = Kp*Td/Ts_sec;
  } else {
    Ti = Kp/Ki; //Se Ki = 0 -> Ti = inf
    q0 = Kp*(1+ (Ts_sec/(2*Ti))+(Td/Ts_sec));
    q1 = -1*Kp* (1 - (Ts_sec/(2*Ti)) + (2*Td/Ts_sec));
    q2 = Kp*Td/Ts_sec;
  }

  /*
  //Slide
  if(Ki==0){
    Ti = 999999; //Número grande -> infinito
    q0 = Kp*(1+ (Td/Ts_sec));
    q1 = (-1)*Kp* (1 + (2*Td/Ts_sec));
    q2 = Kp* ( (Td/Ts_sec));
  } else {
    Ti = Kp/Ki; //Se Ki = 0 -> Ti = inf
    q0 = Kp*(1+ (Ts_sec/(2*Ti))+(Td/Ts_sec));
    q1 = (-1)*Kp* (1 + (2*Td/Ts_sec));
    q2 = Kp* ( (Td/Ts_sec) - (Ts_sec/(2*Ti)));
  }*/
  #else
  Ki = Ki*Ts_sec;
  Kd = Kd/Ts_sec;
  acaoI = 0;
  #endif
}

inline double getKalmanAngle()  {return kalAngleX;}

void setMotor(){
  // input = porcentagem da corrente
  // output = pwm para os motores
  acaoA = acaoTeste;
  acaoB = acaoTeste;
  correnteA = acaoA *4.0/100.0;
  correnteB = acaoB *4.0/100.0;
  /*
  //0,978846550482554  -12,3656561004757 61,3473567757761  12,3936804899468
  pwmA = (0.9788*pow(correnteA,3)) -(12.3657*pow(correnteA,2)) +(61.3474*correnteA) + 12.3937;
  //0,428489165486088  -6,27724281729208 35,3168544839856  15,4368321984141
  pwmB = (0.4285*pow(correnteB,3)) -(6.2772*pow(correnteB,2)) +(35.3168*correnteB) + 15.4368;
  */
  
  //12.1V - fonte casa
  //0.1176   -5.0773   40.0227   14.3006
  pwmA = (0.1176*pow(correnteA,3)) -(5.0773*pow(correnteA,2)) +(40.0227*correnteA) + 14.3006;
  //0.3207   -4.7661   30.2759   16.0771
  pwmB = (0.3207*pow(correnteB,3)) -(4.7661*pow(correnteB,2)) +(30.2759*correnteB) + 16.0771;

  /*//12.0V - fonte laboratório
  //0.1835   -5.1140   38.8759   13.6487
  correnteA = correnteA + 0.1;
  pwmA = (0.1835*pow(correnteA,3)) -(5.1140*pow(correnteA,2)) +(38.8759*correnteA) + 13.6487;
  //0.3516   -5.0109   30.4850   14.9275
  pwmB = (0.3516*pow(correnteB,3)) -(5.0109*pow(correnteB,2)) +(30.4850*correnteB) + 14.9275;*/
  
  if(pwmA < 17){
    pwmA = 17;
  }
  if(pwmB < 17){
    pwmB = 17;
  }
  myservoA.write(pwmA);
  myservoB.write(pwmB);
  
  /*
  int upperLimit = 170;
  int lowerLimit = 17;
  if(acaoA<lowerLimit){
    myservoA.write(lowerLimit);
  } else if (acaoA>upperLimit) {
    myservoA.write(upperLimit);
  } else {
    myservoA.write(acaoA);
  }

  if(acaoB<lowerLimit){
    myservoB.write(lowerLimit);
  } else if (acaoB>upperLimit) {
    myservoB.write(upperLimit);
  } else {
    myservoB.write(acaoB);
  }*/
}

bool RX(){
  if(Serial.available()>0){
    acaoTeste = Serial.read(); // Variavel que receberá os valores enviados pelo programa em python
    return 1;
  }
  return 0;
}

void TX(){
  byte vetor[3];
  vetor[0]= (byte)SOP;
  /*vetor[1]= (byte)((((uint16_t)pos_y) >> 8) & 0x00FF);
  vetor[2]= (byte)(((uint16_t)pos_y) & 0x00FF);*/
  vetor[1]= (byte)((0x3E8 >> 8) & 0x00FF);
  vetor[2]= (byte)(0x3E8 & 0x00FF);
  Serial.write((uint8_t*)vetor,3);
}
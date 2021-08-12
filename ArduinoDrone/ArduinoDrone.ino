// --- Constantes Timers Interrupcao ---
const uint16_t T2_init = 0;
const uint16_t T2_comp = 156; //156 - 10ms

#define SOP 255
#define Ts 10000
#define Ts_sec 0.01
#define python 0
#define algoritmoVelocidade 0
#define fuzzy 0
#define controleAngulo 1
#define controleAltitude 0

uint32_t Tant;
int acaoA = 0;
int acaoB = 0;
float correnteA = 0;
float correnteB = 0;
int pwmA = 17;
int pwmB = 17;
int acao_altitude;
float acao_angle, acaoInst;
float setpoint_angle= 0;
float erro_angle[3]={0,0,0};
float angle[3] = {0,0,0};
float aceleracao, delta_Kp, delta_Ki, delta_Kd;
float Kp_angle, Ki_angle, Kd_angle, Ti_angle, Td_angle;
float acaoP_angle, acaoI_angle, acaoD_angle;
int acao_max_angle = 30;
int acaoI_max_angle = 15;
int acaoD_max_angle = 15;
float q0, q1, q2;

int setpoint_altitude;
int erro_altitude[3]={0,0,0};
float Kp_altitude, Ki_altitude, Kd_altitude;
float acaoP_altitude, acaoI_altitude, acaoD_altitude;
int acao_max_altitude = 80;
int acaoI_max_altitude = 40;

int pos_y_cm = 0;

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
//double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  pinMode(6,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  Serial.begin(115200);
  myservoA.attach(9);
  myservoB.attach(10);
  myservoA.write(17);
  myservoB.write(17); 
  initializingPID();
  #if python
  while(!RX()); //Travando o script para rodar o loop apenas quando o primeiro bit for recebido
  #else
  while(millis()<5000);
  #endif
   //Configurando Interrupcao
   //Modo de Comparação
   TCCR2A = 0;
   //Prescaler 1:1024
   TCCR2B |= (1 << CS22);
   TCCR2B |= (1 << CS21);
   TCCR2B |= (1 << CS20);
   //Inicializa Registradores
   TCNT2 = T2_init;
   OCR2A = T2_comp;
   //Habilita Interrupção do Timer2
   TIMSK2 = (1 << OCIE2A); 
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
  //compAngleX = roll;
  //compAngleY = pitch;

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
    //compAngleX = roll;
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
    //compAngleY = pitch;
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

  //compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  delay(2);
  if(micros() - Tant >= Ts){

  } 
  setMotor();
}

void controlAltitude(){
  //input = altitude
  //output = porcentagem média da corrente para os motores
  erro_altitude[2] = erro_altitude[1];
  erro_altitude[1] = erro_altitude[0];
  erro_altitude[0] = setpoint_altitude - pos_y_cm;
  
  acaoP_altitude = Kp_altitude * erro_altitude[0];
  acaoI_altitude += Ki_altitude * erro_altitude[0];
  /*if((erro_altitude[0] < 200) && (erro_altitude[0] > -200)){
    acaoD_altitude = Kd_altitude * (erro_altitude[0] - erro_altitude[1]);
  } else{
    acaoD_altitude = 0;
  }*/
  acaoD_altitude = Kd_altitude * (erro_altitude[0] - erro_altitude[1]);
  

  if (acaoI_altitude > acaoI_max_altitude){
    acaoI_altitude = acaoI_max_altitude;
  } else if (acaoI_altitude < ((-1)*acaoI_max_altitude)){
    acaoI_altitude = ((-1)*acaoI_max_altitude);
  }

  acao_altitude = acaoP_altitude + acaoI_altitude + acaoD_altitude;

  if (acao_altitude > acao_max_altitude){
    acao_altitude = acao_max_altitude;
  } else if (acao_altitude < 0){
    acao_altitude = 0;
  }
}

void controlAngle(){
  //input = ângulo
  //output = porcentagem da corrente para os motores
  angle[2] = angle[1];
  angle[1] = angle[0];
  angle[0] = (angle[2] + angle[1] + getKalmanAngle())/3;
  erro_angle[2] = erro_angle[1];
  erro_angle[1] = erro_angle[0];
  erro_angle[0] = setpoint_angle - angle[0];
  
  #if algoritmoVelocidade
  acaoInst = q0*erro_angle[0] + q1*erro_angle[1] + q2*erro_angle[2];
  acao_angle = acao_angle + acaoInst;
  #else
  #if fuzzy
  //Fuzzy
  aceleracao = erro_angle[0] - (2*erro_angle[1]) + erro_angle[2];
  //P0.5 I0.08 D0.20
  //P0.5 I0.08*0.010 D0.20/0.010
  //P0.5 I0.0008 D20
  if(abs(erro_angle[0]) < 6){ //erro pequeno
    analogWrite(6,0);
    if( aceleracao < -0.07 ){ //aceleracao negativa
      delta_Kp = Kp_angle/3;
      delta_Ki = (+1)*Ki_angle/3;
      delta_Kd = Kd_angle/3;
    } else if ( aceleracao > 0.07) { //aceleracao positiva
      delta_Kp = 0;
      delta_Ki = (+1)*Ki_angle/3;
      delta_Kd = Kd_angle/3;
    } else { //aceleracao neutra
      delta_Kp = Kp_angle/3;
      delta_Ki = 0;
      delta_Kd = 0;//Kd_angle/3;
    }
  } else{ //erro grande
    analogWrite(6,100);
    if(aceleracao < -0.07){ //aceleracao negativa
      delta_Kp = (-1)*Kp_angle/3;
      delta_Ki = 0;
      delta_Kd = Kd_angle/3;
    } else if ( aceleracao > 0.07) { //aceleracao positiva
      delta_Kp = (-1)*Kp_angle/3;
      delta_Ki = Ki_angle/3;
      delta_Kd = Kd_angle/3;
    } else { //aceleracao neutra
      delta_Kp = 0;
      delta_Ki = Ki_angle/3;
      delta_Kd = (-1)*Kd_angle/3;
    }
  }
  acaoP_angle = (Kp_angle + delta_Kp) * erro_angle[0];
  acaoI_angle += (Ki_angle + delta_Ki) *erro_angle[0];
  acaoD_angle = (Kd_angle + delta_Kd) *(erro_angle[0]-erro_angle[1]);
  #else
  acaoP_angle = Kp_angle * erro_angle[0];
  acaoI_angle += Ki_angle *erro_angle[0];
  acaoD_angle = Kd_angle *(erro_angle[0]-erro_angle[1]);
  #endif
  
  if (acaoI_angle > acaoI_max_angle){
    acaoI_angle = acaoI_max_angle;
  } else if (acaoI_angle < ((-1)*acaoI_max_angle)){
    acaoI_angle = ((-1)*acaoI_max_angle);
  }
  if (acaoD_angle > acaoD_max_angle){
    acaoD_angle = acaoD_max_angle;
  } else if (acaoD_angle < ((-1)*acaoD_max_angle)){
    acaoD_angle = ((-1)*acaoD_max_angle);
  }
  if (erro_angle[0]*erro_angle[1]<0){ //Anti-windup / Troca de sinal do erro
    acaoI_angle = 0;
  }

  acao_angle = acaoP_angle + acaoI_angle + acaoD_angle;
  #endif
  if (acao_angle > acao_max_angle){
    acao_angle = acao_max_angle;
  } else if (acao_angle < ((-1)*acao_max_angle)){
    acao_angle = ((-1)*acao_max_angle);
  }
}

void initializingPID(){
  //Ts_sec = Ts/1000000;
  //Ts_sec = 0.02;

  //1ª sequencia - 20ms ok
  //Kp_angle = 0.6;
  //Ki_angle = 0.4;
  //Kd_angle = 0.7;

  //1ª sequencia - 10ms ok
  /*Kp_angle = 0.9; // laboratorio
  Ki_angle = 0.05;// laboratorio
  Kd_angle = 0.35;// laboratorio*/
  Kp_angle = 0.6; //0.8; //0.5   //0.5
  Ki_angle = 0.4;//0.2; //0.12  //0.5
  Kd_angle = 0.3;//0.3; //0.20  //0.3
  //0.9 0.10  0.20 - 14:55
  //0.5 0.25  0.20 - 15:58
  acaoI_angle = 0;
  acao_max_angle = 40;
  acaoI_max_angle = 15;
  acaoD_max_angle = 15;
  /*Kp_angle = 2.0;
  Ki_angle = 0.7;
  Kd_angle = 1.0;*/
  Kp_altitude = 0.1; //0.10 05/08 as 21:21
  Ki_altitude = 0.008;//0.032 05/08 as 21:21
  Kd_altitude = 0.10;//0.04 05/08 as 21:21
  acao_max_altitude = 80;
  acaoI_max_altitude = 70;
  acaoI_altitude = 30;

  #if algoritmoVelocidade
  Td_angle = Kd_angle/Kp_angle; //Se Kd_angle = 0 -> Td_angle = 0
  /*//Conta própria
  if(Ki_angle==0){
    Ti_angle = 999999; //Número grande -> infinito
    q0 = Kp_angle*(1 + (Td_angle/Ts_sec));
    q1 = -1*Kp_angle* (1 + (2*Td_angle/Ts_sec));
    q2 = Kp_angle*Td_angle/Ts_sec;
  } else {
    Ti_angle = Kp_angle/Ki_angle; //Se Ki_angle = 0 -> Ti_angle = inf
    q0 = Kp_angle*(1+ (Ts_sec/(2*Ti_angle))+(Td_angle/Ts_sec));
    q1 = -1*Kp_angle* (1 - (Ts_sec/(2*Ti_angle)) + (2*Td_angle/Ts_sec));
    q2 = Kp_angle*Td_angle/Ts_sec;
  }*/

  //Slide
  if(Ki_angle==0){
    Ti_angle = 999999; //Número grande -> infinito
    q0 = Kp_angle*(1+ (Td_angle/Ts_sec));
    q1 = (-1)*Kp_angle* (1 + (2*Td_angle/Ts_sec));
    q2 = Kp_angle* ( (Td_angle/Ts_sec));
  } else {
    Ti_angle = Kp_angle/Ki_angle; //Se Ki_angle = 0 -> Ti_angle = inf
    q0 = Kp_angle*(1+ (Ts_sec/(2*Ti_angle))+(Td_angle/Ts_sec));
    q1 = (-1)*Kp_angle* (1 + (2*Td_angle/Ts_sec));
    q2 = Kp_angle* ( (Td_angle/Ts_sec) - (Ts_sec/(2*Ti_angle)));
  }
  #else
  Ki_angle = Ki_angle*Ts_sec;
  Kd_angle = Kd_angle/Ts_sec;
  

  Ki_altitude = Ki_altitude*Ts_sec;
  Kd_altitude = Kd_altitude/Ts_sec;
  
  #endif
}

inline double getKalmanAngle()  {return kalAngleX;}

void setMotor(){
  // input = porcentagem da corrente
  // output = pwm para os motores
  correnteA = (acaoA) *4.0/100.0;
  correnteB = (acaoB) *4.0/100.0;
  /*
  //0,978846550482554  -12,3656561004757 61,3473567757761  12,3936804899468
  pwmA = (0.9788*pow(correnteA,3)) -(12.3657*pow(correnteA,2)) +(61.3474*correnteA) + 12.3937;
  //0,428489165486088  -6,27724281729208 35,3168544839856  15,4368321984141
  pwmB = (0.4285*pow(correnteB,3)) -(6.2772*pow(correnteB,2)) +(35.3168*correnteB) + 15.4368;
  */
  
  /*//12.3V - fonte casa - Identificação feita no dia 03/08/2021
  //0.1750   -5.1728   39.3825   14.2633
  pwmA = (0.1750*pow(correnteA,3)) -(5.1728*pow(correnteA,2)) +(39.3825*correnteA) + 14.2633;
  //0.3054   -4.5860   29.5796   15.8273
  pwmB = (0.3054*pow(correnteB,3)) -(4.5860*pow(correnteB,2)) +(29.5796*correnteB) + 15.8273;*/

  //12.6V - fonte laboratório
  //0.2996   -5.8052   39.6417   14.3045
  pwmA = (0.2996*pow(correnteA,3)) -(5.8052*pow(correnteA,2)) +(39.6417*correnteA) + 14.3045;
  //0.2977   -4.5052   29.1644   16.0212
  pwmB = (0.2977*pow(correnteB,3)) -(4.5052*pow(correnteB,2)) +(29.1644*correnteB) + 16.0212;
  
  if(pwmA < 17){
    pwmA = 17;
  } else if (pwmA >100){
    pwmA = 100;
  }
  if(pwmB < 17){
    pwmB = 17;
  } else if (pwmB > 100){
    pwmB = 100;
  }
  myservoA.write(pwmA);
  myservoB.write(pwmB);
  /*if((pwmA>17 && pwmA <170) || (pwmB>17 && pwmB <170))
  {
    analogWrite(6,100);
  }
  else{
    analogWrite(6,0);
  }*/
  
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
    #if python
    if(Serial.read()==SOP){ //Primeiro byte é o SOP esperado
      pos_y_cm = Serial.read();
      pos_y_cm = (pos_y_cm << 8) | Serial.read(); //pos_y em cm
      Serial.read();  //Apenas para tirar da Serial a a_y
      Serial.read();  //Apenas para tirar da Serial a a_y
      setpoint_altitude = Serial.read();
      setpoint_altitude = setpoint_altitude*100;
      setpoint_angle = Serial.read();
    }
    #else
    setpoint_angle = 0;
    #endif
    return 1;
  }
  return 0;
}

void TX(){
  byte vetor[7];
  vetor[0]= (byte)SOP;
  vetor[1]= (byte)((((int)(getKalmanAngle()*100)) >> 8) & 0x00FF);
  vetor[2]= (byte)(((int)(getKalmanAngle()*100)) & 0x00FF);
  #if controleAngulo
  vetor[3]= (byte)(((int)(acaoP_angle)) & 0x00FF);
  vetor[4]= (byte)(((int)(acaoI_angle)) & 0x00FF);
  vetor[5]= (byte)(((int)(acaoD_angle)) & 0x00FF);
  vetor[6]= (byte)(((int)(acao_angle)) & 0x00FF);
  #endif
  #if controleAltitude
  vetor[3]= (byte)(((int)(acaoP_altitude)) & 0x00FF);
  vetor[4]= (byte)(((int)(acaoI_altitude)) & 0x00FF);
  vetor[5]= (byte)(((int)(acaoD_altitude)) & 0x00FF);
  vetor[6]= (byte)(((int)(acao_altitude)) & 0x00FF);
  #endif
  
  #if python
  Serial.write((uint8_t*)vetor,7);
  #else
  Serial.print(getKalmanAngle()); Serial.print('\t');
  Serial.print(acaoP_angle); Serial.print('\t');
  Serial.print(acaoI_angle); Serial.print('\t');
  Serial.print(acaoD_angle); /*Serial.print('\t');
  Serial.print(aceleracao*100);*/ Serial.print('\n');
  #endif
}

// ======================================================================================================
// --- Interrupção ---
ISR(TIMER2_COMPA_vect)
{
  TCNT2 = T2_init;      //reinicializa TIMER1
  //Serial.print(micros()-Tant); Serial.print("\t");
  //Serial.println(getKalmanAngle());
  //Tant = micros();
  RX();
  TX();
  /*Serial.print(getKalmanAngle()); Serial.print("\t");
  Serial.print(acaoA); Serial.print("\t");
  Serial.print(acaoP_angle); Serial.print("\t");
  Serial.print(acaoI_angle); Serial.print("\t");
  Serial.println(acaoD_angle); Serial.print("\t");
  Serial.print(pwmA); Serial.print("\t");
  Serial.print(pwmB); Serial.print("\t");
  Serial.print(correnteA); Serial.print("\t");
  Serial.println(correnteB);*/

  //setpoint_angle = 0;
  /*setpoint_altitude = 500;
  if(millis()>18000){
    setpoint_altitude = 800;
  }*/
  //setpoint_altitude = ((millis() - 5000)/20)+100;
  
  #if controleAltitude
  controlAltitude();
  #else
  acao_altitude = 40;
  #endif
  
  #if controleAngulo
  controlAngle();
  #else
  acao_angle = 0;
  #endif

  
  /*if(acao_angle>0){
    acaoA = acao_altitude + acao_angle;
    acaoB = acao_altitude;
  } else {
    acaoA = acao_altitude;
    acaoB = acao_altitude - acao_angle;
  }*/
  acaoA = acao_altitude + acao_angle;
  acaoB = acao_altitude - acao_angle;

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
} //end ISR

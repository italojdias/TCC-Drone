// --- Constantes Timers Interrupcao ---
const uint16_t T2_init = 0;
const uint16_t T2_comp = 156; //156 é a constante que equivale a uma interrupção de 10ms

#define SOP 255
#define Ts 10000
#define Ts_sec 0.01
#define python 1
#define controleAngulo 1 //Flag para ativar o controle do ângulo
#define controleAltitude 0 //Flag para ativar o controle da altitude

bool flagInterrupcao = 0;
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

float acao_angle_A, acao_angle_B;
float Kp_angle_A, Ki_angle_A, Kd_angle_A;
float acaoP_angle_A, acaoI_angle_A, acaoD_angle_A;
float acaoD_angle_A_prev;
int acao_max_angle_A = 30;
int acaoI_max_angle_A = 15;
int acaoD_max_angle_A = 15;
int delta_acaoD_max_angle_A = 10;
float Kp_angle_B, Ki_angle_B, Kd_angle_B;
float acaoP_angle_B, acaoI_angle_B, acaoD_angle_B;
int acao_max_angle_B = 30;
int acaoI_max_angle_B = 15;
int acaoD_max_angle_B = 15;

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
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
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
  if(flagInterrupcao){ //Só atualiza o filtro de kalman depois de uma interrupção
    flagInterrupcao = 0;
    digitalWrite(4,HIGH);
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
  
    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    //delay(2);
    digitalWrite(4,LOW);
  }
  setMotor(); //Função escreve o último PWM calculado nos motores
}

void controlAltitude(){
  //input = altitude
  //output = porcentagem média da corrente para os motores
  erro_altitude[2] = erro_altitude[1];
  erro_altitude[1] = erro_altitude[0];
  erro_altitude[0] = setpoint_altitude - pos_y_cm; //Calculando o erro
  
  acaoP_altitude = Kp_altitude * erro_altitude[0];
  acaoI_altitude += Ki_altitude * erro_altitude[0];
  acaoD_altitude = Kd_altitude * (erro_altitude[0] - erro_altitude[1]);
  
  if (acaoI_altitude > acaoI_max_altitude){ //Saturando a ação integrativa
    acaoI_altitude = acaoI_max_altitude;
  } else if (acaoI_altitude < ((-1)*acaoI_max_altitude)){
    acaoI_altitude = ((-1)*acaoI_max_altitude);
  }

  acao_altitude = acaoP_altitude + acaoI_altitude + acaoD_altitude;

  if (acao_altitude > acao_max_altitude){ //Saturando a ação da altitude
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
  angle[0] = 0.700*angle[1] + 0.300*getKalmanAngle(); //Filtro passa baixa
  erro_angle[2] = erro_angle[1];
  erro_angle[1] = erro_angle[0];
  erro_angle[0] = setpoint_angle - angle[0]; //Calculando o erro
  
  acaoP_angle_A =  Kp_angle_A * erro_angle[0];
  acaoI_angle_A += Ki_angle_A *erro_angle[0];
  acaoD_angle_A_prev = acaoD_angle_A;
  acaoD_angle_A =  Kd_angle_A *(erro_angle[0]-erro_angle[1]);
  if( (acaoD_angle_A - acaoD_angle_A_prev) < ((-1)*delta_acaoD_max_angle_A)){
    acaoD_angle_A = acaoD_angle_A_prev - delta_acaoD_max_angle_A;
  } else if ( (acaoD_angle_A - acaoD_angle_A_prev) > delta_acaoD_max_angle_A){
    acaoD_angle_A = acaoD_angle_A_prev + delta_acaoD_max_angle_A;
  }
  acaoP_angle_B =  Kp_angle_B * erro_angle[0];
  acaoI_angle_B += Ki_angle_B *erro_angle[0];
  acaoD_angle_B =  Kd_angle_B *(erro_angle[0]-erro_angle[1]);
  if (acaoI_angle_A > acaoI_max_angle_A){
    acaoI_angle_A = acaoI_max_angle_A;
  } else if (acaoI_angle_A < ((-1)*acaoI_max_angle_A)){
    acaoI_angle_A = ((-1)*acaoI_max_angle_A);
  }
  if (acaoD_angle_A > acaoD_max_angle_A){
    acaoD_angle_A = acaoD_max_angle_A;
  } else if (acaoD_angle_A < ((-1)*acaoD_max_angle_A)){
    acaoD_angle_A = ((-1)*acaoD_max_angle_A);
  }
  if (acaoI_angle_B > acaoI_max_angle_B){
    acaoI_angle_B = acaoI_max_angle_B;
  } else if (acaoI_angle_B < ((-1)*acaoI_max_angle_B)){
    acaoI_angle_B = ((-1)*acaoI_max_angle_B);
  }
  if (acaoD_angle_B > acaoD_max_angle_B){
    acaoD_angle_B = acaoD_max_angle_B;
  } else if (acaoD_angle_B < ((-1)*acaoD_max_angle_B)){
    acaoD_angle_B = ((-1)*acaoD_max_angle_B);
  }
  if (erro_angle[0]*erro_angle[1]<0){ //Anti-windup / Troca de sinal do erro
    acaoI_angle_A = 0;
    acaoI_angle_B = 0;
  }
  acao_angle_A = acaoP_angle_A + acaoI_angle_A + acaoD_angle_A;
  acao_angle_B = acaoP_angle_B + acaoI_angle_B + acaoD_angle_B;
  if (acao_angle_A > acao_max_angle_A){
    acao_angle_A = acao_max_angle_A;
  } else if (acao_angle_A < ((-1)*acao_max_angle_A)){
    acao_angle_A = ((-1)*acao_max_angle_A);
  }
  if (acao_angle_B > acao_max_angle_B){
    acao_angle_B = acao_max_angle_B;
  } else if (acao_angle_B < ((-1)*acao_max_angle_B)){
    acao_angle_B = ((-1)*acao_max_angle_B);
  }
}

void initializingPID(){
  //Constantes PID para o controle do ângulo
  Kp_angle_A = 0.69; 
  Ki_angle_A = 0.08;
  Kd_angle_A = 0.25;
  Kp_angle_B = 0.69; 
  Ki_angle_B = 0.9;
  Kd_angle_B = 0.20;
  acaoI_angle_A = 0;
  acao_max_angle_A = 40;
  acaoI_max_angle_A = 15;
  acaoD_max_angle_A = 25;
  delta_acaoD_max_angle_A = 3;
  acaoI_angle_B = 0;
  acao_max_angle_B = 40;
  acaoI_max_angle_B = 15;
  acaoD_max_angle_B = 40;

  Ki_angle_A = Ki_angle_A*Ts_sec;
  Kd_angle_A = Kd_angle_A/Ts_sec;
  Ki_angle_B = Ki_angle_B*Ts_sec;
  Kd_angle_B = Kd_angle_B/Ts_sec;

  //Constantes PID para o controle da altitude
  Kp_altitude = 0.1;
  Ki_altitude = 0.008;
  Kd_altitude = 0.10;
  acao_max_altitude = 80;
  acaoI_max_altitude = 70;
  acaoI_altitude = 30;

  Ki_altitude = Ki_altitude*Ts_sec;
  Kd_altitude = Kd_altitude/Ts_sec;
}

inline double getKalmanAngle()  {return kalAngleX;} //Função para retornar o ângulo filtrado

void setMotor(){
  // input = porcentagem da corrente
  // output = pwm para os motores
  //Convertendo a porcentagem da ação para cada motor em corrente
  correnteA = (acaoA+4) *4.0/100.0;
  correnteB = (acaoB-2) *4.0/100.0;

  //Modelo para calcular o PWM necessário para cada motor dependendo da corrente desejada
  pwmA = (0.2996*pow(correnteA,3)) -(5.8052*pow(correnteA,2)) +(39.6417*correnteA) + 14.3045;
  pwmB = (0.2977*pow(correnteB,3)) -(4.5052*pow(correnteB,2)) +(29.1644*correnteB) + 16.0212;
  
  //Saturando os valores de PWM
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
      python_running = Serial.read();
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
  vetor[3]= (byte)(((int)(acaoA)) & 0x00FF);
  vetor[4]= (byte)(((int)(acaoB)) & 0x00FF);
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
  Serial.print(erro_angle[0]); Serial.print('\t');
  Serial.print(angle[0]); Serial.print('\t');
  Serial.print(acaoA); Serial.print('\t');
  Serial.print(acaoP_angle_A); Serial.print('\t');
  Serial.print(acaoI_angle_A); Serial.print('\t');
  Serial.print(acaoD_angle_A); Serial.print('\n');
  #endif
}

// ===========================================================
// --- Interrupção ---
ISR(TIMER2_COMPA_vect)
{
  digitalWrite(7,HIGH);
  flagInterrupcao = 1;
  TCNT2 = T2_init;      //reinicializa TIMER1

  RX();
  TX();
  
  #if controleAltitude
  controlAltitude();
  #else
  acao_altitude = 50;
  #endif
  
  #if controleAngulo
  controlAngle();
  #else
  acao_angle = 0;
  #endif

  acaoA = acao_altitude + acao_angle_A;
  acaoB = acao_altitude - acao_angle_B;
  
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
  digitalWrite(7,LOW);
} //end ISR

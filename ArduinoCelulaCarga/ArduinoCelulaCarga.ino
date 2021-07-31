#define SOP 255
#define Ts 20000
#define SecondsT 0.02
#define valorInicial 700
#define python 1

#define Kcc 0.5
#define valorEquilibrio 620
#define massa 0.200
#define gravidade 9.8

uint32_t Tant;
//int Ts = 10000; //ms

//Butterworth fc = 5Hz  fs= 100Hz ordem 3
// a = 1.0000   -2.3741    1.9294   -0.5321
// b = 0.0029    0.0087    0.0087    0.0029
//y[3]= 0.0029*x[3]+0.0087*x[2]+0.0087*x[1]+0.0029*x[0];
//y[3]= y[3]+2.3741*y[2]-1.9294*y[1]+0.5321*y[0];

//Butterworth fc = 5Hz  fs= 50Hz ordem 3
// a = 1.0000   -1.7600    1.1829   -0.2781
// b = 0.0181    0.0543    0.0543    0.0181
//y[3]= 0.0181*x[3]+0.0543*x[2]+0.0543*x[1]+0.0181*x[0];
//y[3]= y[3]+1.7600*y[2]-1.1829*y[1]+0.2781*y[0];

//Butterworth fc = 10Hz fs= 50Hz ordem 3
// a = 1.0000   -0.5772    0.4218   -0.0563
// b = 0.0985    0.2956    0.2956    0.0985
// a[0]*y[3] = b[0]*x[3]+ b[1]*x[2]+ b[2]*x[1]+ b[3]*x[0] -a[1]*y[2] -a[2]*y[1] -a[3]*y[0];

float x[4]={valorInicial,valorInicial,valorInicial,valorInicial};
float y[4]={valorInicial,valorInicial,valorInicial,valorInicial};

int celCargaFiltrada = 700;

float a_y = 0;
float veloc_y = 0;
float pos_y = 0;
bool decolou = 0;


int led1 = 3; // Porta onde o led será inserido
int leitura;
int valorRealSensor = 0;
int potenciometro;
void setup(){
  Serial.begin(115200);
  pinMode(led1, OUTPUT); // Porta onde o led será inserido, configurado como saida
  pinMode(13, OUTPUT);
  while(!RX()); //Travando o script para rodar o loop apenas quando o primeiro bit for recebido
  Tant = micros();
}
void loop(){
  while(micros() - Tant < Ts){
    //Travando o script pra só executar depois do tempo de amostragem
  }
  Tant = micros();

  potenciometro = analogRead(5);
  valorRealSensor = analogRead(0);
  x[0]=x[1];
  x[1]=x[2];
  x[2]=x[3];
  x[3]=valorRealSensor;
  y[0]=y[1];
  y[1]=y[2];
  y[2]=y[3];
  y[3]= 0.0029*x[3]+0.0087*x[2]+0.0087*x[1]+0.0029*x[0];
  y[3]= y[3]+2.3741*y[2]-1.9294*y[1]+0.5321*y[0];
  
  celCargaFiltrada = y[3];
  gettingPosition();
  
  TX();
  RX();
  if(leitura == 49){
    digitalWrite(led1, HIGH); // Liga a porta 13 se o valor recebido for 1
  }
  else if(leitura == 50){
    digitalWrite(led1, LOW); // Desliga a porta 13 se o valor recebido for 2
  }
  
  /*if(valorRealSensor>500){
    TX();  
  }*/
  
}

bool RX(){
  if(Serial.available()>0){
    leitura = Serial.read(); // Variavel que receberá os valores enviados pelo programa em python
    return 1;
  }
  return 0;
}

void TX(){
  byte vetor[3];
  vetor[0]= (byte)SOP;
  vetor[1]= (byte)((((uint16_t)pos_y) >> 8) & 0x00FF);
  vetor[2]= (byte)(((uint16_t)pos_y) & 0x00FF);
  vetor[1]= (byte)((((uint16_t)potenciometro) >> 8) & 0x00FF);
  vetor[2]= (byte)(((uint16_t)potenciometro) & 0x00FF);
  //vetor[1]= (byte)((0x3E8 >> 8) & 0x00FF);
  //vetor[2]= (byte)(0x3E8 & 0x00FF);
  //Serial.flush();
  #if python
  Serial.write((uint8_t*)vetor,3);
  #else
  /*Serial.print(pos_y);Serial.print("\t");
  Serial.print(veloc_y);Serial.print("\t");
  Serial.print(a_y);Serial.print("\t");
  Serial.println(celCargaFiltrada);*/
  #endif
}

void gettingPosition(){
  a_y = ((Kcc*(valorEquilibrio - celCargaFiltrada)) - massa) * gravidade /massa;
  if(a_y > 0){
    decolou = 1;
  }
  if(decolou == 0){
    a_y = 0;
  } else {
    pos_y = pos_y + (veloc_y*SecondsT) + (a_y*SecondsT*SecondsT/2);
    veloc_y = veloc_y + a_y*SecondsT;
  }
  
}

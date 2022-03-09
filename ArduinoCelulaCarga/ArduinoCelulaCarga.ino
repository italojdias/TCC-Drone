// --- Constantes Timers Interrupcao ---
const uint16_t T1_init = 0;
const uint16_t T1_comp = 625; //625 é a constante que equivale a uma interrupção de 10ms

#define SOP 255
#define SecondsT 0.01
#define valorInicial 700 //Valor inicial para iniciar o filtro de butterworth
#define python 1 

#define Kcc 0.000590 //Constante encontrada de forma experimental
int valorEquilibrio = 620; //Valor inicial, mas na função setup esse valor é recalculado
#define massa 0.200
#define gravidade 9.8

uint32_t Tant;

float x[4]={valorInicial,valorInicial,valorInicial,valorInicial};
float y[4]={valorInicial,valorInicial,valorInicial,valorInicial};

int celCargaFiltrada = 700;

float a_y = 0;
int a_y_int = 0; //a_y_int= a_y * 100
float veloc_y = 0;
float pos_y = 0;
float delta_pos_y =0;
int pos_y_cm = 0;
bool decolou = 0;


int led1 = 13; // Porta onde o led será inserido
int leitura;
int valorRealSensor = 0;

void setup(){
  Serial.begin(115200);
  pinMode(led1, OUTPUT); // Porta onde o led será inserido, configurado como saida
  int i = 0;
  valorEquilibrio = 0;
  Tant = millis();
  while(!RX()) //Travando o script para rodar o loop apenas quando o primeiro bit for recebido
  { //Setup para descobrir o valor da tara da balança
    if(millis()-Tant> 250){
      Tant = millis();
      valorEquilibrio += analogRead(0);
      i++;
    }
  }
  valorEquilibrio = valorEquilibrio/i;
  x[0]=valorEquilibrio;
  x[1]=valorEquilibrio;
  x[2]=valorEquilibrio;
  x[3]=valorEquilibrio;
  
  //Configurando Interrupcao
  //Modo de Comparação
  TCCR1A = 0;
  //Prescaler 1:256
  TCCR1B |=  (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  //Inicializa Registradores
  TCNT1 = T1_init;
  OCR1A = T1_comp;
  //Habilita Interrupção do Timer1
  TIMSK1 = (1 << OCIE1A); 
}
void loop(){
  //Não faz nada / Toda lógica está dentro da função de interrupçãp
}

bool RX(){
  if(Serial.available()>0){
    leitura = Serial.read(); // Variavel que receberá os valores enviados pelo programa em python
    return 1; // Se o arduino receber algum byte via serial
  }
  return 0; // Se o arduino NÃO receber nenhum byte via serial
}

void TX(){
  byte vetor[5];
  //Preenchendo o buffer da mensagem para enviar pela serial
  vetor[0]= (byte)SOP;
  vetor[1]= (byte)((((uint16_t)pos_y_cm) >> 8) & 0x00FF);
  vetor[2]= (byte)(((uint16_t)pos_y_cm) & 0x00FF);
  vetor[3]= (byte)((((uint16_t)a_y_int) >> 8) & 0x00FF);
  vetor[4]= (byte)(((uint16_t)a_y_int) & 0x00FF);
  #if python
  Serial.write((uint8_t*)vetor,5);
  #else
  Serial.print(pos_y);Serial.print("\t");
  Serial.print(veloc_y);Serial.print("\t");
  Serial.print(a_y);Serial.print("\t");
  Serial.println(celCargaFiltrada);
  #endif
}

void gettingPosition(){
  //Convertendo o valor analógico da célula de carga em aceleração
  a_y = ((Kcc*(valorEquilibrio - celCargaFiltrada)) - massa) * gravidade /massa;
  if(a_y > 0){ //Detectando se o drone já teria decolado
    decolou = 1;
  }
  if(decolou == 0){ //Posição só começa a variar quando a aceleração resultante é positiva
    a_y = 0;
  } else {
    delta_pos_y = (veloc_y*SecondsT) + (a_y*SecondsT*SecondsT/2); //Equação horária da posição
    pos_y = pos_y + delta_pos_y;
    veloc_y = veloc_y + a_y*SecondsT; //Equação horária da velocidade
  }
  a_y_int = ((int)(a_y*100));
  pos_y_cm = ((int)(pos_y*100));
}

// ======================================================================================================
// --- Interrupção ---
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = T1_init;      //reinicializa TIMER1
  valorRealSensor = analogRead(0); //Lendo o valor atual da célula de carga
  
  //Filtrando a célula de carga usando um Butterworth com as seguintes características
  //Butterworth fc = 5Hz  fs= 100Hz ordem 3
  // a = 1.0000   -2.3741    1.9294   -0.5321
  // b = 0.0029    0.0087    0.0087    0.0029
  x[0]=x[1];
  x[1]=x[2];
  x[2]=x[3];
  x[3]=valorRealSensor;
  y[0]=y[1];
  y[1]=y[2];
  y[2]=y[3];
  y[3]= (0.0029*x[3])+(0.0087*x[2])+(0.0087*x[1])+(0.0029*x[0]);
  y[3]= y[3]+(2.3741*y[2])-(1.9294*y[1])+(0.5321*y[0]);
  
  celCargaFiltrada = y[3]; //butterworth
  gettingPosition(); //Atualizando a variável com o valor da posição atual
  
  TX();
  RX();
  if(leitura == 49){
    digitalWrite(led1, HIGH); // Liga a porta 13 se o valor recebido for 1
  }
  else if(leitura == 50){
    digitalWrite(led1, LOW); // Desliga a porta 13 se o valor recebido for 2
  }
} //end ISR

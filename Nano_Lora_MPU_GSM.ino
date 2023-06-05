// Programa: Acelerometro com ESP8266 NodeMCU
/* 
 *  Adaptado do desenvolvido pelo FilpeFlop https://www.filipeflop.com/blog/acelerometro-com-esp8266-nodemcu
 */

#include <Wire.h>         // biblioteca de comunicação I2C
#include<SoftwareSerial.h> 
#include <String.h>
#include <math.h>
//========SoftwareSerial==================
SoftwareSerial gprsSerial(2, 3);  //D2, D3 : Será usada a serial por software para deixar livre a serial do monitor serial


/********************************
 * 
 * variaveis GSM
 *  
 */

int u1 = 0; 
int u2 = 0;
int u3 = 0;
int mpu1 = 1600;  
int mpu2 = 1600;
int mpu3 = 1600;
int temp = 1600;

int retorno=0;
int Pos1, Pos2, ContPos, Espera;
int RST_PIN=10;
long PeriodoDeEnvioGSM;  // quantidade de milisegundos entre envios do GSM. 1.800.000ms sao 30min

float RX_Pwr;

String str, Resposta;


 /*************************
  * ********************/
/*
 * Definições de alguns endereços mais comuns do MPU6050
 * os registros podem ser facilmente encontrados no mapa de registros do MPU6050
 */
const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro
 
bool led_state = false;   //Definir estado de LED do Arduino nano
bool first_time =true;    // Variavel para verificar se é a primeira vez que esta sendo ligado
bool leitura = true;
  

unsigned long millisTarefa1 = millis();    // Variavel para o tempo de envio de sinal
unsigned long delta_tempo = 3000000;
const unsigned long timeGetInfo =10000;  //Tempo para puxar dados do sensor

String postStr;   //Variavel global com o texto que será enviado.

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, angx, angy, angz;



void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);        // funcao apra fazer o resset pelo pino RST_PIN=7
  pinMode(A0,INPUT); //Sensores Umidade
  pinMode(A1,INPUT); //Sensores Umidades 1
  pinMode(A2,INPUT); //Sensores Umidades 2
  Serial.begin(9600);
  delay(100);
  Serial.println("Iniciando configuração do MPU6050");
  gprsSerial.begin(9600);
  Serial.println(F("InitI2C"));
  initI2C(); //Inicia comunicação I2C
  Serial.println(F("InitMPU"));
  initMPU(); // Inicia o MPU
    Serial.println(F("CheckADDR"));
  checkMPU(MPU_ADDR); // Verifica o MPU
 
  Serial.println("Configuração finalizada, iniciando GSM");  
  // Resset o SIM800
  digitalWrite(RST_PIN, LOW);    // reset the SIM800  making the voltage LOW in reset pin
  delay(2000); 
  digitalWrite(RST_PIN, HIGH);   // set the SIM800  making the voltage HIGH in reset pin
  delay(15000);                  // Espera 15 segundos para a CLARO ser reconhecida no SIM800
                                 // Deve ser alterado por algum teste que indique que o SIM800 registrou na rede

//gprsSerial.println("AT");
  gprsSerial.println("AT+COPS?"); // Display the current network operator
  delay(1000);
  ShowSerialData();    
  
  gprsSerial.println("AT+CMEE=2"); // Habilita report de erros. Com o CMEE=2 é habilitado 
  delay(1000);                     // o codigo de erro e o uso de palavas que identificam o erro
  ShowSerialData();   
}
void loop()
{
  
  
  atualiza_delta_tempo();
  

  if ( leitura && delta_tempo>timeGetInfo)
  {
        
        ler_sensores();
        criarTextoParaEnviar();
        millisTarefa1 = millis();
        mpu1 = AcX;
        mpu2 = AcY;
        mpu3 = AcZ;
        temp = (Tmp/340.00+36.53)*100;
        Serial.println(F("Enviando dados"));
        loopGSM();
        Serial.println(F("Saiu loopGSM"));
        piscaLed(3); //Pisca 3 vezes depois de enviar os dados
        delay(7000);
        leitura = true;
  }
  
  delay(1000);
  }



  /*****************************************
   * 
   * *************FUNÇÕES*******************
   * 
   * **************************************/


void ShowSerialData(){
  while(gprsSerial.available()!=0){ 
    Serial.write(gprsSerial.read());    
    }
  delay(3000);
  Serial.println();
}

void ShowFullSerialData(){
  str="";           //Armazena todo o comando  e a resposta
  Resposta="";       //Armazena somente a resosta que esta entre os caracteres Pos1 e Pos2
  ContPos=0;         //conta a posicao dos caracteres que estao chegando pela porta serial
  while(gprsSerial.available()!=0){ 
    retorno= gprsSerial.read();
    ContPos=ContPos+1;
    str=str+(char)retorno;
    if(Pos1<=ContPos && ContPos<=Pos2){
      Resposta=Resposta+(char)retorno;
    }
    Serial.write(retorno); 
    delay(Espera);   
    Serial.print(retorno, DEC);  
    }
  delay(Espera*2);
  Serial.println();
  Serial.print(str);
  Serial.println();
  Serial.print(Resposta);
  Serial.println();
}


void loopGSM() {
  // put your main code here, to run repeatedly:
  //PeriodoDeEnvioGSM=3600000;  // 60 min
  PeriodoDeEnvioGSM=600000;  // 10 min
  //PeriodoDeEnvioGSM=1800000;  // 30 min
  //PeriodoDeEnvioGSM=300000;   // 5 min 
  //PeriodoDeEnvioGSM=120000;   // 2 min 
  delay(1000);    
  
  EnviaDadosViaGSM();
}


void EnviaDadosViaGSM(){
 gprsSerial.println("AT");
  delay(1000);
  //ShowSerialData();
          //------Inicio do ShowSerialData
          while(gprsSerial.available()!=0){ 
          Serial.write(gprsSerial.read());    
          }
          delay(3000);
          Serial.println();
          //------FIm do ShowSerialData
  delay(1000);

  gprsSerial.println("AT+CPIN?");  //Indica se uma senha sera requerida. READY significa que nao requer senha
  delay(1000);
  ShowSerialData();
  delay(1000);

  gprsSerial.println("AT+CSQ");  // Indica a qualidade do sinal. Entre 2 e 30 significa potencia entre -110 e -54dBm
                                 // potencia= (resposta-2)*2 - 110 dBm
  delay(1000);
  
  ShowSerialData();
  // Colocar aqui um print da potencia RX_Pwr=(resposta-2)*2 - 110
  delay(1000);

  gprsSerial.println("AT+CREG?");  //Network Registration . Resposta +CREG: X, Y - X nao importa muito. Y=1 significa que o SIM800 esta registrado na rede.
  delay(1000);
  ShowSerialData();
  delay(1000);

  gprsSerial.println("AT+CGATT?"); //Conectar ou desconectar do serviço de GPRS. Resposta é +CGATT: X. Se X=0 esta desconetado, se X=1 esta conectado.
  delay(1000);
  ShowSerialData();
  delay(1000);

  gprsSerial.println("AT+CSTT?"); // Verifica se o APN. usuario e senha estao definidos
  delay(3000);
  //ShowSerialData();
   Resposta="";
   Espera=1;
   Pos1=20;  //posicao do caracter inicial da resposta
   Pos2=24;  //posicao do caracter final  da resposta
   ShowFullSerialData();
  delay(1000);

  // Aqio fazer um teste para ver se a resposta é CMNET. Se
  // resposta==CMNET, entao aplicar em CSTT o APN, login e senha. Senao
  // pular os comandos AT+CSTT= e AT+CIICR.
  if(Resposta=="CMNET"){ 
              gprsSerial.println("AT+CSTT=\"inlog.claro.com.br\", \"datatem\", \"datatem\""); // SETA o APN. usuario e senha 
              //gprsSerial.println("AT+CSTT=\"datatem.tim.br\", \"tim\", \"tim\""); // SETA o APN. usuario e senha 
              delay(3000);
              ShowSerialData();
              delay(1000);

              gprsSerial.println("AT+CIICR");  //Abre a conexão sem fio com GPRS
              delay(3000);
              ShowSerialData();
              delay(1000);

              gprsSerial.println("AT+CIFSR");  //Obtem o valor do IP da conexão
              delay(3000);
              ShowSerialData();
              delay(1000);
          }


  

  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");
  delay(3000);
  ShowSerialData();
  delay(1000);

  gprsSerial.println("AT+CREG?");  //Para testar se o SIM800 esta registrado na rede antes do comendo CIPSEND
  delay(1000);
  ShowSerialData();
  delay(1000);

  gprsSerial.println("AT+CIPSTATUS");  //Consulta o status da conexão atual. A resosta é OK STATE:X sEsera-se que  X=CONNECT OK
  delay(1000);
  ShowSerialData();
  delay(1000);
  
  Resposta="";
  gprsSerial.println("AT+CIPSEND");      //begin send data to remote server      
          delay(3000);
          Espera=1;
          Pos1=14;  //posicao do caracter inicial da resposta
          Pos2=14;  //posicao do caracter final  da resposta
          ShowFullSerialData();
          
  if(Resposta==">"){ 
            //GET https://api.thingspeak.com/update?api_key=09YNJD9P01Z7ZNSU&field1=0
            //String str="GET https://api.thingspeak.com/update?api_key=NKRNDHMOOAXH3GEF&field1=" + String(t) ;
            String str="GET https://api.thingspeak.com/update?api_key=6VTFWN2QSYC3NI1G&field1=" + String(u1) + "&field2=" + String(u2) + "&field3=" + String(u3) + "&field4=" + String(mpu1) + "&field5=" + String(mpu2) + "&field6=" + String(mpu3) + "&field7=" + String(temp);
            gprsSerial.println(str);     //begin send data to remote server
            delay(4000);
            ShowSerialData();
            
            //Aqui fazer um teste do recebimento do SEND OK
            gprsSerial.println((char)26);   //CTRL+Z envia dados e fecha a conexão TCP
            delay(8000);                    //waitting for reply, important! the time is base on the condition of internet 
            gprsSerial.println();
            //ShowSerialData();
            Resposta="";
            Espera=1;
            Pos1=6;  //posicao do caracter inicial da resposta
            Pos2=12;  //posicao do caracter final  da resposta
            ShowFullSerialData();

            gprsSerial.println("AT+CIFSR");  //Obtem o valor do IP da coexão
            delay(3000);
            //ShowSerialData();
            Resposta="";
            Espera=1;
            Pos1=12;  //posicao do caracter inicial da resposta
            Pos2=27;  //posicao do caracter final  da resposta
            ShowFullSerialData();
            
            delay(1000);

            gprsSerial.println("AT+CIPSTATUS");      //CLose the TCP connection    
            delay(3000);
            ShowSerialData();

            //delay(300000);   // 5 minutos
            //delay(1800000);   // 30 minutos 
            Serial.println(PeriodoDeEnvioGSM);
            Serial.println();
            delay(PeriodoDeEnvioGSM);           
          }
  else{
            digitalWrite(RST_PIN, LOW);    // reset the SIM800  making the voltage LOW in reset pin
            delay(2000); 
            digitalWrite(RST_PIN, HIGH);   // set the SIM800  making the voltage HIGH in reset pin
            delay(15000);                  // Espera 15 segundos para a CLARO ser reconhecida no SIM800
                                 // Deve ser alterado por algum teste que indique que o SIM800 registrou na rede
      }
}
   /*************************
    * Funções NÃO GSM
    ************************/
void piscaLed(int ntimes)
{
  for (int i = 0; i < ntimes; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
void ler_sensores()
{
        readRawMPU();    // lê os dados do sensor e os almacena nas variaves globais
        checkMPU(MPU_ADDR);
        readUmidade(); // lê os dados do sensor e os almacena nas variaves globais

}
void atualiza_delta_tempo()
{

if (first_time == true)
  {
    delta_tempo = timeGetInfo*10;
    
    Serial.println(F("Inicio leitura local (sem Lora) *************"));
    ler_sensores();
    criarTextoParaEnviar(); 
    Serial.println(F("Fim leitura local (sem Lora)*************"));
    first_time=false;
    
  }
  else
  {
    delta_tempo = millis() - millisTarefa1;
  }
  
}


void initI2C()   //Inicializa o I2C para o sensor de aceleração
{
  //Serial.println("---inside initI2C");
  //Wire.begin(sda_pin, scl_pin);
  Wire.begin();  //Usa os portos padroes do Nano 
}
 
/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int16_t reg, int16_t val)      //aceita um registro e um valor como parâmetro
{
  int retorno;
  int nbyte;
  Serial.println(F("writeREGMPU"));
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Serial.println(F("After begin transmission"));
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
    Serial.println(F("After first write"));
  nbyte = Wire.write(val);                      // escreve o valor no registro
  Serial.print(F("Nbyte escritos = "));
  Serial.print(nbyte);
  Serial.println(F(" - After second write"));
  retorno = Wire.endTransmission(true);           // termina a transmissão
  Serial.print(F("endTransmission =  "));
  Serial.print(retorno);
  Serial.println(F("After end transmission"));

}
 
/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}
 
/*
 * função que procura pelo sensor no endereço 0x68
 */
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
 
  if(data == 0)
  {
    Serial.print(F("Dispositivo encontrado no endereço: 0x"));
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println(F("Dispositivo não encontrado!"));
  }
}
 
/*
 * função que verifica se o sensor responde e se está ativo
 */
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);
     
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(data == 104||data==152) 
  {
    Serial.println(F("MPU6050 Dispositivo respondeu OK! (104)"));
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64) Serial.println(F("MPU6050 em modo SLEEP! (64)"));
    else Serial.println(F("MPU6050 em modo ACTIVE!"));
  }
  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
  Serial.print("Data Recuperada=");
  Serial.println(data);
  
}
 
/*
 * função de inicialização do sensor
 */
void initMPU()
{
  setSleepOff();
  
  setGyroScale();
  setAccelScale();
}
 
/* 
 *  função para configurar o sleep bit  
 */
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
 
/* função para configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s
 
    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}
 
/* função para configurar as escalas do acelerômetro
   registro da escala do acelerômetro: 0x1C[4:3]
   0 é 250°/s
 
    AFS_SEL   Full Scale Range
      0           ± 2g            0b00000000
      1           ± 4g            0b00001000
      2           ± 8g            0b00010000
      3           ± 16g           0b00011000
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}
 
/* função que lê os dados 'crus'(raw data) do sensor
   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:
 
  0x3B 59 ACCEL_XOUT[15:8]
  0x3C 60 ACCEL_XOUT[7:0]
  0x3D 61 ACCEL_YOUT[15:8]
  0x3E 62 ACCEL_YOUT[7:0]
  0x3F 63 ACCEL_ZOUT[15:8]
  0x40 64 ACCEL_ZOUT[7:0]
 
  0x41 65 TEMP_OUT[15:8]
  0x42 66 TEMP_OUT[7:0]
 
  0x43 67 GYRO_XOUT[15:8]
  0x44 68 GYRO_XOUT[7:0]
  0x45 69 GYRO_YOUT[15:8]
  0x46 70 GYRO_YOUT[7:0]
  0x47 71 GYRO_ZOUT[15:8]
  0x48 72 GYRO_ZOUT[7:0]
    
*/
void readRawMPU()
{  
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 

  angx = atan(AcZ/AcY);
  angy = atan(AcZ/AcX);
  angz = atan(AcY/AcX);


                             
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //Equação disponibilizada pelo fabricante
  Serial.print(" | TmpCrua = "); Serial.print(Tmp); //Equação disponibilizada pelo fabricante
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | Ângulo X = "); Serial.print(angx);
  Serial.print(" | Ângulo Y = "); Serial.print(angy);
  Serial.print(" | Ângulo Z = "); Serial.println(angz);

  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor
  delay(50);                                        
}

void readUmidade()
{
  u1 = readSensor(A0);
  u2 = readSensor(A1);
  u3 = readSensor(A2);
}

int readSensor(int pin) { //Read 1
  int sensorValue = 0;
  int totalu = 0;
  int ntimes = 10;
  // lê o sensor 10 vezes
  for (int i = 0; i < ntimes; i++) {
    sensorValue = analogRead(pin);
    totalu += sensorValue;
    delay(5);
  }

  // calcula a média
  return totalu / ntimes;
}

void criarTextoParaEnviar()
  {  
Serial.println("*******************************");
postStr = String(AcX);
postStr += "/";
postStr += String(AcY);
postStr += "/";
postStr += String(AcZ);
postStr += "/";
postStr += String(round((Tmp/340.00+36.53)*100));
postStr += "/";
postStr += String(u1);
postStr += "/";
postStr += String(u2);
postStr += "/";
postStr += String(u3);
Serial.println(postStr);
  }

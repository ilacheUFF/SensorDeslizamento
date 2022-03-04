// Programa: Acelerometro com ESP8266 NodeMCU
/* 
 *  Adaptado do desenvolvido pelo FilpeFlop https://www.filipeflop.com/blog/acelerometro-com-esp8266-nodemcu
 */
//#include <ESP8266WiFi.h> //Biblioteca do ESP8622
#include <Wire.h>         // biblioteca de comunicação I2C
#include<SoftwareSerial.h> 
#include <math.h>
//========SoftwareSerial==================
SoftwareSerial SWSerial2(2, 3);  //D2, D3 : Será usada a serial por software para deixar livre a serial do monitor serial
 
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
 

//const int sda_pin = D2; // definição do pino I2C SDA
//const int scl_pin = D1; // definição do pino I2C SCL
 
bool led_state = false;
bool first_time =true;

unsigned long millisTarefa1 = millis();
const unsigned long timeGetInfo =4000;  //Tempo para puxar dados do sensor

String postStr;

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, angx, angy, angz;
int u1,u2,u3 ; //Leituras das umidades

//int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 
  


 
/*
 * função que configura a I2C com os pinos desejados 
 * sda_pin -> D2
 * scl_pin -> D1
 */
void initI2C() 
{
  //Serial.println("---inside initI2C");
  //Wire.begin(sda_pin, scl_pin);
  Wire.begin();  //Usa os portos padroes do NODE D1=SCL D2=SDA
}
 
/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int16_t reg, int16_t val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão

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
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
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
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
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

  postStr = String(AcX);
        postStr +="/";
        postStr += String(AcY);
        postStr +="/";
        postStr += String(AcZ);
        postStr +="/";
        postStr += String(round((Tmp/340.00+36.53)*100));
        postStr +="/";
                             
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
  Serial.println("*******************************");
  Serial.println(postStr);
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor
  delay(50);                                        
}

void readUmidade()
{
  u1=analogRead(A0);
  u2=analogRead(A1);
  u3=analogRead(A2);
  postStr += String(u1);
  postStr +="/";
  postStr += String(u2);
  postStr +="/";
  postStr += String(u3);
  Serial.println(postStr);
  }

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0,INPUT); //Sensores Umidade
  pinMode(A1,INPUT); //Sensores Umidades 1
  pinMode(A2,INPUT); //Sensores Umidades 2
  Serial.begin(9600);
  Serial.println("nIniciando configuração do MPU6050n");
  SWSerial2.begin(9600);
  
  initI2C();
  initMPU();
  checkMPU(MPU_ADDR);
 
  Serial.println("nConfiguração finalizada, iniciando loopn");  
}
void loop()
{
  if (((millis() - millisTarefa1)>timeGetInfo) || (first_time==true))
  {
        readRawMPU();    // lê os dados do sensor
        checkMPU(MPU_ADDR);
        readUmidade();
        first_time=false;

        millisTarefa1=millis();
        Serial.println("Enviando2");
        postStr +="/D";
        SWSerial2.print(postStr);
  }
  delay(1000);
  }

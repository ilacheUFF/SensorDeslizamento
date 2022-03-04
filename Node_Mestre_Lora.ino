//=======================================================================================//
/*  Modificado por Ivanovich 17/06.  Liga uma saida do RAdio e le a entrada analogica, desliga a saida do radio
 
 Radioenge Equip. de Telecom.
 Abril de 2020
 Código exemplo para uso dos módulos LoRaMESH com o ESP8266 + ThingSpeak
 
 Fontes consultadas:
https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/ 08-04-2020
https://stackoverflow.com/questions/5697047/convert-serial-read-into-a-useable-string-using-arduino //08/04/2020
https://www.filipeflop.com/blog/esp8266-com-thingspeak/ //13-04-2020

 */



#include <ESP8266WiFi.h>
//#include <Ticker.h>
#include<SoftwareSerial.h> 
//========SoftwareSerial==================
SoftwareSerial SWSerial(4, 5);  //D2, D1 : Será usada a serial por software para deixar livre a serial do monitor serial
SoftwareSerial SWSerial2(12, 14);  //D6, D5 : Será usada a serial por software para deixar livre a serial do monitor serial
//SoftwareSerial SWSerial2(2, 3);  //D6, D5 : Será usada a serial por software para deixar livre a serial do monitor serial
//===============Timer====================
//Ticker blinker;

/* defines - wi-fi */
//#define SSID_REDE "Federal 2014" /* coloque aqui o nome da rede que se deseja conectar */
//#define SENHA_REDE "bucaramanga" /* coloque aqui a senha da rede que se deseja conectar */
#define SSID_REDE "ENG_TER_2G" /* coloque aqui o nome da rede que se deseja conectar */
#define SENHA_REDE "engenharia235" /* coloque aqui a senha da rede que se deseja conectar *


//#define INTERVALO_ENVIO_THINGSPEAK 60000 /* intervalo entre envios de dados ao ThingSpeak (em ms) */

#define LED_PLACA 16
 
 int dadosSensores[7]; //São 7 dados, 3 acelerações 1 temperatura 3 umidades
/* constantes e variáveis globais */
char endereco_api_thingspeak[] = "api.thingspeak.com";
String chave_escrita_thingspeak = "DMU34DLNR83EQP4G";  /* Coloque aqui sua chave de escrita do seu canal */
unsigned long last_connection_time;
bool first_time=true;

//constantes e variáveis globais
unsigned long millisTarefa1 = millis();
const unsigned long timeGetInfo =4000;  //Tempo para puxar dados do sensor
char EnderecoAPIThingSpeak[] = "api.thingspeak.com";
//String ChaveEscritaThingSpeak = "6S9XXXXXXXXXX5T4"; //Write API Key da sua conta no ThingSpeak.com


long lastConnectionTime; 
//WiFiClient client;
int count = 0;
char inData[255]; // Buffer com tamanho suficiente para receber qualquer mensagem
char inChar=-1; // char para receber um caractere lido
byte indice = 0; // índice para percorrer o vetor de char = Buffer
const int nRadios=5;
int listaEndDevices[nRadios] = {2,2,2,2,2};//pode aumentar o tamanho do arry e incluir um novo ID. coloquei 1,2,1,2,1 ... porque no teste só temos os IDs 1 e 2 ... em uma rede maior ... deve-se cololcar os IDs existentes 1,2,3,50,517, 714,... qualquer inteiro de 1 até 1023.
int polingID=0;//usado para escolher qual ID do vetor acima será usado.
char flag= false; //flag usada para avisar que a serial recebeu dados
char comandos = 0;//usado para escolher qual comando enviar.
char listaGPIO[]={5,0}; // usado para ler as GPIOs que estão neste vetor ... pode-se ler da 0 até 7
char mensaje='a';
long valorAD;
//===============================================================================
////Timer : Quando este timer "estourar" ... o led mudará seu estado e será incrementado o tempo usado no polling.
//void ICACHE_RAM_ATTR onTimerISR(){
//    digitalWrite(LED_BUILTIN,!(digitalRead(LED_BUILTIN)));  //Toggle LED Pin
//    timer1_write(600000);//12us
//    count++;
//}
//================================================================================

// ===================== protótipos das funções  =================================
char lerSerial();
//char TrataRX(byte indice);
void TrataRX(byte indice);
void escreve(int contador);
char LerGPIO(int id, char gpio);
char SetGPIO(int id, char gpio, char nivel);
uint16_t CalculaCRC(char* data_in, uint32_t length);
void EnviarSerial(int id, char mensaje, int muda);
void lerRadio();
void envia_informacoes_thingspeak(String string_dados);
//================================================================================


void setup() {

  Serial.begin(9600);
  SWSerial.begin(9600);
  SWSerial2.begin(9600);

  last_connection_time = 0;
 
    /* Inicializa sensor de temperatura e umidade relativa do ar */
 
    /* Inicializa e conecta-se ao wi-fi */
  //init_wifi();
    

  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output

}




// the loop function runs over and over again forever
void loop() {

if (SWSerial2.available())
{
  delay(20);
  for (int i = 0; i <= 6; i++) {
  dadosSensores[i]=SWSerial2.parseInt();
  Serial.print(dadosSensores[i]);
  //Serial.print(SWSerial2.read());
  Serial.print(":");
  delay(20);
  }
  while(SWSerial2.available()){
  SWSerial2.parseInt();}
}
Serial.println(":");
Serial.println("***************");
delay(500);


}

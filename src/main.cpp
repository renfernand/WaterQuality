// Programa Arduino para ler sensores de qualidade de Agua
// Usage : 
// Sensor Turbidez - Pinos A0, 5V, GND
// Sensor de Temperatura (ds18b20) - PInos (vermelho 5V, Preto-GND, Amarelo - Resistor 4K7 entre amarelo e vermelho, e Amarelo ligado no pino 2 digital)
//        https://curtocircuito.com.br/blog/Categoria%20Arduino/como-utilizar-o-ds18b20
// Sensor de Condutividade: Medidor de TDS Sensor de Condutividade
//        https://www.usinainfo.com.br/blog/projeto-medidor-de-tds-arduino-para-condutividade-da-agua/
// ATENCAO (Sensor de Condutividade)
// 1. A sonda não pode ser usada em água acima de 55 graus centígrados.
// 2. A sonda não pode ser deixada muito perto da borda do recipiente, caso contrário afetará a leitura.
#include <Arduino.h>  
#include <OneWire.h>  
#include <DallasTemperature.h>

#define IMPRIME_SERIAL 0
#define NIVEL_TANQUE_MAX  10   //nivel do tanque para desligar a bomba
#define NIVEL_TANQUE_MIN  13   //nivel do tanque para desligar a bomba

#define RFF  //teste github

// Structure da mensagem para enviar para ESP32
#define HEADER_MSG 0xCA
#define FOOTER_MSG 0xFEFE
typedef struct struct_message {
  uint8_t header;
  float turb_v;   //turbidez voltage
  float turb_ntu; //turbidez 0 a 10000 NTU
  float temp;     //temperature
  float cond_v;   //condutividade voltage
  float cond_tds; //condutividade tds
  int dist_media; //distancia media
  bool bomb_status; //status da bomba
  uint16_t footer;
} struct_message;
struct_message mydata;

 // TECHATRONIC.COM  
 long duration;  
 int distanceCm, distanceInch; 

// DEFINE PINAGENS SENSORES E LEDS
const int pinoLed = 13;
const int Bombapin = 4;
const int dist_trigPin = 9;  
const int dist_echoPin = 10;  

// Pino sensor temperatura DS18B20 (pwm)
#define Pin_Sensor_temp 2 
float temperature;

//sensor de condutividade
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float cond_averageVoltage = 0;
float cond_tdsValue = 0;
//Pino sensor condutividade (analogico)
#define TdsSensorPin A1

// sensor de turbidez 
float turb_voltagem;
float turb_NTU;
// Define o pino de Leitura do Sensor Turbidez (analogico)
int SensorTurbidezPin = A0;

int ligadesliga=0; 
uint8_t count=0;

//sensor de distancia
#define MAX_POS 5
int media_movel[5];
int pos=0;
int timeout=0;

//prototypes
OneWire oneWire(Pin_Sensor_temp);  
DallasTemperature sensors(&oneWire); 

//=============================================
// TRATA SENSOR DE TURBIDEZ

// Sistema de arredondamento para Leitura
float ArredondarPara( float ValorEntrada, int CasaDecimal ) {
  float multiplicador = powf( 10.0f, CasaDecimal );
  ValorEntrada = roundf( ValorEntrada * multiplicador ) / multiplicador;
  return ValorEntrada;
}


//=============================================
// TRATA SENSOR DE CONDUTIVIDADE

int getMedianNum(int bArray[], int iFilterLen)
{
	int bTab[iFilterLen];
	for (byte i = 0; i<iFilterLen; i++)
		bTab[i] = bArray[i];

	int i, j, bTemp;
	for (j = 0; j < iFilterLen - 1; j++){
		for (i = 0; i < iFilterLen - j - 1; i++) {
			if (bTab[i] > bTab[i + 1]) {
				bTemp = bTab[i];
				bTab[i] = bTab[i + 1];
				bTab[i + 1] = bTemp;
			}
		}
	}
	
	if ((iFilterLen & 1) > 0)
		bTemp = bTab[(iFilterLen - 1) / 2];
	else
		bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  
  return bTemp;
}

// =========================================================
// TRATA COMUNICACAO SERIAL
uint8_t senddataserial(uint8_t *pdata, uint8_t len){
  int i;

  for (i=0;i<len;i++){
     Serial.write(*pdata++);
  }

  return 1;
} 


//=============================
// SETUP CONFIG

void setup() {
 Serial.begin(9600);

  //Pino led interno - Somente para Debug
  pinMode(pinoLed, OUTPUT); 

  //Pino Saida Bomba Dagua
  pinMode(Bombapin, OUTPUT); 

  //Leitura sensor temperatura
  sensors.begin(); 

  //sensor de condutividade
  pinMode(TdsSensorPin,INPUT);

  //sensor de distancia
  pinMode(dist_trigPin, OUTPUT);  
  pinMode(dist_echoPin, INPUT); 


}
 

//=============================
// MAIN LOOP

void loop() {
  uint8_t ret=0;
  uint16_t i=0;
 	static unsigned long analogSampleTimepoint = millis();
	static unsigned long printTimepoint = millis();
	

 // leitura sensor temperatura
  sensors.requestTemperatures(); 
  //Serial.println(sensors.getTempCByIndex(0));
  temperature = sensors.getTempCByIndex(0);
  if ((temperature <0) || (temperature >50))
     temperature = mydata.temp;
  mydata.temp = temperature;
 //================================================
 //leitura sensor de condutividade

	if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
	{
		analogSampleTimepoint = millis();
		analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
		analogBufferIndex++;

		if(analogBufferIndex == SCOUNT)
			analogBufferIndex = 0;
	}
	
	if(millis()-printTimepoint > 800U)
	{
		printTimepoint = millis();
		for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
			analogBufferTemp[copyIndex]= analogBuffer[copyIndex];

		cond_averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
	
    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationCoefficient = 1.0 + 0.02 * (temperature-25.0); 

    //temperature compensation
    float compensationVolatge = cond_averageVoltage/compensationCoefficient; 
      
    cond_tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value

    mydata.cond_v = cond_averageVoltage;
    mydata.cond_tds = cond_tdsValue;
  } 

  //======================================
  // leitura sensor turbidez  
  turb_voltagem = 0;
  // Realiza a soma dos "i" valores de voltagem
  for (i = 0; i < 800; i++) {
    turb_voltagem += ((float)analogRead(SensorTurbidezPin) / 1023) * 5;
  }
  
  // Realiza a média entre os valores lidos na função for acima
  turb_voltagem = turb_voltagem / 800;
  turb_voltagem = ArredondarPara(turb_voltagem, 1);
 
  // Se Voltagem menor que 2.5 fixa o valor de NTU
  if (turb_voltagem < 2.0)
    turb_NTU = 3000;
  else if (turb_voltagem > 4.2) {
    turb_NTU = 0;
    turb_voltagem = 4.2;
  }
  else {
    // Senão calcula o valor de NTU através da fórmula
    turb_NTU = -1120.4 * sq(turb_voltagem) + 5742.3 * turb_voltagem - 4353.8;
  }

  mydata.turb_v = turb_voltagem;
  mydata.turb_ntu = turb_NTU;

  //====================================
  // leitura sensor ultrassonico (nivel)
   digitalWrite(dist_trigPin, LOW);  
   delayMicroseconds(2);  
   digitalWrite(dist_trigPin, HIGH);  
   delayMicroseconds(10);  
   digitalWrite(dist_trigPin, LOW);  
   duration = pulseIn(dist_echoPin, HIGH);  
   distanceCm= duration*0.034/2;  
   distanceInch = duration*0.0133/2;  
 
  //=====================================
  //controle de nivel
  //Somente considera valores menores que 50cm...E o valor eh a media movel dos ultimos 5 valores
   int dist_tot=0;
   int dist_media=0;
   int dist_count=0;
   #if 1
   if (distanceCm < 50) {
    timeout =0;
     media_movel[pos] = distanceCm;
     if (pos>=MAX_POS)
       pos=0;
     else  
       pos++;
     
    for (i=0; i < MAX_POS; i++){
      if (media_movel[i] >0){
          dist_tot += media_movel[i];
          dist_count++;
      }
    }
    dist_media = dist_tot / dist_count;
    mydata.dist_media = dist_media;
  
    if (dist_media <= NIVEL_TANQUE_MAX) 
      mydata.bomb_status = 0;
    else {
      if (dist_media > NIVEL_TANQUE_MIN)
         mydata.bomb_status = 1;
    }      
        
    digitalWrite(Bombapin, mydata.bomb_status);

  }
  else{
    timeout++;
    if (timeout>10){
      mydata.bomb_status = 0;
    }
  }
  #else
    mydata.dist_media = distanceCm;
    if (mydata.dist_media > 20) 
      mydata.bomb_status = 1;
      else      
      mydata.bomb_status = 0;
        
    digitalWrite(Bombapin, mydata.bomb_status);
  
  #endif

  //=========================================
  //pisca led - Somente para debug
  if (ligadesliga) 
     ligadesliga = 0;
  else
     ligadesliga = 1;
  digitalWrite(pinoLed, ligadesliga); 
 
  // Write all sensors
  #if IMPRIME_SERIAL
		Serial.print("Temp: ");
		Serial.print(temperature,2);
    //condutividade
		Serial.print(" - Cond: V= ");
		Serial.print(cond_averageVoltage,2);
		Serial.print("TDS Value:");
		Serial.print(cond_tdsValue,0);
		Serial.print(" ppm ");
    //turbidez
		Serial.print(" - Turb: V= ");
		Serial.print(turb_voltagem,2);
		Serial.print("NTU = ");
		Serial.print(turb_NTU,0);
		Serial.print("NTU ");
    //distance
    Serial.print(" Distance= ");  
    //Serial.println(distanceCm);  
    //Serial.print(" Media: ");  
    Serial.println( mydata.dist_media);  
    #endif

  //========================================
  //build message and send to ESP32
  mydata.header = HEADER_MSG;
  mydata.footer = FOOTER_MSG;

  ret = senddataserial((uint8_t *)&mydata,sizeof(mydata));

  delay(300);  
  
  ret = ret;
}
 

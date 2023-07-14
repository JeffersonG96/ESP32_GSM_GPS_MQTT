#include <Arduino.h>

//Librerias
#include <Wire.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include<stdlib.h>
#include <MAX30100_PulseOximeter.h>   //sensor de frecuencia cardiaca 


#include <Adafruit_MLX90614.h>
// #include <Adafruit_Sensor.h>


// TaskHandle_t Task1;

#define SerialAT Serial1

#include <TinyGPSPlus.h> //https://github.com/mikalhart/TinyGPSPlus
// #include <TinyGsmClient.h> //https://github.com/vshymanskyy/TinyGSM

#define TINY_GSM_MODEM_SIM800

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Your GPRS credentials, if any
const char apn[]      = "intern.claro.com.ec";
const char gprsUser[] = "claro";
const char gprsPass[] = "claro";

//GSM Module Settings
//GSM Module RX pin to ESP32 2
//GSM Module TX pin to ESP32 4
#include <TinyGsmClient.h>
#define rxPin 4
#define txPin 2
HardwareSerial sim800(1);
TinyGsm         modem(sim800);

//Enviar los datos de mqtt por gsm 
TinyGsmClient mqttClient(modem);
PubSubClient client(mqttClient);

//GPS Module Settings
//GPS Module RX pin to ESP32 17
//GPS Module TX pin to ESP32 16
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;

//sensor de frecuencia y spo2
PulseOximeter pox;  //instancia de la librería 

int counterHS = 2;
float getHeart=60.0;
float getSpo2=95.0;
float averageHeart =0.0;
float averageSpo2 =0.0;
float send_data_max30100_heart=0.0;
float send_data_max30100_spo2=0.0;

//Variables para el sensor MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int tempCounter=100;
float averageTemp = 20;
float send_data_temperature = 0;

//VARIABLES SERVIDOR 
//TODO: variables http  
String dId = "2023";
String dIdpassword = "caa4KsQ5ac";
String webhook_endpoint = "http://20.125.99.178:3001/api1/getdevicecredentials";
const char* mqttServer = "20.125.99.178";
int mqttPort = 1883;
String userName = "root";
String password = "Ambato_Avanzadas2023?_%.";


//Variables globales
long lastReconnect = 0;
DynamicJsonDocument mqttDataDoc(2048);
DynamicJsonDocument LatLon(526);
double latitud = 0;
double longitud = 0;
float send_latitud = 0;
float send_longitud = 0;
long lastsend = 5000;

#define REPORTING_PERIOD_MS 900
uint32_t tsLastReport = 0;
MAX30100 mx;

//*FUNCIONES 
void ConectGPRS();
void verifyGPRS();
bool reconnect();
void procesarSensores();
void sendData();
void getUbication();
void getHeartSpo2Temp();
void getMqttCredentials();

void readTemperature(){
  float temperaturaObjeto = mlx.readObjectTempC();
  if(temperaturaObjeto < 10){
    return;
  }
  if(tempCounter > 0){
  averageTemp =((averageTemp + temperaturaObjeto)/2 );
  tempCounter--;
  }
  if(tempCounter == 0){
    tempCounter=200;
    float mapTemp = 0.109*averageTemp + 33.07;
    send_data_temperature = mapTemp;
  }
}

void onBeatDetected(){ 
  Serial.print("ON BEAT");
  Serial.print("\t");
  Serial.print("HEART Get: ");
  Serial.print(getHeart);
  Serial.print("\t");
  Serial.print("Heart Send: ");
  Serial.println(send_data_max30100_heart);
  
    if(getHeart < 50 || getSpo2 < 50 ){
        return;
      }

      if(counterHS > 0){
        averageHeart = ((averageHeart + getHeart)/2);
        averageSpo2 = ((averageSpo2 + getSpo2)/2);
        counterHS = counterHS -1;
          send_data_max30100_heart=averageHeart;
          send_data_max30100_spo2=averageSpo2;
      }

      if(counterHS == 0){
      counterHS = 3;
      }
}

//
void setup(){

  //Set Serial monitor baud rate
  Serial.begin(115200);
  Serial.println("esp32 serial initialize");
  delay(10);
  
  //Set GPS module baud rate
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("neogps serial initialize");
  delay(10);

  //Funcion para conectar a la red GSM 
  ConectGPRS();

  // Wire.begin();
  Wire.begin(21, 22, (uint32_t) 400000);

  //Iniciar MAX30100
  Serial.println(pox.begin() ? F("MAX30100 iniciado correctamente") : F("Error al iniciar MAX30100"));
  delay(500);
  pox.setIRLedCurrent(MAX30100_LED_CURR_37MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Iniciar MLX90614 (Temperatura)
  Serial.println(mlx.begin() ? F("MLX90614 iniciado correctamente") : F("Error al iniciar MLX90614"));

  //Parametros para ejecuta el segundo núcle de la ESP
  // xTaskCreatePinnedToCore(loop2,"Task_1",2000,NULL,1,&Task1,0);

}


void loop(){

  verifyGPRS();
  getUbication();
  getHeartSpo2Temp();

  getMqttCredentials();
  delay(5000);
  procesarSensores();
  sendData();
  
}

//obtener contraseña de mqtt y variables de la plataforma 

//obtener los datos de los sensores 

//procesar los sensores 
void procesarSensores() {


//Hacer un archivo Json de 
//Latitud
char char_arrayLat[20];
snprintf(char_arrayLat,sizeof(char_arrayLat),"%0.6f",send_latitud); 
// mqttDataDoc["latitud"] = char_arrayLat;
float sendLat = atof(char_arrayLat);
LatLon["lat"] = sendLat;

//longitud 
char char_arrayLong[20];
snprintf(char_arrayLong,sizeof(char_arrayLong),"%0.6f",send_longitud);
float sendLon = atof(char_arrayLong);
LatLon["lng"] =  sendLon;

serializeJsonPretty(LatLon, Serial);
delay(5000);

mqttDataDoc["variables"][0]["last"]["value"] = LatLon;


mqttDataDoc["longitud"] = char_arrayLong;

//Frecuancia Cardiaca 
char char_arrayHeart[20];
snprintf(char_arrayHeart,sizeof(char_arrayHeart),"%0.0f",send_data_max30100_heart);
mqttDataDoc["Heart"] = char_arrayHeart;

//Saturacion de oxigeno 
char char_arraySpo2[20];
snprintf(char_arraySpo2,sizeof(char_arraySpo2),"%0.0f",send_data_max30100_spo2);
mqttDataDoc["SpO2"] = char_arraySpo2;

//Temperatura 
char char_arrayTemp[20];
snprintf(char_arrayTemp,sizeof(char_arrayTemp),"%0.2f",send_data_temperature);
mqttDataDoc["Temp"] = char_arrayTemp;

}


void sendData(){

  long now = millis();

  if(now-lastsend >= 0){
    lastsend = now + 5000;
    String topic = "a/b/temp";

    String toSend = "";
    serializeJson(mqttDataDoc,toSend);
    client.publish(topic.c_str(),toSend.c_str());

  }

}


void getHeartSpo2Temp(){

   pox.update();
   if (millis() - tsLastReport > REPORTING_PERIOD_MS){
    mx.resetFifo();
   }

  getHeart = pox.getHeartRate();
  getSpo2 = pox.getSpO2();

   if (millis() - tsLastReport > 600){
  readTemperature();
  }

}


void getUbication(){

    while (neogps.available() > 0){
    
    if(gps.encode(neogps.read())){
    if (gps.location.isValid()){
      send_latitud = gps.location.lat();
      send_longitud = gps.location.lng();
    } else {
      Serial.println("No GPS");
    }
    }
  } 


  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println("GPS  NO  DETECTADO");
    delay(5000);
    // neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  }
}

/// conectar a la red gsm 
void ConectGPRS(){

  sim800.begin(9600, SERIAL_8N1, rxPin, txPin,false);
  Serial.println("SIM800L serial initialize");
  delay(500); 

  Serial.println("++++++++++++ MODEM INFO ++++++");
  delay(100);
  Serial.println(modem.getModemInfo());
 
  Serial.println("Initializing modem...");
  if(!modem.restart()){
    Serial.println("GSM fail");
    delay(2000);
    ESP.restart();
    return;
  }
Serial.println("Modem Restart OK");

  if(!modem.waitForNetwork()){
    Serial.println("Error al conectar a la RED");
    delay(2000);
    ESP.restart();
    return;
  }
Serial.println("Red ok");

  if(!modem.gprsConnect(apn, gprsUser, gprsPass)){
    Serial.println("GPRS no conectado");
    delay(2000);
    ESP.restart();
    return;
  }
Serial.println("GPRS conectado");

}

bool reconnect(){

  // if(!getMqttCredentials()){
  //   Serial.println("");
  //   Serial.println("Error en obtener credenciales para MQTT");
  //   delay(5000);
  //   ESP.restart();
  // }

  client.setServer(mqttServer, mqttPort);
  Serial.println("Intentando Conectar al Broker MQTT (EMQX)");

  // const char* userName = mqttDataDoc["username"];
  // const char* password = mqttDataDoc["password"];


  // JsonObject usuario = dataServer["usuario"];
  // String uid = usuario["uid"];
  // String topic = uid + "/+/sdata";
  // String dId = "ESP32_" + uid;



  
  String topic = "a/b/#";

  if(client.connect(dId.c_str(), userName.c_str(), password.c_str())){
   Serial.println("");
   Serial.println("CONECTADO AL BROKER MQTT");
  if(client.subscribe(topic.c_str())){
    Serial.println("Subscrito: " + topic);
  }
  return true;

  } else {
    Serial.println("No se logro conectar al Broker MQTT");
    return false;
  }
  return true;
}

//verificar si esta conectado a GPRS - datos 
void verifyGPRS(){

  if(!modem.isGprsConnected()){
    Serial.println("Desconectado");
    delay(1000);
    Serial.println("Reconectando...");
     
  if(!modem.waitForNetwork(100000,true)) {
      Serial.println("GPRS Failed");
    } 
  else {
    if(!modem.gprsConnect(apn,gprsUser,gprsPass)) {
        Serial.println("Error al conectar GPRS");
        delay(5000);
        ESP.restart();
      }
    else {
      Serial.println("GPRS Reconectado => OK");
      }
    }
  }

  //*Verificar conexión a MQTT
  if(!client.connected()){
  long now = millis();
  if (now - lastReconnect > 5000){
    lastReconnect = millis();
    if(reconnect()){
      lastReconnect = 0;
    }
  }

} else { client.loop(); }

} 


void getMqttCredentials(){

mqttDataDoc["username"] = "jQflvB3M5G";
mqttDataDoc["password"] = "1kr3KqrPIM";
mqttDataDoc["topic"] = "64ae2a8fe070f194c3b187b6/2023/";

JsonArray variables = mqttDataDoc.createNestedArray("variables");

JsonObject variables_0 = variables.createNestedObject();
variables_0["variable"] = "5KnbucfCua";
variables_0["variableFullName"] = "MAPA";
variables_0["variableType"] = "input";
variables_0["variableSendFreq"] = "1";

JsonObject variables_1 = variables.createNestedObject();
variables_1["variable"] = "2PAybTzigS";
variables_1["variableFullName"] = "Frecuencia Cardiaca";
variables_1["variableType"] = "input";
variables_1["variableSendFreq"] = "5";

JsonObject variables_2 = variables.createNestedObject();
variables_2["variable"] = "9cTiqcUyG5";
variables_2["variableFullName"] = "SpO2";
variables_2["variableType"] = "input";
variables_2["variableSendFreq"] = "5";

JsonObject variables_3 = variables.createNestedObject();
variables_3["variable"] = "Jn3zQU7iCC";
variables_3["variableFullName"] = "Temperatura";
variables_3["variableType"] = "input";
variables_3["variableSendFreq"] = "5";

JsonObject variables_4 = variables.createNestedObject();
variables_4["variable"] = "SWqxbvyW1E";
variables_4["variableFullName"] = "Video 01";
variables_4["variableType"] = "esp_cam";
variables_4["variableSendFreq"] = "1";


}


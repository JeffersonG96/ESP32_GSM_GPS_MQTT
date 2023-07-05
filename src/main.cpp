#include <Arduino.h>

//Librerias 
# include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>


#define SerialAT Serial1

#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
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

//VARIABLES SERVIDOR
//TODO: variables http  
const char* mqttServer = "20.171.71.95";
int mqttPort = 1883;

//Variables globales
long lastReconnect = 0;


//*FUNCIONES 
void ConectGPRS();
void verifyGPRS();
bool reconnect();



void setup(){

  //Set Serial monitor baud rate
  Serial.begin(115200);
  Serial.println("esp32 serial initialize");
  delay(10);
  
  //Set GPS module baud rate
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("neogps serial initialize");
  delay(10);

  //Set GSM module baud rate
   
  // Restart takes quite some time
  // To skip it, call init() instead of restart()

  ConectGPRS();

}


void loop(){

  verifyGPRS();
  

}

/// conectar a la red gsm 
void ConectGPRS(){

  sim800.begin(9600, SERIAL_8N1, rxPin, txPin,false);
  Serial.println("SIM800L serial initialize");
  delay(2000); 

  Serial.println("++++++++++++ MODEM INFO ++++++");
  delay(100);
  Serial.println(modem.getModemInfo());
  // String modemInfo = modem.getModemInfo();
  // sim800.println(modemInfo);

  Serial.println("Initializing modem...");
  if(!modem.restart()){
    Serial.println("GSM fail");
    delay(2000);
    ESP.restart();
    return;
  }
Serial.println("Modem Restart OK");

  if(!modem.waitForNetwork(120000,true)){
    Serial.println("Error al conectar a la red");
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

  String dId = "ESP32GSM";
  String userName = "root";
  String password = "root";
  String topic = "a/b";

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

  // Serial.print("GPRS: ");
 
  if(modem.isGprsConnected())
    Serial.println("Connectado a GPRS");
  else {
    Serial.println("Desconectado");
    Serial.println("Reconectando...");
     
  if(!modem.waitForNetwork(100000,true)) {
      Serial.println("GPRS Failed");
    } 
  else {
    if(!modem.gprsConnect(apn,gprsUser,gprsPass)) {
        Serial.println("Error al conectar GPRS");
        delay(5000);
      }
    else {
      Serial.println("GPRS OK");
      }
    }
  }


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
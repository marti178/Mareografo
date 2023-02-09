/**************************************************************
* Proyect: Mareografo V1
* Author: Cronon Tecnologia
* Date: 9/02/2023
* Proyect Manager: Fernando Agostino
* Proyect Leader: Martin Marotta
 **************************************************************/

// Select your modem:
 #define TINY_GSM_MODEM_SIM7080 true
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialMaxB Serial1

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro

#define SerialAT Serial2




// Define the serial console for debug prints, if needed


// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Your GPRS credentials, if any
const char apn[]      = "igprs.claro.com.ar";
const char gprsUser[] = "clarogrps";
const char gprsPass[] = "clarogprs999";

// MQTT details
const char* broker = "mqtt.cronon.com.ar";

const char* topicDistancia      = "Aquaman1/Distancia";
const char* topicInit      = "Aquaman1/init";
const char* topicTemperatura = "Aquaman1/Temperatura";
const char* topicPresion = "Aquaman1/Presion";
const char* topicSignal = "Aquaman1/Signal";
const char* topicBattery = "Aquaman1/Battery";
const char* topicAquaman1 = "Aquaman1/Aquaman1";
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

#include <Arduino.h>
#include<SFE_BMP180.h>
#define DATAIN 32
#define RANGING 33

uint32_t lastReconnectAttempt = 0;
SFE_BMP180 pressure;
double temp;
double pres;

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicAquaman1) {
    
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  boolean status = mqtt.connect(broker, "aquaman1", "P0s31d0n1");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }

  SerialMon.println(" success");
  mqtt.publish(topicInit, "Aquaman1 started");
  mqtt.subscribe(topicAquaman1);
  return mqtt.connected();
}

double getPressure() //Retorna la presion en mb
{
  char status;
  double T,P,p0,a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("Error obteniendo informacion de presion\n");
      }
      else Serial.println("Error iniciando mediciones de presion\n");
    }
    else Serial.println("Error obteniendo mediciones de temperatura\n");
  }
  else Serial.println("Error iniciando mediciones de temperatura\n");
  return 1;
}

void SensorRead(char *dist){

    /* Funcion que devuelve ERROR si en el puerto serie no recibio la suficiente cantidad de datos 
    establecida (6) para leer, dado que todavia no llego el paquete completo de los
    datos del Sensor o llego con errores (sin R al comienzo)

    Recibe Char* de 6 lugares.

    */
    
    char * distancia=NULL;
    int data=0;
    int count=0;
    int flag=1;

    while (flag==1 and count<1000)
    {
    
    digitalWrite(RANGING,HIGH);
    delayMicroseconds(200);
    digitalWrite(RANGING,LOW);
    data=SerialMaxB.available();
    if (data==6)
    {
    distancia=(char *)malloc(data*sizeof(char));
    SerialMaxB.read(distancia,data);
    if ((*distancia=='R')){
    *(distancia+(data*sizeof(char)-1))='\0';
    *(distancia)=*(distancia+1);
    *(distancia+1*sizeof(char))='.';
    strcpy(dist,distancia);
    free(distancia);
        flag=0;
        return ;
    }
    else{
        distancia=(char *)malloc(data*sizeof(char));
        SerialMaxB.read(distancia,data);
        free(distancia);
        flag=1;
        count++;
    }
    }else{
        flag=1;
        count++; //
    }
    }
    strcpy(dist,"ERROR");
    
    } 

void setup() {
  //Seteo de modo de pines
  pinMode(DATAIN,INPUT); //Pin 32 entrada del Maxbotix
  pinMode(RANGING,OUTPUT); //Pin 33 salida para marcar comienzo de lectura del Maxbotix

/*   pinMode(4,OUTPUT); 
  digitalWrite(4,LOW);
  delay(1000);
  digitalWrite(4,HIGH);  */
   char dist[6];
  delay(1000);
  // Set console baud rate
  SerialMon.begin(9600);
  delay(1000);
  SerialAT.begin(9600,SERIAL_8N1,26,27);
  delay(1000);
  SerialMaxB.begin(9600,SERIAL_8N1,32);
  delay(1000);

   if (pressure.begin())
    SerialMon.println("BMP180 Inicio completado");
  else
  {
    // Hubo un problema
    // Consulte los comentarios en la parte superior de este esquema para conocer las conexiones adecuadas.

    SerialMon.println("BMP180 Error de inicio ");
  }

  delay(1000);
  SerialMon.println("Wait...");
  delay(1000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  delay(5000);
  modem.setPreferredMode(3); //Mode NB-Iot
  delay(1000);
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  delay(1000);
  SerialMon.print("Waiting for network...");
  delay(1000);
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }
  
  SensorRead(dist);
  SerialMon.println(dist);
    SensorRead(dist);
  SerialMon.println(dist);
    SensorRead(dist);
  SerialMon.println(dist);
    SensorRead(dist);
  SerialMon.println(dist);
#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  int csq;
  double a;
  
  char dist[6];
  char presion[6];
  char signal[6];
  char temperatura[6];
  int Time=0;
  


  SerialMon.println("Entre al LOOP!");
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }


    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
    }

  }
     //Lectura de la presion
    pres= getPressure();
    SerialMon.print("Presion: ");
    SerialMon.print(pres);
    SerialMon.print(" mb ");  
    //Lectura de temperatura
    Time=pressure.startTemperature();
    delay(Time);
    pressure.getTemperature(temp);
    SerialMon.print(temp);
    SerialMon.print(" ºC "); 
    Serial.print(" distancia: ");
    //Lectura de distancia
    SensorRead(dist);
    SerialMon.print(dist);
    SerialMon.println(" metros "); 


  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }
  csq = modem.getSignalQuality();
  sprintf(presion,"%f",pres);
  sprintf(temperatura,"%f",temp);
  itoa(csq,signal,6);
  mqtt.publish(topicDistancia,dist);
  mqtt.publish(topicPresion,presion);
  mqtt.publish(topicTemperatura,temperatura);
  mqtt.publish(topicSignal,signal);
  delay(6000);

  mqtt.loop();
}
#include <Arduino.h>

#include "Configuration.h"
#include <WiFi.h>
#include "NTPClient.h"
#include <Wire.h>
#include <SPI.h>
//-----OTA Webserver-----//
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
//-----------------------//
#include "WiFiUdp.h"
#ifdef ESP_NOW
  #include <esp_now.h>
#endif
#include <esp_wifi.h>

#ifdef MQTT
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);
#endif

//------------------Sensor Libraries---------------------//
#ifdef DHT_SENSOR
#include "DHT.h"
DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef BMP280_SENSOR
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
#endif

#ifdef BME280_SENSOR
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#endif

#ifdef BME680_SENSOR
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;
#endif

#ifdef BH1750_SENSOR
#include <BH1750.h>
BH1750 lightMeter;
#endif

#ifdef VEML6070_SENSOR
#include "Adafruit_VEML6070.h"
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#endif

#ifdef VEML6075_SENSOR
#include "Adafruit_VEML6075.h"
Adafruit_VEML6075 uv = Adafruit_VEML6075();
#endif

//------Global Variables---------//

float temperature; //Degrees Celcius
float humidity;    //Percentage
int pressure;      //Hectapascals
float lux;         //Lux level
float uva;         //Raw UV A
float uvb;         //Raw UV B
float uvi;         //UV Index
float windspeed;
float winddirection;
float dewpoint;
float heatindex;
float windchill;
float windgust;
float rainfall;
float rainrate;
int altitude;       //Meters

float raw_temp = 0;
int raw_pressure = 0;
int raw_altitude = 0;
int tippings_new = 0;
int tippings_old = 0;


//---------Wind Vane input Pins-------------//

#define NORTH 4
#define NORTH_EAST 19
#define EAST 23
#define SOUTH_EAST 25
#define SOUTH 26
#define SOUTH_WEST 27
#define WEST 32
#define NORTH_WEST 33

long lastWind = 0;
float rainFallcalc = 0;
int calrotations = 0; // total rotations in the specified time
int rotations = 0;
volatile int rotations_total = 0;
int total_tippings = 0;
volatile unsigned long tiptime = millis();
volatile unsigned long tipcount = 0;


const long UTC_Offset = Timezone;
const char* host = "Weather-Station";
long lastMsg = 0;

AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", (UTC_Offset * 3600));

unsigned long clearRainRate = 0;
unsigned long ESP_time_now = 0;
unsigned long debug_time_now = 0;
unsigned long esp_debug_time_now = 0;
unsigned long wind_time_now = 0;
unsigned long tip_time_now = millis();


unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

//------ESP-Now Variables------------//
#ifdef ESP_NOW
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = BASE_Station_address;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
float ESP_temperature; //Degrees Celcius
float ESP_humidity;    //Percentage
int ESP_pressure;      //Hectapascals
float ESP_lux;         //Lux level
float ESP_uva;         //Raw UV A
float ESP_uvb;         //Raw UV B
float ESP_uvi;         //UV Index
float ESP_windspeed;
float ESP_winddirection;
float ESP_dewpoint;
float ESP_heatindex;
float ESP_windchill;
float ESP_windgust;
float ESP_rainfall;
float ESP_rainrate;
int ESP_altitude;       //Meters
  
} struct_message;

// Create a struct_message called myData
struct_message weatherData;

// Insert your SSID
constexpr char WIFI_SSID[] = wifi_ssid;

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG
    if (millis() >= esp_debug_time_now + DEBUG_PERIOD * 1000){
      esp_debug_time_now += (DEBUG_PERIOD * 1000);
      Serial.print("\r\nLast Packet Send Status:\t");
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
  #endif
}
#endif






//---------------------------------------------------------------------------------------------------------//
//--------------------------------------------------Wifi, MQTT and ESP-now---------------------------------//
//---------------------------------------------------------------------------------------------------------//

//-------- Setup and connect to WiFi and MQTT ---------//
void setup_wifi()
{   
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  String hostname = host_name;
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str());                                 //define hostname

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi Connected ");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println();
}

#ifdef MQTT
//-------- MQTT Reconnection --------//
void reconnect()
{

  while (!client.connected())
  {
    Serial.print("Connecting to MQTT broker ...");
    if (client.connect("WeatherStation", mqtt_user, mqtt_password))
    {
      Serial.println("MQTT Connection OK");
    }
    else
    {
      Serial.print("MQTT Error : ");
      Serial.print(client.state());
      Serial.println(" Waiting 5 secondes before retrying");
      delay(5000);
    }
  }
}

void mqttPublish()
{
  long now = millis();
  // Send a message every minute
  if (now - lastMsg > 1000 * PUBLISH_TIME)
  {
    lastMsg = now;

    client.publish(temperature_topic, String(temperature).c_str(), true); // Publish Temperature deg/C
    client.publish(pressure_topic, String(pressure).c_str(), true);       // Publish Pressure hPa
    client.publish(humidity_topic, String(humidity).c_str(), true);       // Publish Humidity %
    client.publish(heatindex_topic, String(heatindex).c_str(), true);     // Publish Heat index deg/C
    client.publish(dewpoint_topic, String(dewpoint).c_str(), true);       // Publish Dew Point deg/C
    client.publish(lux_topic, String(lux).c_str(), true);                 // Publish lux
    client.publish(windchill_topic, String(windchill).c_str(), true);     // Publish Wind Chill deg/C
    client.publish(altitude_topic, String(altitude).c_str(), true);       // Publish Pressure Altitude
    client.publish(uvi_topic, String(uvi).c_str(), true);                 // Publish UV Index
    client.publish(rainrate_topic, String(rainrate).c_str(), true);       // Publish Rain Fall mm/hr
  }
}

void windPublish()
{
  
  long now = millis();
  // Send a message every minute
  if (now - lastWind > 1000 * WIND_TIME)
  {
    lastWind = now;
  }
  client.publish(windspeed_topic, String(windspeed).c_str(), true);         // Publish windspeed km/hr
  client.publish(winddirection_topic, String(winddirection).c_str(), true); // Publish wind direction deg
}
#endif



//-----------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------Interupt input function---------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------//

#ifdef ESP_32
void IRAM_ATTR isr_rotation()
{
  volatile unsigned long ContactBounceTime;
  
  if ((millis() - ContactBounceTime) > 15 ) // debounce the switch contact.
  {
  rotations++;
  ContactBounceTime = millis();
  }
}

//-----------------------------------------------------------------------------------------------------------------------//

void IRAM_ATTR isr_rain()
{
  volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

  if ((millis() - ContactBounceTime) > 15 ) // debounce the switch contact.
  { 
  total_tippings++;
  ContactBounceTime = millis();
  }
  
  tipcount = millis() - tiptime;
  tiptime = millis();
  
  rainrate = 0.39*3600000 / tipcount;
  client.publish(rainrate_topic, String(rainrate).c_str(), true);                // Publish Rain Fall mm/hr
  
}
#endif

//------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------Manual Calculations-------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------------//

float temperaturecalc(){
  float calctemp = 0;

  #ifdef CELSIUS
    calctemp = raw_temp;
  #endif

  #ifdef FARENHEIT
    calctemp = (raw_temp *1.8) + 32;
  #endif

  return calctemp;
}

int presscalc(){
  float alt_feet =0;
  float press_adj = 0;
  int pressure_alt = 0;

  alt_feet = LOCATION_ALTITUDE * 3.28084;
  press_adj = alt_feet / 30;

  pressure_alt = raw_pressure + press_adj;

  return pressure_alt; 

}

//----------------------------Calculate WindSpeed---------------------------//
float calcWindspeed()
{
  int period = 5000;
  float windcalc;       // calculated wind speed
  
  
  if (millis() >= wind_time_now + period)
  {
    wind_time_now += period;
    calrotations = rotations;
    rotations = 0;
  }

  if (calrotations == 0 || calrotations <= 8)
  {
    windcalc = calrotations * 1.98;
  }
  else if (calrotations >= 9 || calrotations <= 17)
  {
    windcalc = calrotations * 1.92;
  }
  else if (calrotations >= 18 || calrotations <= 30)
  {
    windcalc = calrotations * 1.3;
  }
  else if (calrotations >= 31 || calrotations <= 50)
  {
    windcalc = calrotations * 1.28;
  }
  else if (calrotations >= 51 || calrotations <= 60)
  {
    windcalc = calrotations * 1.25;
  }
  else if (calrotations >= 61)
  {
    windcalc = calrotations * 1.21;
  }

  return windcalc;
}

//---------------------------Wind Direction------------------------------//
float returnWindDirection(){

  float dir = 0;


  int N = 0;
  int NE = 0;
  int E = 0;
  int SE = 0;
  int S = 0;
  int SW = 0;
  int W = 0;
  int NW = 0;

  N = digitalRead(NORTH);
  NE = digitalRead(NORTH_EAST);
  E = digitalRead(EAST);
  SE = digitalRead(SOUTH_EAST);
  S = digitalRead(SOUTH);
  SW = digitalRead(SOUTH_WEST);
  W = digitalRead(WEST);
  NW = digitalRead(NORTH_WEST);
  
if (N == 1 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 1 && NW == 0)
{
  dir = 360;
}
else if (N == 1 && NE == 1 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 22.5;
}
else if (N == 0 && NE == 1 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 45;
}
else if (N == 0 && NE == 1 && E == 1 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 67.5;
}
else if (N == 0 && NE == 0 && E == 1 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 90;
}
else if (N == 0 && NE == 0 && E == 1 && SE == 1 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 112.5;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 1 && S == 0 && SW == 0 && W == 0 && NW == 0)
{
  dir = 135;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 1 && S == 1 && SW == 0 && W == 0 && NW == 0)
{
  dir = 158.5;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 1 && SW == 0 && W == 0 && NW == 0)
{
  dir = 180;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 1 && SW == 1 && W == 0 && NW == 0)
{
  dir = 202.5;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 1 && W == 0 && NW == 0)
{
  dir = 225;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 1 && W == 1 && NW == 0)
{
  dir = 247.5;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 1 && NW == 0)
{
  dir = 270;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 1 && NW == 1)
{
  dir = 292.5;
}
else if (N == 0 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 1)
{
  dir = 315;
}
else if (N == 1 && NE == 0 && E == 0 && SE == 0 && S == 0 && SW == 0 && W == 0 && NW == 1)
{
  dir = 337.5;
}

return dir;
}

//------------------------Dewpoint-----------------------------------//
float returnDewPoint(){
  float Ts;
  float T = temperature;
  float RH = humidity;
  
  Ts = T -(( 100 - RH ) / 5 );

  return Ts;
}

//-----------------Wind Chill--------------------------------//
float returnWindChill()
{
  float chill;
  float V = windspeed;
  float T = temperature;

  if (temperature < 15 && windspeed > 5)
    chill = 13.12 + 0.6215 * T - 11.37 * (pow(V, 0.16)) + 0.3965 * T * (pow(V, 0.16));    //Metric formula wor wind chill
  else
    chill = temperature;

  return chill;
}

//--------------------Heat Index----------------------------//
float getHeatIndexcalc()
{
  float heatindexC;
  float hi;
  float tempF;
  float tempC = temperature;

  tempF = (tempC * 1.8) + 32; // Confert Celcius to Farinheit

  hi = 0.5 * (tempF + 61.0 + ((tempF - 68.0) * 1.2) + (humidity * 0.094));              //Basic heat index formula for temp below 79F or 26C

  if (hi > 79)
  {                                                                                     //Advance Formula for temp above 79F or 26C
    hi = -42.379 + 2.04901523 * tempF + 10.14333127 * humidity +                        //HI= c1+c2T+c3R+c4TR+c5T2+c6R2+c7T2R+c8TR2+c9T2R2
         -0.22475541 * tempF * humidity +
         -0.00683783 * pow(tempF, 2) +
         -0.05481717 * pow(humidity, 2) +
         0.00122874 * pow(tempF, 2) * humidity +
         0.00085282 * tempF * pow(humidity, 2) +
         -0.00000199 * pow(tempF, 2) * pow(humidity, 2);

    if ((humidity < 13) && (tempF >= 80.0) && (tempF <= 112.0))                         //Reduction in Heatindex based on Humidity and Temperature
      hi -= ((13.0 - humidity) * 0.25) * sqrt((17.0 - abs(tempF - 95.0)) * 0.05882);

    else if ((humidity > 85.0) && (tempF >= 80.0) && (tempF <= 87.0))                   //Addition in Heat Index based on Humidity and Temperature
      hi += ((humidity - 85.0) * 0.1) * ((87.0 - tempF) * 0.2);
  }
  heatindexC = (hi - 32) / 1.8;                                                         //Confert Heat Index from Farenheit to Celcius

  return heatindexC;
}


//------ Fuction to reset Rainrate after a giver interval------//
void resetRainRate()
{
  int clearRainRate = 0;

  if (millis() >= clearRainRate + (1000 * 60 * 20))  //20min  // interval set in config section
  { 
    clearRainRate = millis();
    rainrate = 0;
  }
}

//------ Function to work out Rainfall in a set time period -------//
float returnRainFall(){
  int ntp_hrs = timeClient.getHours();
  int ntp_min = timeClient.getMinutes();

if (ntp_hrs == 0 && ntp_min <= 5 )
  {
    #ifdef MQTT
    client.publish(rainfall_topic, String(rainfall).c_str(), true); // Publish total Rain Fall in mm from set clear time
    #endif
    total_tippings = 0;
  }
  else
  {
    rainFallcalc = total_tippings * 0.38;          //Tip bucket volume x numbet of tips gives rainfall
  }

return rainFallcalc;
}

void esp_now_send_values(){

  if (millis() >= ESP_time_now + ESP_PERIOD * 1000)
  {
    ESP_time_now += (ESP_PERIOD * 1000);
    // Set values to send
  weatherData.ESP_temperature = temperature;
  weatherData.ESP_humidity = humidity;
  weatherData.ESP_pressure = pressure;
  weatherData.ESP_lux = lux;
  weatherData.ESP_uva = uva; 
  weatherData.ESP_uvb = uvb; 
  weatherData.ESP_uvi = uvi; 
  weatherData.ESP_windspeed = windspeed; 
  weatherData.ESP_winddirection = winddirection; 
  weatherData.ESP_dewpoint = dewpoint; 
  weatherData.ESP_heatindex = heatindex; 
  weatherData.ESP_windchill = windchill; 
  weatherData.ESP_windgust = windgust; 
  weatherData.ESP_rainfall = rainfall;
  weatherData.ESP_rainrate = rainrate;
  weatherData.ESP_altitude = altitude;   

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &weatherData, sizeof(weatherData));
  #ifdef DEBUG
    if (millis() >= debug_time_now + DEBUG_PERIOD * 1000)
    {
      if (result == ESP_OK) {
      Serial.println("Sent with success");
      }
    else {
      Serial.println("Error sending the data");
      }
    }  
  #endif
  } 
}

void debug(){

  if (millis() >= debug_time_now + DEBUG_PERIOD * 1000)
  {
    debug_time_now += (DEBUG_PERIOD * 1000);
    Serial.print("Rotations = ");
    Serial.print(rotations);
    Serial.print("Total Rotations = ");
    Serial.print(rotations_total);
    Serial.print("\tSpeed = ");
    Serial.print(windspeed);
    Serial.print("\tDirection = ");
    Serial.print(winddirection);
    Serial.print("\tTemperature = ");
    Serial.print(temperature);
    Serial.print("\tPressure = ");
    Serial.print(pressure);
    Serial.print("\tHumidity = ");
    Serial.print(humidity);
    Serial.print("\tAltitude = ");
    Serial.print(altitude);
    Serial.print("\tHeat Index = ");
    Serial.print(heatindex);
    Serial.print("\tWind Chill = ");
    Serial.print(windchill);
    Serial.print("\tUV Index = ");
    Serial.print(uvi);
    Serial.print("\tLux = ");
    Serial.print(lux);    
    Serial.print("\tDewpoint = ");
    Serial.print(dewpoint);
    Serial.print("\tCurrent HRS = ");
    Serial.print(timeClient.getHours());
    Serial.print("\tCurrent Min = ");
    Serial.print(timeClient.getMinutes());
    Serial.print("\tRain Rate = ");
    Serial.print(rainrate);
    Serial.print("\tTotal Tippings = ");
    Serial.print(total_tippings);
    Serial.println();
  }
  
}


//-------------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------Setup Function ---------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------//
void setup(){
  Serial.begin(115200);



//---------------------ESP NoW Setup----------------//
  #ifdef ESP_NOW
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
#endif

//----------------OTA Setup-----------------------//

server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am GEcko Weather Station. Add '/update' to the URL");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  Wire.begin();

  setup_wifi();                           //Start Wifi

#ifdef MQTT                             //Connect to Wifi network

  client.setServer(mqtt_server, 1883);    // Configure MQTT connexion

#endif

  timeClient.begin();                     //Start NTP server


//-----------------------------Interupt Setups------------------------------//
#ifdef ESP_32
  attachInterrupt(WINDSPD, isr_rotation, RISING); // Wind speed Interupt setup
  attachInterrupt(RAINPIN, isr_rain, RISING);     // Rain Interupt setup
#endif

//------------------------------------------------------------------//
//------------------------Sensors Setup-----------------------------//
//------------------------------------------------------------------//
//DHT setup
#ifdef DHT_SENSOR
  Serial.println(F("DHTxx test!"));
  dht.begin();
#endif
  //BMP280 setup
#ifdef BMP280_SENSOR
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
#endif
  //BME280 setup
#ifdef BME280_SENSOR
  Serial.println(F("BME280 test"));

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
#endif
  //BME680 setup
#ifdef BME680_SENSOR
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }
  // Set up oversampling and filter initialization on BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
#endif
//BH1750 setup
#ifdef BH1750_SENSOR
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));
#endif
//VEML6070 setup
#ifdef VEML6070_SENSOR
  uv.begin(VEML6070_4_T); // pass in the integration time constant
#endif

//VEML6075 setup
#ifdef VEML6075_SENSOR
  Serial.println("VEML6075 Full Test");
  if (!uv.begin())
  {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
  }
  Serial.println("Found VEML6075 sensor");

  // Set the integration constant
  uv.setIntegrationTime(VEML6075_100MS);
  // Get the integration constant and print it!
  Serial.print("Integration time set to ");
  switch (uv.getIntegrationTime())
  {
  case VEML6075_50MS:
    Serial.print("50");
    break;
  case VEML6075_100MS:
    Serial.print("100");
    break;
  case VEML6075_200MS:
    Serial.print("200");
    break;
  case VEML6075_400MS:
    Serial.print("400");
    break;
  case VEML6075_800MS:
    Serial.print("800");
    break;
  }
  Serial.println("ms");
  // Set the high dynamic mode
  uv.setHighDynamic(true);
  // Get the mode
  if (uv.getHighDynamic())
  {
    Serial.println("High dynamic reading mode");
  }
  else
  {
    Serial.println("Normal dynamic reading mode");
  }
  // Set the mode
  uv.setForcedMode(false);
  // Get the mode
  if (uv.getForcedMode())
  {
    Serial.println("Forced reading mode");
  }
  else
  {
    Serial.println("Continuous reading mode");
  }
  // Set the calibration coefficients
  uv.setCoefficients(2.22, 1.33,          // UVA_A and UVA_B coefficients
                     2.95, 1.74,          // UVB_C and UVB_D coefficients
                     0.001461, 0.002591); // UVA and UVB responses
#endif

//Wind Vane input setup
pinMode(NORTH,  INPUT);
pinMode(NORTH_EAST,  INPUT);
pinMode(EAST,  INPUT);
pinMode(SOUTH_EAST,  INPUT);
pinMode(SOUTH,  INPUT);
pinMode(SOUTH_WEST,  INPUT);
pinMode(WEST,  INPUT);
pinMode(NORTH_WEST,  INPUT);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------Loop Function--------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//


void loop() {

#ifdef MQTT
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  mqttPublish();
  windPublish();
#endif

  timeClient.update();
//------------------------------------------------------------------//
//------------------------Sensors Readings--------------------------//
//------------------------------------------------------------------//
  //--------Temperature Reading----------//
#if (TEMP_SENSE == 1)
  raw_temp = dht.readTemperature();
#elif (TEMP_SENSE == 2)
  raw_temp = bmp.readTemperature();
#elif (TEMP_SENSE == 3)
  raw_temp = bme.readTemperature();
#elif (TEMP_SENSE == 4)
  raw_temp = (bme.temperature);
#else
#endif
  //--------Humidity Reading----------//
#if (HUMIDITY_SENSE == 1)
  humidity = dht.readHumidity();
#elif (HUMIDITY_SENSE == 2)
  humidity = 0;
#elif (HUMIDITY_SENSE == 3)
  humidity = bme.readHumidity();
#elif (HUMIDITY_SENSE == 4)
  humidity = (bme.humidity);
#else
#endif
  //--------Pressure Reading-----------//
#if (PRESS_SENSE == 1)
  raw_pressure = 0;
#elif (PRESS_SENSE == 2)
  raw_pressure = bmp.readPressure();
#elif (PRESS_SENSE == 3)
  raw_pressure = (bme.readPressure() / 100);
#elif (PRESS_SENSE == 4)
  raw_pressure = (bme.pressure / 100.0);
#else
#endif
  //--------Altitude Reading-----------//
#if (PRESS_SENSE == 1)
  altitude = 0;
#elif (PRESS_SENSE == 2)
  altitude = bmp.readAltitude(1013.25);
#elif (PRESS_SENSE == 3)
  altitude = bme.readAltitude(1013.25);
#elif (PRESS_SENSE == 4)
  altitude = bme.readAltitude(1013.25);
#else
#endif
//--------Light Reading-------------//
#if (LIGHT_SENSE == 1)
  lux = lightMeter.readLightLevel();
#else
#endif

  //--------UV Reading---------------//
#if (UV_SENSE == 1)
  uva = uv.readUV();
#elif (UV_SENSE == 2)
  uva = uv.readUVA();
  uvb = uv.readUVB();
  uvi = uv.readUVI();
#else
#endif


//-------Temperature Reading--------//
//temperature = temperaturecalc();
temperature = raw_temp;
pressure = presscalc();



heatindex = getHeatIndexcalc();
windchill = returnWindChill();
dewpoint = returnDewPoint();


//------------Wind & Rain-------------//

windspeed = calcWindspeed();
winddirection = returnWindDirection();
rainfall = returnRainFall();

  resetRainRate();
#ifdef DEBUG
  debug();
#endif

#ifdef ESP_NOW
esp_now_send_values();
#endif

}
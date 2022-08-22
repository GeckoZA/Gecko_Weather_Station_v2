#include <Arduino.h>
//---------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------Weeather Station Config--------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//

//-----------------------------------------Define Processor Type-------------------------------------------------------//


#define ESP_32                 

//------------------------------------------Define Units----------------------------------------------------------//

//#define BASE_Station_address {0x18, 0xFE, 0x34, 0xD6, 0x7E, 0x15}
#define BASE_Station_address {0xC4, 0xDD, 0x57, 0x92, 0xA5, 0xE8}

//---------Temperature--------------//
#define CELSIUS
//#define FARENHEIT

//---------Pressure--------------//
//#define mbar
//#define inHg

//---------Altitude--------------//
//#define meters
//#define feet

//---------Wind Speed--------------//
//#define km/hr
//#define mph
//#define kts
//#define m/s

//---------Rain--------------//

//#define mm
//#define inch

//---------------------------------------------------------------------------------------------------------------------//



//-------------------------------------------Uncomment Sensors Used----------------------------------------------------//

//#define DHT_SENSOR         //DHT     Temperature           Humidity
//#define BMP280_SENSOR      //BMP280  Temperature Pressure
#define BME280_SENSOR       //BME280  Temperature Pressure  Humidity
//#define BME680_SENSOR       //BME680  Temperature Pressure  Humidity  VOX
#define BH1750_SENSOR       //BH1750  Lux
//#define VEML6070_SENSOR     //VEML6070  UV-A
#define VEML6075_SENSOR     //VEML6075  UV-A UV-B UV-Index

//------------------------------------------Individual Sensor for specific reading-------------------------------------//
//----------------------------------Select the number of the sensor used for the reading-------------------------------//

// Temperature sensor
//DHT           = 1
//BMP280        = 2
//BME280        = 3
//BME680        = 4
#define TEMP_SENSE 3

//Pressure Sensor
//DHT           = 1
//BMP280        = 2
//BME280        = 3
//BME680        = 4
#define PRESS_SENSE 3
#define SEALEVELPRESSURE_HPA (1013.25)
#define LOCATION_ALTITUDE 1537

//Humidity Sensor
//DHT           = 1
//BMP280        = 2
//BME280        = 3
//BME680        = 4
#define HUMIDITY_SENSE 3

//VOX Sensor
// BME680       = 1
//#define VOX_SENSE 0


//Light Sensor
//BH1750        = 1
#define LIGHT_SENSE 1

//UV sensor
//VEML6070      = 1
//VEML6075      = 2
#define UV_SENSE 2

//------------------------------------Analog sensors Pin (GPIO numbers) and Type Configuration----------------------------------------//

//#define DHTTYPE DHT22                           // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21                         // DHT 21 (AM2301)
//#define DHTPIN 14

//---------------------------------------------Wind and Rain Sensor (GPIO Numbers)----------------------------------------------------//
#define RAINPIN 17                               // Rain sensor pin
#define WINDSPD 16                              // Wind Speed Sensor
//#define WINDDIR 35                              // Wind Direction

//---------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------WiFi, MQTT and Time Zone Config--------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//


#define host_name "Weather Station v2"
#define wifi_ssid "Fornax"                      //Wifi Name
#define wifi_password "Premier11"               //WiFi Password
#define mqtt_server "192.168.1.132"             //MQTT Server address
#define mqtt_user "mqttpegasus"                 //MQTT username
#define mqtt_password "pegasus"                 //MQTT Password

#define Timezone 2

//---------------------------------------------- MQTT topic config --------------------------------------------------- //

#define windspeed_topic "WeatherStation/windspeed"          //Topic windspeed Km/hr
#define winddirection_topic "WeatherStation/winddirection"  //Topic Winddirection deg
#define temperature_topic "WeatherStation/temperature"      //Topic Temperature deg C
#define pressure_topic "WeatherStation/pressure"            //Topic Pressure hPa
#define humidity_topic "WeatherStation/humidity"            //Topic Humidity %
#define heatindex_topic "WeatherStation/heatindex"          //Topic Heat Index deg C
#define dewpoint_topic "WeatherStation/dewpoint"            //Topic Dew Point deg C
#define lux_topic "WeatherStation/lux"                      //Topic Lux
#define windchill_topic "WeatherStation/windchill"          //Topic Wind Chill deg C
#define rainfall_topic "WeatherStation/rainfall"            //Topic Rain Fall in mm pertime period
#define uvi_topic "WeatherStation/uv"                       //Topic Rain Fall in Raw UVA
#define vox_topic "WeatherStation/vox"                      //Topic Rain Fall in resistance
#define altitude_topic "WeatherStation/altitude"            //Topic Rain Fall in pressure altitude in meters
#define rainrate_topic "WeatherStation/rainrate"            //Topic Rain Fall in mm/hr

#define PUBLISH_TIME 30                                     // Time in seconds to publish mqtt messages
#define WIND_TIME 10                                        // Time in seconds to publish mqtt messages
#define INTERVAL 20                                          //Rain Rate reset time in min
#define cleartime 24                                        //Total Rain fall clear time


//------------------------------------------------ESP-Now Screen MAC Address-----------------------------------------//

//#define BASE_Station_address {0x18, 0xFE, 0x34, 0xD6, 0x7E, 0x15}
#define BASE_Station_address {0xC4, 0xDD, 0x57, 0x92, 0xA5, 0xE8}

//-----------------------------------------Uncomment for Outputs----------------------------------------------------- //

#define MQTT                                    //MQTT Output based on settings
#define ESP_NOW                                 //ESP-NOW Protocol
//#define DEBUG                                   //Serial output

#define DEBUG_PERIOD 5
#define ESP_PERIOD 0.1
//-------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------- End Config --------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

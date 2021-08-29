//For cable connections, follow
//https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test 
#include "LowPower.h"
#include <TheThingsNetwork.h>
#define loraSerial Serial1
#define debugSerial Serial
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h> //For external temperature sensor
#include <DallasTemperature.h>

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_US915
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
// Set your AppEUI and AppKey

const char *appEui = "70B3D57ED003AAD2";
const char *appKey = "A8457C53BDF9B05B9D76FB03BD6D0CE2";


#define SEALEVELPRESSURE_HPA (1013.25)
#define PRESSURE_BASE 40000

  //**** Define if the station has vis and wind sensors
#define VIS_SENSOR false
#define WIND_SENSOR true

#define VIS_SENSOR_INTERVAL 61000 //Warm up time MILI SECONDS required by the VIS sensor 

/******************** STATION STATE **************************
state:
{
  sleepTime:txIntervalSleep,
  inbuildLed:0
}
/***** Sleeping time TXT_INTERVAL is multiplied by 8 seconds enterSleep function*/
//Initiate state
int contInitialAttemps=0;
#define INITIAL_ATTEMPS 10
#define DEFAULT_SLEEP_TEMP 900
bool intervalSleepChanged=false;
int txIntervalSleep= 10; //6*8**1000 ms =48 seconds
int inbuildLed=0;

//For BME280 sensor
Adafruit_BME280 bme; // I2C
bool statusBME=false;
//********** DIGITAL SENSORS PIN *************//
#define EXTERNAL_TEMP_1 7
#define EXTERNAL_TEMP_2 6
#define EXTERNAL_TEMP_3 5

//********** POWER CONTROLLER PIN *************//
int POWER_PIN = 8; //5 v
int POWER_PIN12=9; //12V

//********** EXTERNAL TEMP CONTROLLER *************//
OneWire external_temp_1(EXTERNAL_TEMP_1);
DallasTemperature sensors_1(&external_temp_1);
OneWire external_temp_2(EXTERNAL_TEMP_2);
DallasTemperature sensors_2(&external_temp_2);
OneWire external_temp_3(EXTERNAL_TEMP_3);
DallasTemperature sensors_3(&external_temp_3);


//********** ANALOG SENSORS PIN *************//
const int aLightPin=A0;
const int aLiquidLevelPin=A1;
const int aBattery=A2;
const int aVisPin=A3;
const int aWindPin=A4;

const float a=21.01;
const  float b=-0.1411;

//Download inbuild pin led



void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  //POWER SENSORS
  pinMode(POWER_PIN,OUTPUT);
  pinMode(POWER_PIN12,OUTPUT);
 
  if(WIND_SENSOR || VIS_SENSOR)
  {
    //Warming up VIS sensor
    debugSerial.println("Turning on 12 volts");
    digitalWrite(POWER_PIN12,HIGH);
    if(VIS_SENSOR)
    {
      debugSerial.println("Warming up VIS optical sensor");
      delay(VIS_SENSOR_INTERVAL); //Use this time for avoiding sleeping mode*
    }    
  }
  digitalWrite(POWER_PIN,HIGH);
  delay(2000);
  
  // INIT TTN INTERFACE
  while (!debugSerial && millis() < 10000)
    ;
  debugSerial.println("-- STATUS");
  ttn.wake();
  ttn.showStatus();
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
  
  //****Callback to receive downlink
  //used to update the status of the device. 
  ttn.onMessage(message);

  //INIT BME SENSOR
  // default settings
   statusBME = bme.begin();  
    if (!statusBME) {
        debugSerial.println("Could not find a valid BME280 sensor, check wiring!");
       
    }
    debugSerial.println("-- Default Test --");
    debugSerial.println();
    sensors_1.begin(); 
    sensors_2.begin();
    delay(1000);
    Serial.println("OK!");
}

void loop()
{
  //********** Internal box sensors BME280 **************//
  uint32_t intTemperature=getIntTemperature()*100;
  debugSerial.print("Internal temperature: ");
  debugSerial.println(intTemperature);
  uint32_t intHumidity=getIntHumidity()*100;
  debugSerial.print("Humidity: ");
  debugSerial.println(intHumidity);
  uint32_t intAirPressure=getAirPressure()*100;
  debugSerial.print("Air pressure: ");
  debugSerial.println(intAirPressure); 
  //To be able to send airpresure in 2 bytes
  intAirPressure=intAirPressure-PRESSURE_BASE;
  uint32_t light=getIntLight();
  debugSerial.print("Light: ");
  debugSerial.println(light);  

  //***************** External temperature sensors *********************//  
  uint32_t extTemperature_1=getExtTemperature(sensors_1)*100;
  debugSerial.print("External Temperature 1: ");
  debugSerial.println(extTemperature_1);
  uint32_t extTemperature_2=getExtTemperature(sensors_2)*100;
  debugSerial.print("External Temperature 2: ");
  debugSerial.println(extTemperature_2);
  
  //************** Battery sensor **************//
  uint32_t battery=getBattery()*100;
  debugSerial.print("Battery: ");
  debugSerial.println(battery);
  
  byte payload[16];   
  //Floats
  payload[0]=highByte(intHumidity);
  payload[1]=lowByte(intHumidity);
  payload[2]=highByte(intAirPressure);
  payload[3]=lowByte(intAirPressure);
  payload[4]=highByte(extTemperature_1);
  payload[5]=lowByte(extTemperature_1);
  payload[6]=highByte(extTemperature_2);
  payload[7]=lowByte(extTemperature_2);
  payload[8]=highByte(intTemperature);
  payload[9]=lowByte(intTemperature);
  //Int analog readings
  payload[10]=light; //from 0 to 255
  payload[11]=highByte(battery); //voltage
  payload[12]=lowByte(battery); //
  //Send state: sleepTime
  payload[13]=highByte(txIntervalSleep); //voltage
  payload[14]=lowByte(txIntervalSleep); //
  payload[15]=inbuildLed; 
  
  
  
  ttn.sendBytes(payload,sizeof(payload));
 
  //******** SLEEP THE NODE ***************//
  //Sleepin LORA module
    //ttn.sleep(TX_INTERVAL_SLEEP * 8L * 1000L);
    ttn.sleep(txIntervalSleep * 8L * 1000L);
    Serial.print("****** SLEEPING LORA Module:");
    //Serial.print(TX_INTERVAL_SLEEP*8);
    Serial.print(txIntervalSleep*8);
    Serial.println(" seconds");
  //Sleeping sensors connected to POWER_PIN
    digitalWrite(POWER_PIN12,LOW);
    digitalWrite(POWER_PIN,LOW);   
    delay(1000); //To let print and PINs low correctly before sleeping 
   
   //********** Going to sleep power down ********/
   enterSleepTesting();
   // enterSleep(); //sleep TX_INTERVAL_SLEEP*8 seconds
    
   //******** WAKING UP THE NODE ***********//
   //Wake up VIS sensor VIS_SENSOR_INTERVAL before the whole node
    if(WIND_SENSOR || VIS_SENSOR)
    {
      //Warming up VIS sensor
      digitalWrite(POWER_PIN12,HIGH);
      if(VIS_SENSOR)
      {
        debugSerial.println("Warming up VIS optical sensor");
        delay(VIS_SENSOR_INTERVAL); //Use this time for avoiding sleeping mode*
      }
    }
    
    //Waking up LoRa module
    ttn.wake();
    digitalWrite(POWER_PIN,HIGH);
    delay(2000);
    //************** Start again BME sensor//
    statusBME=bme.begin();
    if (!statusBME) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
     }
    delay(1000);
    Serial.println("***** Just wake up! ******** ");

     //After 10 times of sending the default value changes to 8*900 seconds 
    if((!intervalSleepChanged) && (contInitialAttemps<INITIAL_ATTEMPS))
    {
        contInitialAttemps=contInitialAttemps +1;
        if (contInitialAttemps==INITIAL_ATTEMPS)
        {
            txIntervalSleep= DEFAULT_SLEEP_TEMP; 
          } 
     }
  
}

//Get Internal sensors
float getIntTemperature(){
  if (statusBME){
    return bme.readTemperature(); //*C
   }else{
    return 0.0; //sensor could not be init
    }
}
float getIntHumidity(){
 if (statusBME){
    return bme.readHumidity(); //%
  }
  else{
    return 0.0;
    }
}
float getAirPressure(){
   if (statusBME){
    return (bme.readPressure() / 100.0F); //hPa
  }
  else
  {
    return 0.0;
    }
}
float getAltitude(){
  if(statusBME){
    return (bme.readAltitude(SEALEVELPRESSURE_HPA)); //m
  }
  else{
    return 0.0;
  }
}

//Analog light
int getIntLight(){
  //Measured from the analog pin
  int sensorVal=analogRead(aLightPin);
  int light=map(sensorVal,0,1023,0,255);
  return light;
}
//External sensors
float getExtTemperature(DallasTemperature sensors){
    sensors.requestTemperatures(); 
    return sensors.getTempCByIndex(0); //*C
}

//Analog liquid level
float getAnalogLiquidLevel(){
  //Measured from the analog pin
  float sensorVal=analogRead(aLiquidLevelPin);
  sensorVal=sensorVal/100;
  return sensorVal;
  //return (a+b*pow(sensorVal,3)); 
}

//Get Battery
float getBattery(){
  int sensorVal=analogRead(aBattery);
  float batteryVoltage=sensorVal*(5.0/1023.0);
  return batteryVoltage;
 }
float getVis(){
  int visVal=analogRead(aVisPin);
  float visVal_Voltage=visVal*(5.0/1023.0);
  return visVal_Voltage;
  
  }
float getWindSpeed(){
  float voltageMax=2.0;
  float voltageMin=0.45;
  float windSpeedMin=0;
  float windSpeedMax=32.4;
  int windSensorDelay=500; //mili seconds
  int nWindReadings=20;
  
  float windSpeed=0.0;
  int windSensorValue=0;
  float windSpeedVoltage=0.0;
  float totalWindSpeed=0.0;
  float avrWindSpeed=0.0;
  for(int i=0;i<nWindReadings;i++){
      //0. Init windSpeed to 0
      windSpeed=0.0;
      //1. take value from analog pin
      windSensorValue=analogRead(aWindPin);
      //2convert to speed
      //2.1 Convert to voltage
      windSpeedVoltage =windSensorValue*(5.0/1023);
      //2.2 Convert to speed
      if (windSpeedVoltage > voltageMin){
        windSpeed=((windSpeedVoltage -voltageMin)*windSpeedMax/(voltageMax-voltageMin))*2.232694; 
        
      }
      //Add value to total
      totalWindSpeed=totalWindSpeed+windSpeed;
      delay(windSensorDelay);
    }
    //Calculate average speed
    avrWindSpeed=totalWindSpeed/nWindReadings;
    return avrWindSpeed;
  
}  
 
  /***************************************************
 *  Name:        enterSleep
 *  Returns:     Nothing.
 *  Parameters:  intervals: intervals of 8 s that arduino will sleep.
 *  Description: Enters the arduino into sleep mode.
 ***************************************************/
void enterSleep(){
  //for(int i=0; i<TX_INTERVAL_SLEEP; i++){
  for(int i=0; i<txIntervalSleep; i++){
      LowPower.powerDown(SLEEP_8S,ADC_OFF,BOD_OFF);  
    }
  }
  
void enterSleepTesting(){

    Serial.println("***** entra en sleeping testing! ******** ");
    //long sleepInterval=TX_INTERVAL_SLEEP * 8L * 1000L;
    long sleepInterval=txIntervalSleep * 8L * 1000L;
    Serial.println(sleepInterval);
    delay(sleepInterval);
    Serial.println("***** end of delay! ******** ");
 }

 void message(const uint8_t *payload, size_t size, port_t port)
 {
   if(payload[0])
   {
      Serial.println("New sleep interval defined to: ");
      txIntervalSleep=payload[0];
      Serial.println(txIntervalSleep);
   }  
   if(payload[1]==1)
   {
      digitalWrite(LED_BUILTIN, HIGH);
      inbuildLed=1;  
   }
   if(payload[1]==0)
   {
    digitalWrite(LED_BUILTIN, LOW);
    inbuildLed=0;
   } 
 }

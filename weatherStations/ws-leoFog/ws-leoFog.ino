//LoRa weather station with TTN-uno leonardo 
//with Fog sensor, liquid level etape (analog sensor)
//Temperature sensor: Adafruit SHT-30 water proof
//WS-Df robot SEN0186

#include "LowPower.h"
#include <TheThingsNetwork.h>
#define loraSerial Serial1
#define debugSerial Serial
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Adafruit_SHT31.h>

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED003AAD2";
const char *appKey = "DD4CFEBA51CFC5CC341B395FC21C0F06";

Adafruit_SHT31 sht31 = Adafruit_SHT31();
SoftwareSerial mySerial(11, 12); // RX, TX , this is for DFRobot SEN0186 serial communication

#define SEALEVELPRESSURE_HPA (1013.25)
#define PRESSURE_BASE 40000

//**** Define if the station has vis and wind sensors
#define VIS_SENSOR false
#define WIND_SENSOR false
#define SEND_TO_TTN false
#define VIS_SENSOR_INTERVAL 61000 //Warm up time MILI SECONDS required by the VIS sensor 

/************************************************************************************/
/************************ SLEEP FUNCTION PARAMETERS *********************************/
//When turned on, the node will send data INITIAL_ATTEMPS in the initial value of sleepInterval.
//After the INITIAL_ATTEMPS, the sleepInterval will be changed to DEFAULT_SLEEP_TEMP     
#define INITIAL_ATTEMPS 5
#define DEFAULT_SLEEP_TEMP 1400   //actual sleeping time=DEFAULT_SLEEP_TEMP*8 seconds
int initialAttempsCounter=0;
int sleepInterval=10;             //Initially the node will sleep 80 seconds
bool intervalSleepChanged=false;
int inbuildLed=0;

//********** POWER CONTROLLER PIN *************//
int POWER_5V_PIN8 = 8; //5 v
int POWER_12V_PIN9= 9; //12V

//********** ANALOG SENSORS PIN *************//
const int aLightPin=A0;
const int aLiquidLevelPin=A1;
const int aBattery=A2;
const int aVisPin=A3;

//**********  SENSORS VARIABLES *************//
uint32_t   temperature;
uint32_t   humidity;
uint32_t   light;
uint32_t   windDirection; 
uint32_t   windSpeedAverage;  //wind speed average last one minute
uint32_t   rainFallOneHour;  
uint32_t   barPressure;
uint32_t   liquidLevel;
uint32_t   visibility;
uint32_t   battery;

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  mySerial.begin(9600); //RX, TX
  //POWER SENSORS
  pinMode(POWER_5V_PIN8,OUTPUT);
  pinMode(POWER_12V_PIN9,OUTPUT);

  //Temperature and humidity sensor
  sht31.begin(0x44);
  warming12VSensors(); 
  digitalWrite(POWER_5V_PIN8,HIGH);
  delay(2000);
 
  while (!debugSerial && millis() < 10000);
  // INIT TTN INTERFACE if activated
  if(SEND_TO_TTN)
  {
    ttn.wake();
    ttn.showStatus();
    debugSerial.println("-- JOIN");
    ttn.join(appEui, appKey);
    //****Callback to receive downlink
    //used to update the status of the device. 
    ttn.onMessage(message);
 }
  debugSerial.println();
  Serial.println("OK setup!");
}

void loop()
{
  getBuffer();  //Begin from the SEN0186
  Serial.print("Wind Direction: ");
  windDirection=WindDirection()*100;
  Serial.print(windDirection/100);
  Serial.println("  ");
  Serial.print("Average Wind Speed (One Minute): ");
  windSpeedAverage=WindSpeedAverage()*100;
  Serial.print(windSpeedAverage/100);
  Serial.println("m/s  ");
  Serial.print("Max Wind Speed (Five Minutes): ");
  Serial.print(WindSpeedMax());
  Serial.println("m/s");
  Serial.print("Rain Fall (One Hour): ");
  rainFallOneHour=RainfallOneHour()*100;
  Serial.print(rainFallOneHour/100);
   Serial.println("mm  ");
  Serial.print("Barometric Pressure: ");
  barPressure=BarPressure()*100;
  Serial.print(barPressure/100);
  Serial.println("hPa");
  visibility=getVis()*100;
  debugSerial.print("Vis (Km): ");
  debugSerial.println(visibility/100);
  //***************** External temperature sensors *********************//  
  debugSerial.print("Temperature: ");
  temperature=sht31.readTemperature()*100;
  debugSerial.println(temperature/100);
  humidity=sht31.readHumidity()*100;
  debugSerial.print("Humidity: ");
  debugSerial.println(humidity/100);
  uint32_t light=getIntLight();
  debugSerial.print("Light: ");
  debugSerial.println(light);  
  
  //************** Battery sensor **************//
  uint32_t battery=getBattery()*100;
  debugSerial.print("Battery: ");
  debugSerial.println(battery/100);
  Serial.print(sleepInterval*8);
  Serial.println(" seconds");
  //Prepare LoRa packet if activated
  if(SEND_TO_TTN)
  {
      byte payload[20];   
      //Floats
      payload[0]=highByte(temperature);
      payload[1]=lowByte(temperature);
      payload[2]=highByte(humidity);
      payload[3]=lowByte(humidity);
      payload[4]=highByte(windSpeedAverage);
      payload[5]=lowByte(windSpeedAverage);
      payload[6]=highByte(windDirection);
      payload[7]=lowByte(windDirection);
      payload[8]=highByte(rainFallOneHour);
      payload[9]=lowByte(rainFallOneHour);
      payload[10]=highByte(barPressure);
      payload[11]=lowByte(barPressure);
      payload[12]=highByte(liquidLevel);
      payload[13]=lowByte(liquidLevel);
      payload[14]=highByte(battery); //voltage
      payload[15]=lowByte(battery); //
      payload[16]=highByte(visibility); //voltage
      payload[17]=lowByte(visibility); //    
      //Int analog readings
      payload[18]=light; //from 0 to 255
      //Send state: sleepTime
      payload[19]=highByte(sleepInterval); //voltage
      payload[20]=lowByte(sleepInterval); //
      ttn.sendBytes(payload,sizeof(payload));
      //******** SLEEP THE NODE ***************//
      ttn.sleep(sleepInterval * 8L * 1000L);
      Serial.print("****** SLEEPING LORA Module:");
      Serial.print(sleepInterval*8);
      Serial.println(" seconds");
  }
  //Sleeping sensors connected to POWER_12V_PIN
  digitalWrite(POWER_12V_PIN9,LOW);
  delay(1000); //To let print and PINs low correctly before sleeping 
   
   //********** Going to sleep power down ********/
   enterSleepTesting();
   //enterSleep(); //sleep TX_INTERVAL_SLEEP*8 seconds
   
   //******** WAKING UP THE NODE ***********//
   warming12VSensors(); 
   //Waking up LoRa module
   if (SEND_TO_TTN)
   {
      ttn.wake();
   }
   digitalWrite(POWER_5V_PIN8,HIGH);
   delay(2000);
   Serial.println("***** Just wake up! ******** ");

    /***************************************************************************/
    /********Check if the inital attemps of sending data have finished*********/
    if((!intervalSleepChanged) && (initialAttempsCounter < INITIAL_ATTEMPS))
    {
        initialAttempsCounter = initialAttempsCounter +1;
        if (initialAttempsCounter == INITIAL_ATTEMPS)
        {
            sleepInterval= DEFAULT_SLEEP_TEMP; 
         } 
     }
}
/*******************WARMING UP 12V SENSORS************************/
//Wind and optical sensors
void warming12VSensors(){
    //Wake up VIS sensor VIS_SENSOR_INTERVAL before the whole node
    if(WIND_SENSOR || VIS_SENSOR)
    {
      //Warming up VIS sensor
      digitalWrite(POWER_12V_PIN9,HIGH);
      if(VIS_SENSOR)
      {
        debugSerial.println("Warming up VIS optical sensor");
        delay(VIS_SENSOR_INTERVAL); //Use this time for avoiding sleeping mode*
      }
    }
}


//Analog light
int getIntLight(){
  //Measured from the analog pin
  int sensorVal=analogRead(aLightPin);
  int light=map(sensorVal,0,1023,0,255);
  return light;
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

 
  /***************************************************
 *  Name:        enterSleep
 *  Returns:     Nothing.
 *  Parameters:  intervals: intervals of 8 s that arduino will sleep.
 *  Description: Enters the arduino into sleep mode.
 ***************************************************/
void enterSleep(){
  for(int i=0; i<sleepInterval; i++){
      LowPower.powerDown(SLEEP_8S,ADC_OFF,BOD_OFF);  
    }
  }
  
void enterSleepTesting(){

    Serial.println("***** entra en sleeping testing! ******** ");
    long sleepIntervalTest= 8L * 1000L;
    //long sleepIntervalTest=sleepInterval * 8L * 1000L;
    Serial.println(sleepIntervalTest);
    delay(sleepIntervalTest);
    Serial.println("***** end of delay! ******** ");
 }

 void message(const uint8_t *payload, size_t size, port_t port)
 {
   
   if(payload[0])
   {
      Serial.println("New sleep interval defined to: ");
      sleepInterval=payload[0];
      Serial.println(sleepInterval);
      intervalSleepChanged=true;
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


 //DF robot weather station
char                 databuffer[35];
double               temp;

void getBuffer()  //Get weather status data
{
    Serial.println("En getBuffer ");
  int index;
  for (index = 0; index < 35; index ++)
  {
    if (mySerial.available())
    {
      databuffer[index] = mySerial.read();
      if (databuffer[0] != 'c')
      {
        index = -1;
      }
    }
    else
    {
      index --;
    }
  }
   int i;
  for (i=0;i<35;i++)
  {
    Serial.print(databuffer[i]);
  }
   Serial.println("");
}

int transCharToInt(char *_buffer, int _start, int _stop) //char to int?
{
  int _index;
  int result = 0;
  int num = _stop - _start + 1;
  int _temp[num];
  for (_index = _start; _index <= _stop; _index ++)
  {
    _temp[_index - _start] = _buffer[_index] - '0';
    result = 10 * result + _temp[_index - _start];
  }
  return result;
}

int WindDirection()   //Wind Direction
{
  return transCharToInt(databuffer, 1, 3);
}

float WindSpeedAverage()  //air Speed (1 minute)
{
  temp = 0.44704 * transCharToInt(databuffer, 5, 7);
  return temp;
}

float WindSpeedMax()  //Max air speed (5 minutes)
{
  temp = 0.44704 * transCharToInt(databuffer, 9, 11);
  return temp;
}

float Temperature()  //Temperature ("C")
{
  temp = (transCharToInt(databuffer, 13, 15) - 32.00) * 5.00 / 9.00;
  return temp;
}

float RainfallOneHour()  //Rainfall (1 hour)
{
  temp = transCharToInt(databuffer, 17, 19) * 25.40 * 0.01;
  return temp;
}

float RainfallOneDay()  //Rainfall (24 hours)
{
  temp = transCharToInt(databuffer, 21, 23) * 25.40 * 0.01;
  return temp;
}

int Humidity()  //Humidity
{
  return transCharToInt(databuffer, 25, 26);
}

float BarPressure()  //Barometric Pressure
{
  temp = transCharToInt(databuffer, 28, 32);
  return temp / 10.00;
}

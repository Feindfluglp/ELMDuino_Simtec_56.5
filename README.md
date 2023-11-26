This is a library that has been modified to read from Opel (and other??) vehicles that are equipped with an engine control unit of the brand Simtec 56.5.
It is still in alpha and is still being developed, so it is far from finished. But you can connect to a control unit and read out the raw data.
Since the original library has no function for reading out direct hex values, I used it for old cars and got the basic function to work.


https://github.com/PowerBroker2/ELMduino

Here you will find the original files, which can also be used to read out CAN and also have other protocols. Only ISO 14230-4 KWP(FAST and 5 baud) is directly supported in this library.

If anyone has any suggestions, ideas or other questions, just write here on Github.


### Installation
open arduino ide -> click sketch -> include libraries -> add zip libraries
Now select the zip file that you just downloaded.
Now you have an example under file -> examples -> ELMDuino_Simtec_56 -> Arduino_software_serial_test that you can use.


# Example Code:
```C++
#include <SoftwareSerial.h>
#include "ELMDuino_Simtec_565.h"


SoftwareSerial mySerial(5, 4); // RX, TX
#define ELM_PORT mySerial


ELM327_Simtec_565 myELM327;



void setup()
{
#if LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  Serial.begin(115200);
  ELM_PORT.begin(38400);
  
 
  Serial.println("Attempting to connect to ELM327...");

  if (!myELM327.begin(ELM_PORT, 5, true))
  {
    Serial.println("Couldn't connect to OBD scanner");
    while (1);
  }

  Serial.println("Connected to ELM327");
}


void loop()
{
  float rpm = myELM327.getSUCTION_TEMP_VCC();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    Serial.print("RPM: "); 
    Serial.println(rpm);
    delay(2000);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    myELM327.printError();
}
```


# Protocols:
```C++
4        - ISO 14230-4 KWP (5 baud init)
5        - ISO 14230-4 KWP (fast init)
```


# Functions:
```C++
bool get_vin();

float getECU_VCC();
int getMOTOR_RPM();
int getCAR_SPEED();
float getINJECTOR_PULSE();
int getREQUIRED_MOTOR_RPM();
float getLAMBDA_VCC();
float getCYL_IGN_ANGLE(int Cylinder = 0);
float getTROTTLE_POS();
int getIDLE_REGULATOR();
float getCOOLANT_TEMP_VCC();
float getSUCTION_TEMP_VCC();
```


### Description for Vehicle Indification Number (VIN)
The function to get the vehicle identification number you have to get_vin(); address and then read out the vin.

# Example Vin Code:
```C++
.....

void loop()
{
  myELM327.get_vin();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    Serial.print("VIN: "); 
    Serial.println(myELM327.vin);
    
.....
```


### Description for Cylinder Knock Delay (Function: getCYL_IGN_ANGLE)
In order to read the knock delay correctly, you have to pass on the cylinder number. There are a maximum of 4 cylinders that can be addressed. (I can't promise if there will be more)

# Example Knock Delay Code:
```C++
.....

void loop()
{
  float CylNockVar = myELM327.getCYL_IGN_ANGLE(1);

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    Serial.print("Cylinder Knock Delay: "); 
    Serial.println(CylNockVar);
    
.....
```


### Description ELM327 Connect
To change the protocol, change the 2 transfer variable to 4 (5 baud) or 5 (FAST).
The debug output can be changed to false or true using the 3 transfer variables.
You can also add a variable that can be transferred separated by a comma and that is the delay when fetching the data. between 1000 and 2000 is advisable. I use 2000 and have no problems reading it out.

# Example Knock Delay Code:
```C++
.....

  Serial.println("Attempting to connect to ELM327...");

  if (!myELM327.begin(ELM_PORT, 5, true, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner");
    while (1);
  }
  
.....
```

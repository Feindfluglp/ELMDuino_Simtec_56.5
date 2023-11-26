/*
 *  Modifcation Libary for Simtec 56.5 Engine Control Unit 
 *  The Orginal Autor is: PowerBroker2
 *  https://github.com/PowerBroker2/ELMduino
 *  
 *  Thx for Write all PowerBroker2
 */

#include <SoftwareSerial.h>
#include "ELMDuino_Simtec_565.h"


SoftwareSerial mySerial(5, 4); // RX, TX
#define ELM_PORT mySerial


ELM327_Simtec_565 myELM327;


float ECU_VCC = 0;


void setup()
{
#if LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  Serial.begin(115200);
  ELM_PORT.begin(38400);

  Serial.println("Attempting to connect to ELM327...");
/*
 *  0 - ELM_PORT
 *  1 - 4 is the Protocol ISO 14230-4 KWP (5 baud init)
 *      5 is the Protocol ISO 14230-4 KWP (fast init)
 *  2 - true or false for Debug all
 *  3 - 2000 is the Delay Range: 1000 - 2000
 *  The best Delay for me is 2000 (2 sec)
 */
  if (!myELM327.begin(ELM_PORT, 5, true, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner");
    while (1);
  }

  Serial.println("Connected to ELM327");
}


void loop()
{
  /*
   * For more Info show in Readme
   */
  ECU_VCC = myELM327.getECU_VCC();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    Serial.print("ECU VCC: "); Serial.println(ECU_VCC);
    delay(1000);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    myELM327.printError();
}

/*
	Read data from the engine control unit simtec 56.5 with the engine or similar

	Engines: X20XEV, X18XE
	Cylinder: is currently only for 4 cylinders (Knock Delay)

	- Opel Omega B
	- Opel Vectra B
	.....
	
	All cars with the Simtec 56.5 control unit should be able to read out the library

---------------------------------------------------------------------

	The source file from this library is here whit all function CAN:
	https://github.com/PowerBroker2/ELMduino

	Thx for Write all PowerBroker2

*/


#pragma once
#include "Arduino.h"

//-------------------------------------------------------------------------------------//
// AT commands (https://www.sparkfun.com/datasheets/Widgets/ELM327_AT_Commands.pdf)
//-------------------------------------------------------------------------------------//
const char * const DISP_DEVICE_DESCRIPT       = "AT @1";       // General
const char * const DISP_DEVICE_ID             = "AT @2";       // General
const char * const STORE_DEVICE_ID            = "AT @3 %s";    // General
const char * const REPEAT_LAST_COMMAND        = "AT \r";       // General
const char * const ALLOW_LONG_MESSAGES        = "AT AL";       // General
const char * const AUTOMATIC_RECEIVE          = "AT AR";       // OBD
const char * const ADAPTIVE_TIMING_OFF        = "AT AT0";      // OBD
const char * const ADAPTIVE_TIMING_AUTO_1     = "AT AT1";      // OBD
const char * const ADAPTIVE_TIMING_AUTO_2     = "AT AT2";      // OBD
const char * const DUMP_BUFFER                = "AT BD";       // OBD
const char * const BYPASS_INIT_SEQUENCE       = "AT BI";       // OBD
const char * const TRY_BAUD_DIVISOR           = "AT BRD %02d"; // General
const char * const SET_HANDSHAKE_TIMEOUT      = "AT BRT %02d"; // General
const char * const CALIBRATE_VOLTAGE_CUSTOM   = "AT CV %04d";  // Volts
const char * const RESTORE_CV_TO_FACTORY      = "AT CV 0000";  // Volts
const char * const SET_ALL_TO_DEFAULTS        = "AT D";        // General
const char * const DISP_CURRENT_PROTOCOL      = "AT DP";       // OBD
const char * const DISP_CURRENT_PROTOCOL_NUM  = "AT DPN";      // OBD
const char * const ECHO_OFF                   = "AT E0";       // General
const char * const ECHO_ON                    = "AT E1";       // General
const char * const FORGE_EVENTS               = "AT FE";       // General
const char * const PERFORM_FAST_INIT          = "AT FI";       // ISO
const char * const HEADERS_OFF                = "AT H0";       // OBD
const char * const HEADERS_ON                 = "AT H1";       // OBD
const char * const DISP_ID                    = "AT I";        // General
const char * const SET_ISO_BAUD_10400         = "AT IB 10";    // ISO
const char * const SET_ISO_BAUD_4800          = "AT IB 48";    // ISO
const char * const SET_ISO_BAUD_9600          = "AT IB 96";    // ISO
const char * const IREAD_IGNMON_INPUT_LEVEL   = "AT IGN";      // Other
const char * const SET_ISO_SLOW_INIT_ADDRESS  = "AT IIA %02d"; // ISO
const char * const DISP_KEY_WORDS             = "AT KW";       // ISO
const char * const KEY_WORD_CHECKING_OFF      = "AT KW0";      // ISO
const char * const KEY_WORD_CHECKING_ON       = "AT KW1";      // ISO
const char * const LINEFEEDS_OFF              = "AT L0";       // General
const char * const LINEFEEDS_ON               = "AT L1";       // General
const char * const LOW_POWER_MODE             = "AT LP";       // General
const char * const MEMORY_OFF                 = "AT M0";       // General
const char * const MEMORY_ON                  = "AT M1";       // General
const char * const MONITOR_ALL                = "AT MA";       // OBD
const char * const MONITOR_FOR_RECEIVER       = "AT MR %02d";  // OBD
const char * const MONITOR_FOR_TRANSMITTER    = "AT MT %02d";  // OBD
const char * const NORMAL_LENGTH_MESSAGES     = "AT NL";       // OBD
const char * const SET_PROTO_OPTIONS_AND_BAUD = "AT PB %s";    // OBD
const char * const PROTOCOL_CLOSE             = "AT PC";       // OBD
const char * const ALL_PROG_PARAMS_OFF        = "AT PP FF OFF";// PPs
const char * const ALL_PROG_PARAMS_ON         = "AT PP FF ON"; // PPs
const char * const SET_PROG_PARAM_OFF         = "AT PP %s OFF";// PPs
const char * const SET_PROG_PARAM_ON          = "AT PP %s ON"; // PPs
const char * const SET_PROG_PARAM_VAL         = "AT PP %s SV %s"; // PPs
const char * const DISP_PP_SUMMARY            = "AT PPS";      // PPs
const char * const RESPONSES_OFF              = "AT R0";       // OBD
const char * const RESPONSES_ON               = "AT R1";       // OBD
const char * const SET_RECEIVE_ADDRESS_TO     = "AT RA %02d";  // OBD
const char * const READ_STORED_DATA           = "AT RD";       // General
const char * const READ_VOLTAGE               = "AT RV";       // Volts
const char * const PRINTING_SPACES_OFF        = "AT S0";       // OBD
const char * const PRINTING_SPACES_ON         = "AT S1";       // OBD
const char * const STORE_DATA_BYTE            = "AT SD ";      // General
const char * const SET_HEADER                 = "AT SH %s";    // OBD
const char * const PERFORM_SLOW_INIT          = "AT SI";       // ISO
const char * const SET_PROTOCOL_TO_H_SAVE     = "AT SP %d";    // OBD
const char * const SET_REC_ADDRESS            = "AT SR %02d";  // OBD
const char * const SET_STANDARD_SEARCH_ORDER  = "AT SS";       // OBD
const char * const SET_TIMEOUT_TO_H_X_4MS     = "AT ST %s";  // OBD
const char * const SET_WAKEUP_TO_H_X_20MS     = "AT SW %s";  // ISO
const char * const SET_TESTER_ADDRESS_TO      = "AT TA %02d";  // OBD
const char * const TRY_PROT_H_AUTO_SEARCH     = "AT TP A%c";   // OBD
const char * const TRY_PROT_H                 = "AT TP %c";    // OBD
const char * const SET_WAKEUP_MESSAGE         = "AT WM";       // ISO
const char * const WARM_START                 = "AT WS";       // General
const char * const RESET_ALL                  = "AT Z";        // General




//-------------------------------------------------------------------------------------//
// Class constants
//-------------------------------------------------------------------------------------//
const int8_t QUERY_LEN	           = 8;
const int8_t ELM_SUCCESS           = 0;
const int8_t ELM_NO_RESPONSE       = 1;
const int8_t ELM_BUFFER_OVERFLOW   = 2;
const int8_t ELM_GARBAGE           = 3;
const int8_t ELM_UNABLE_TO_CONNECT = 4;
const int8_t ELM_NO_DATA           = 5;
const int8_t ELM_STOPPED           = 6;
const int8_t ELM_TIMEOUT           = 7;
const int8_t ELM_GETTING_MSG       = 8;
const int8_t ELM_MSG_RXD           = 9;
const int8_t ELM_GENERAL_ERROR     = -1;


// Non-blocking (NB) command states
typedef enum { SEND_COMMAND,
               WAITING_RESP,
               RESPONSE_RECEIVED,
               DECODED_OK,
               ERROR } obd_cmd_states;


class ELM327_Simtec_565
{
public:
	Stream* elm_port;

	bool connected = false;
	bool debugMode;
	char* payload;
	uint16_t PAYLOAD_LEN;
	int8_t nb_rx_state = ELM_GETTING_MSG;
	uint64_t response;
	uint16_t recBytes;
	uint8_t numPayChars;
	uint16_t timeout_ms;
	char vin[20];
	char ClearArray[84];

	// is currently only for 4 cylinders (Knock Delay)
	const int CylinderNum = 4;
	byte responseByte_0;
	byte responseByte_1;
	byte responseByte_2;
	byte responseByte_3;
	byte responseByte_4;
	byte responseByte_5;
	byte responseByte_6;
	byte responseByte_7;



	bool begin(Stream& stream, int SelectProtocol, const bool& debug = false, const uint16_t& timeout = 2000, const uint16_t& payloadLen = 200 );
	bool initializeELM(int SelectProtocol);
	void flushInputBuff();
	uint64_t findResponse();
	bool queryPID(const uint8_t& service, const uint16_t& pid, const uint8_t& num_responses = 1);
	bool queryPID(char queryStr[]);
	void sendCommand(const char *cmd);
	int8_t sendCommand_Blocking(const char *cmd);
	int8_t get_response();
	bool timeout();
	void printError();

	// Get data from car
	bool get2101Data();
	bool get_vin();

	// Get 2101 Data
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

	// Convert function
	int ConvertHextoMathvalue(char HexValue1, char HexValue2);



private:
	char query[QUERY_LEN] = { '\0' };
	bool longQuery = false;
	uint32_t currentTime;
	uint32_t previousTime;

	obd_cmd_states nb_query_state = SEND_COMMAND; // Non-blocking query state

	void upper(char string[], uint8_t buflen);
	void formatQueryArray(uint8_t service, uint16_t pid, uint8_t num_responses);
	uint8_t ctoi(uint8_t value);
	int8_t nextIndex(char const *str,
	                 char const *target,
	                 uint8_t numOccur);
};

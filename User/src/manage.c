#include "manage.h"



const char FirmwareVer[] = "3.33";
const char EEPROMVer[]  = "2.00";
const char MCUVer[] = "STM32F103C8T6";

unsigned short  g_RunTime = 0;

unsigned short  g_BatVolt=0;

unsigned char g_CarRunningMode = INFRARED_TRACE_MODE;

//���¶�����manage.h��
//#define CONTROL_MODE 			1   
//#define INFRARED_TRACE_MODE 	2    
//#define ULTRA_FOLLOW_MODE		3       
//#define ULTRA_AVOID_MODE	   4      

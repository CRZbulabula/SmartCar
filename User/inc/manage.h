/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://shop275516297.taobao.com/
�� �� ��: 	manage.h
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	




***********************************************************************/


#ifndef _MANAGE_H
#define _MANAGE_H



#define CONTROL_MODE 			1
#define INFRARED_TRACE_MODE 	2
#define ULTRA_FOLLOW_MODE		3
#define ULTRA_AVOID_MODE	   4

extern const char FirmwareVer[];
extern const char EEPROMVer[];
extern const char MCUVer[];

extern unsigned short  g_RunTime;
extern unsigned short g_BatVolt;
extern unsigned char   g_CarRunningMode;




#endif


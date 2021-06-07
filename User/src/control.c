/**********************************************************************
��Ȩ���У�	  ����ʵ����MiaowLabs��2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	  control.c
��    ��:   ����ʵ����MiaowLabs
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	


***********************************************************************/
#include "math.h"
#include "stdio.h"
#include "control.h"
#include "debug.H"
#include "MPU6050.H"
#include "communicate.h"
#include "bsp.h"
#include "ultrasonic.h"
#include "infrare.h"

unsigned char g_u8MainEventCount;
unsigned char g_u8SpeedControlCount;
unsigned char g_u8SpeedControlPeriod;
unsigned char g_u8DirectionControlPeriod;
unsigned char g_u8DirectionControlCount;

unsigned char g_cMotorDisable = 0;//ֵ����0ʱ�������ת��������ֹͣת��


int g_iGravity_Offset = 0;

/******������Ʋ���******/
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;

// �����ַֿ�����
float g_fLeftSpeedControlOut, g_fLeftSpeedControlOutNew, g_fLeftSpeedControlOutOld;
float g_fRightSpeedControlOut, g_fRightSpeedControlOutNew, g_fRightSpeedControlOutOld;


/******�ٶȿ��Ʋ���******/
#define MOVE_START_SPEED 50 // ��������
#define MOVE_MAX_SPEED 30   // �н��������ֵ

int MOVE_CONTROL = 0;

short  g_s16LeftMotorPulse;
short  g_s16RightMotorPulse;

int  g_s32LeftMotorPulseOld;
int  g_s32RightMotorPulseOld;
int  g_s32LeftMotorPulseSigma;
int  g_s32RightMotorPulseSigma;

// �����ַֿ�����
float g_iCarLeftSpeedSet, g_iCarRightSpeedSet; // �����������ٶ�
float g_fCarLeftSpeed, g_fCarRightSpeed;
float g_fCarLeftSpeedOld, g_fCarRightSpeedOld;
float g_fCarLeftPosition, g_fCarRightPosition;

/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*/
PID_t g_tCarAnglePID={17.0, 0, 23.0};	//*5 /10
PID_t g_tCarSpeedPID={15.25, 1.08, 0};	//i/10
/******�������Ʋ���******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;

float g_fCarAngle;         	//
float g_fGyroAngleSpeed;		//     			
float g_fGravityAngle;			//

// ֱ�м���
int g_iMoveCnt = 0;

// ת�����
int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;

int g_iDestinationRelatedDirection = 0;	// ������յ�ķ���-1��ʾ��ǰ����������  0��ʾ��ǰ����ֱ��    1��ʾ��ǰ����������
int g_iWallRelatedPosition = 0;			// �����ǽ��λ�ã�  -1��ʾ�����ǽ��ǰ��  1��ʾ���ұ�ǽ��ǰ��  0��ʾ�������

static int AbnormalSpinFlag = 0;

/***************************************************************
** ��������: CarUpstandInit
** ��������: ȫ�ֱ�����ʼ������
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/
void CarUpstandInit(void)
{
	//g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
	g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
	g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

	g_fCarLeftSpeed = g_fCarRightSpeed = 0;
	g_fCarLeftPosition = g_fCarRightPosition = 0;
	g_fCarAngle = 0;
	g_fGyroAngleSpeed = 0;
	g_fGravityAngle = 0;

	g_fLeftSpeedControlOut = g_fRightSpeedControlOut = 0;
	g_fAngleControlOut = g_fBluetoothDirectionOut = 0;
	g_fLeftMotorOut = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed = g_fBluetoothDirection = 0;
	g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

	g_u8MainEventCount = 0;
	g_u8SpeedControlCount = 0;
	g_u8SpeedControlPeriod = 0;
}


/***************************************************************
** ��������: AbnormalSpinDetect
** ��������: ���ת���쳣���      
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
***************************************************************/

void AbnormalSpinDetect(short leftSpeed,short rightSpeed)
{
	static unsigned short count = 0;
	
	//�ٶ�����Ϊ0ʱ��⣬���򲻼��
	if(g_iCarLeftSpeedSet == 0 && g_iCarRightSpeedSet == 0)
	{
		if(((leftSpeed>30)&&(rightSpeed>30)&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
			||((leftSpeed<-30)&&(rightSpeed<-30))&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
		{// ���ҵ��ת�ٴ���30��������ͬ������ʱ�䳬��250ms���ҳ����ǶȲ�����30�ȣ����ж�Ϊ���տ�ת
			count++;
			if(count>50){
				count = 0;
				AbnormalSpinFlag = 1;
			}
		}
		else{
			count = 0;
		}
	}
	else{
		count = 0;
	}
}

/***************************************************************
** ��������: LandingDetect
** ��������: С���ŵؼ��      
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
***************************************************************/
void LandingDetect(void)
{
	static float lastCarAngle = 0;
	static unsigned short count = 0,count1 = 0;
	
	if(AbnormalSpinFlag == 0)return;
	
	// С���Ƕ�5��~-5���������
	if((g_fCarAngle > -5) && (g_fCarAngle < 5))
	{
		count1++;
		if(count1 >= 50)
		{//ÿ��250ms�ж�һ��С���Ƕȱ仯�����仯��С��0.8������-0.8���ж�ΪС����ֹ
			count1 = 0;
			if(((g_fCarAngle - lastCarAngle) < 0.8) && ((g_fCarAngle - lastCarAngle) > -0.8))
			{
				count++;
				if(count >= 4){
					count = 0;
					count1 = 0;
					g_fCarLeftPosition = g_fCarRightPosition = 0;
					AbnormalSpinFlag = 0;
				}
			}
			else{
				count = 0;
			}
			lastCarAngle = g_fCarAngle;
		}
	}
	else
	{
		count1 = 0;
		count = 0;
	}
}

/***************************************************************
** ��������: MotorManage
** ��������: ���ʹ��/ʧ�ܿ���      
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
***************************************************************/
void MotorManage(void)
{

	AbnormalSpinDetect(g_s16LeftMotorPulse, g_s16RightMotorPulse);
		
	LandingDetect();
	
	if(AbnormalSpinFlag)
	{	
		g_cMotorDisable |= (0x01<<1);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<1);
	}
	
	if(g_fCarAngle > 30 || g_fCarAngle < (-30))
	{
		g_cMotorDisable |= (0x01<<2);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<2);
	}
	
}

/***************************************************************
** ��������: SetMotorVoltageAndDirection
** ��������: ���ת�ټ�������ƺ���             
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2018��08��27��
***************************************************************/
void SetMotorVoltageAndDirection(int i16LeftVoltage,int i16RightVoltage)
{
	  if(i16LeftVoltage<0)
    {	
			GPIO_SetBits(GPIOA, GPIO_Pin_3 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_4 );
      i16LeftVoltage = (-i16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOA, GPIO_Pin_4 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_3 ); 
    }

    if(i16RightVoltage<0)
    {	
     	GPIO_SetBits(GPIOB, GPIO_Pin_0 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_1 );
      i16RightVoltage = (-i16RightVoltage);
    }
    else
    {
			GPIO_SetBits(GPIOB, GPIO_Pin_1 );				    
			GPIO_ResetBits(GPIOB, GPIO_Pin_0 );	      
    }

	if(i16RightVoltage > MOTOR_OUT_MAX)  
	{
		i16RightVoltage = MOTOR_OUT_MAX;
	}
	if(i16LeftVoltage > MOTOR_OUT_MAX)
	{
	   i16LeftVoltage = MOTOR_OUT_MAX;
	}  
	
	if(g_cMotorDisable)
	{
		TIM_SetCompare1(TIM3,0);
		TIM_SetCompare2(TIM3,0); 
	}
	else
	{
		TIM_SetCompare1(TIM3,i16RightVoltage);
		TIM_SetCompare2(TIM3,i16LeftVoltage);
	}
}


/***************************************************************
** ��������: MotorOutput
** ��������: ����������
             ��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,����
			 �������������������������������
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/ 
** �ա���:   2014��08��01��
***************************************************************/
void MotorOutput(void)
{
	// ����ĵ��������ڽǶȻ������� + �ٶȻ��⻷,����� - g_fSpeedControlOut 
	// ����Ϊ�ٶȻ��ļ��Ը��ǶȻ���һ�����ǶȻ��Ǹ��������ٶȻ���������
	g_fLeftMotorOut  = g_fAngleControlOut - g_fLeftSpeedControlOut - g_fBluetoothDirection ;	
	g_fRightMotorOut = g_fAngleControlOut - g_fRightSpeedControlOut + g_fBluetoothDirection ;


	/*������������*/
	if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*������ʹ�������ֹ����PWM��Χ*/			
	if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((int)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((int)g_fLeftMotorOut, (int)g_fRightMotorOut);
}

void GetMotorPulse(void)  //�ɼ�����ٶ�����
{ 	
	g_s16LeftMotorPulse = TIM_GetCounter(TIM2);     
	g_s16RightMotorPulse = -TIM_GetCounter(TIM4);
	TIM2->CNT = 0;
	TIM4->CNT = 0;   //����

	g_s32LeftMotorPulseSigma +=  g_s16LeftMotorPulse;
	g_s32RightMotorPulseSigma += g_s16RightMotorPulse; 
	
	g_iMoveCnt -= (g_s16LeftMotorPulse + g_s16RightMotorPulse) * 0.5;
	g_iLeftTurnRoundCnt -= g_s16LeftMotorPulse;
	g_iRightTurnRoundCnt -= g_s16RightMotorPulse;
}

/***************************************************************
** ����  ��: MiaowLabs Team
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 2015��11��29��
** ��������: AngleCalculate
** ��������: �ǶȻ����㺯��           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ����MiaowLabs��Ȩ����**************************
***************************************************************/
void AngleCalculate(void)
{
	//-------���ٶ�--------------------------
	//����Ϊ��2gʱ�������ȣ�16384 LSB/g
	g_fGravityAngle = atan2(g_fAccel_y/16384.0,g_fAccel_z/16384.0) * 180.0 / 3.14;
	g_fGravityAngle = g_fGravityAngle - g_iGravity_Offset;

	//-------���ٶ�-------------------------
	//��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	g_fGyro_x  = g_fGyro_x / 16.4;  //������ٶ�ֵ			   
	g_fGyroAngleSpeed = g_fGyro_x;	
	
	//-------�����˲�---------------
	g_fCarAngle = 0.98 * (g_fCarAngle + g_fGyroAngleSpeed * 0.005) + 0.02 *	g_fGravityAngle;

	// ��е���
	//g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;
}
/***************************************************************
** ����  ��: ����ʵ����MiaowLabs
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 2018��08��27��
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ����MiaowLabs��Ȩ����**************************
***************************************************************/
void AngleControl(void)	 
{
	g_fAngleControlOut = (CAR_ANGLE_SET - g_fCarAngle) * g_tCarAnglePID.P *5 + \
	(CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * (g_tCarAnglePID.D / 10);
}

/***************************************************************
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/

void SpeedControl(void)
{
	float fP, fI;   	
	float fDelta;
	
	// �����ֱַ����
	g_fCarLeftSpeed = g_s32LeftMotorPulseSigma;
	g_fCarLeftSpeed = 0.7 * g_fCarLeftSpeedOld + 0.3 * g_fCarLeftSpeed; //��ͨ�˲���ʹ�ٶȸ�ƽ��
	g_fCarLeftSpeedOld = g_fCarLeftSpeed;
	g_fCarRightSpeed = g_s32RightMotorPulseSigma;
	g_fCarRightSpeed = 0.7 * g_fCarRightSpeedOld + 0.3 * g_fCarRightSpeed; //��ͨ�˲���ʹ�ٶȸ�ƽ��
	g_fCarRightSpeedOld = g_fCarRightSpeed;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0; //ȫ�ֱ��� ע�⼰ʱ����

	// ���ֵ���
	fDelta = CAR_LEFT_SPEED_SET;
	fDelta -= g_fCarLeftSpeed;
	fP = fDelta * (g_tCarSpeedPID.P);
	fI = fDelta * (g_tCarSpeedPID.I / 10.0);
	g_fCarLeftPosition += fI;
	//g_fCarLeftPosition += g_fBluetoothSpeed; // ��ʱ����

	//������������
	if((s16) g_fCarLeftPosition > CAR_POSITION_MAX) g_fCarLeftPosition = CAR_POSITION_MAX;
	if((s16) g_fCarLeftPosition < CAR_POSITION_MIN) g_fCarLeftPosition = CAR_POSITION_MIN;

	// �������
	g_fLeftSpeedControlOutOld = g_fLeftSpeedControlOutNew;
	g_fLeftSpeedControlOutNew = fP + g_fCarLeftPosition;

	// ���ֵ���
	fDelta = CAR_RIGHT_SPEED_SET;
	fDelta -= g_fCarRightSpeed;
	fP = fDelta * (g_tCarSpeedPID.P);
	fI = fDelta * (g_tCarSpeedPID.I / 10.0);
	g_fCarRightPosition += fI;
	//g_fCarRightPosition += g_fBluetoothSpeed; // ��ʱ����  
	
	//������������
	if((s16) g_fCarRightPosition > CAR_POSITION_MAX) g_fCarRightPosition = CAR_POSITION_MAX;
	if((s16) g_fCarRightPosition < CAR_POSITION_MIN) g_fCarRightPosition = CAR_POSITION_MIN;
	
	// �������
	g_fRightSpeedControlOutOld = g_fRightSpeedControlOutNew;
	g_fRightSpeedControlOutNew = fP + g_fCarRightPosition;
}


/***************************************************************
** ��������: SpeedControlOutput
** ��������: �ٶȻ������������-�ֶಽ��αƽ���������������ܽ���ֱ�����ĸ��Ž��͡�
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/
void SpeedControlOutput(void)
{
  float fLeftValue, fRightValue;
  fLeftValue = g_fLeftSpeedControlOutNew - g_fLeftSpeedControlOutOld;
  g_fLeftSpeedControlOut = fLeftValue * (g_u8SpeedControlPeriod + 1) / 
  						   SPEED_CONTROL_PERIOD + g_fLeftSpeedControlOutOld;
		 
  fRightValue = g_fRightSpeedControlOutNew - g_fRightSpeedControlOutOld;
  g_fRightSpeedControlOut = fRightValue * (g_u8SpeedControlPeriod + 1) / 
  						   SPEED_CONTROL_PERIOD + g_fRightSpeedControlOutOld;
}


/***************************************************************
** ��������: Scale
** ��������: ���̹�һ������
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/
float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

/***************************************************************
** ��������: Steer
** ��������: ң���ٶȼ�����������
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/
void Steer(float direct, float speed)
{
	if(direct > 0)
		g_fBluetoothDirection = Scale(direct, 0, 10, 0, 400);
	else
		g_fBluetoothDirection = -Scale(direct, 0, -10, 0, 400);

	if(speed > 0) 
		g_iCarLeftSpeedSet = g_iCarRightSpeedSet = Scale(speed, 0, 10, 0, 70);
	else
		g_iCarLeftSpeedSet = g_iCarRightSpeedSet = -Scale(speed, 0, -10, 0, 70);

}

/***************************************************************
** ����  ��: Songyibiao
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 20160415
** ��������: UltraControl
** ��������: ����������/����           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ����MiaowLabs��Ȩ����**************************/
void UltraControl(int mode)
{
	// #if INFRARE_DEBUG_EN > 0
	// char buff[32];	
	// #endif

	// ����ģʽ
	if(mode == 0)
	{
		if((Distance >= 0) && (Distance<= 12))
		{//����С��12cm�����
			Steer(0, -4);
		}
		else if((Distance> 18) && (Distance<= 30))	
		{//�������18cm��С��30��ǰ��
			Steer(0, 4);
		}
		else
			Steer(0, 0);
	}
	else if(mode == 1)
	{

		// #if INFRARE_DEBUG_EN > 0
		// sprintf(buff, "22222\n");
		// DebugOutStr(buff);
		// #endif

		if((Distance >= 0) && (Distance<= 20))
		{
			// #if INFRARE_DEBUG_EN > 0
			// sprintf(buff, "33333\n");
			// DebugOutStr(buff);
			// #endif
			if(g_iDestinationRelatedDirection == 0)			// ��ǰ����յ�����ֱ��
			{
				if (g_iWallRelatedPosition == 0 || g_iWallRelatedPosition == 1)
				{
					//��ת750�����������ת��Ƕ�ԼΪ90��(Ĭ��Ҳ����ת)
					Steer(5, 0);
					g_iLeftTurnRoundCnt = 750;
					g_iRightTurnRoundCnt = -750;
				}
				else
				{
					// ��ת750�����������ת��Ƕ�ԼΪ90��
					Steer(-5, 0);
					g_iLeftTurnRoundCnt = -750;
					g_iRightTurnRoundCnt = 750;
				}
			}
			else if(g_iDestinationRelatedDirection == -1)	// ��ǰ����յ����������ߣ����Ҽ���ײǽ
			{
				//��ת750�����������ת��Ƕ�ԼΪ90��
				Steer(5, 0);
				g_iLeftTurnRoundCnt = 750;
				g_iRightTurnRoundCnt = -750;
			}
			else											// ��ǰ����յ����������ߣ����Ҽ���ײǽ
			{
				// ��ת750�����������ת��Ƕ�ԼΪ90��
				Steer(-5, 0);
				g_iLeftTurnRoundCnt = -750;
				g_iRightTurnRoundCnt = 750;
			}
		}
		// ת�����
		if(g_iDestinationRelatedDirection == 0)
		{
			if ((g_iWallRelatedPosition == 0 || g_iWallRelatedPosition == 1) && g_iLeftTurnRoundCnt < 0 && g_iRightTurnRoundCnt > 0)
			{
				// #if INFRARE_DEBUG_EN > 0
				// sprintf(buff, "44444\n");
				// DebugOutStr(buff);
				// #endif
				// ֱ��ͨ����ת��Ϊ������
				g_iDestinationRelatedDirection = 1;
				g_iWallRelatedPosition = 0;
				Steer(0, 4);
			}
			else if (g_iDestinationRelatedDirection == -1 && g_iLeftTurnRoundCnt > 0 && g_iRightTurnRoundCnt < 0)
			{
				// #if INFRARE_DEBUG_EN > 0
				// sprintf(buff, "44444\n");
				// DebugOutStr(buff);
				// #endif
				// ֱ��ͨ����ת��Ϊ������
				g_iDestinationRelatedDirection = -1;
				g_iWallRelatedPosition = 0;
				Steer(0, 4);
			}
		}
		else if(g_iDestinationRelatedDirection == -1 && g_iLeftTurnRoundCnt < 0 && g_iRightTurnRoundCnt > 0)
		{
			// #if INFRARE_DEBUG_EN > 0
			// 	sprintf(buff, "44444\n");
			// 	DebugOutStr(buff);
			// 	#endif
			// ������ͨ����ת�ָ���ֱ�� ��ǽ�����
			g_iDestinationRelatedDirection = 0;
			g_iWallRelatedPosition = -1;
			Steer(0, 4);
		}
		else if(g_iDestinationRelatedDirection == 1 && g_iLeftTurnRoundCnt > 0 && g_iRightTurnRoundCnt < 0)
		{
			// #if INFRARE_DEBUG_EN > 0
			// 	sprintf(buff, "44444\n");
			// 	DebugOutStr(buff);
			// 	#endif
			// ������ͨ����ת�ָ���ֱ�� ��ǽ���ұ�
			g_iDestinationRelatedDirection = 0;
			g_iWallRelatedPosition = 1;
			Steer(0, 4);
		}
	}
}

/***************************************************************
** ����  ��: MiaowLabs Team
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 20160415
** ��������: TailingControl
** ��������: ����Ѱ��           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ����MiaowLabs��Ȩ����**************************
***************************************************************/
void TailingControl(void)
{
#if INFRARE_DEBUG_EN > 0
	char buff[32];	
#endif
	char result;
	float direct = 0;
	float speed = 0;

	result = InfraredDetect();
	
	// Lb and Rb is not available
	if(result & infrared_channel_Lc)
		direct -= 6;
	if(result & infrared_channel_La)
		direct -= 4;
	if(result & infrared_channel_Ra)
		direct += 4;
	if (result & infrared_channel_Rc)
		direct += 6;

	if (direct == 0) speed = 1;
	else speed = 2;

	Steer(direct, speed);

#if INFRARE_DEBUG_EN > 0
	sprintf(buff, "Steer:%d, Speed:%d\r\n",(int)direct,  (int)speed);
	DebugOutStr(buff);
#endif
}
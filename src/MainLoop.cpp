/*---------------------------------------------------------*/
/*
   	Weixu ZHU (Harry)
   		zhuweixu_harry@126.com
	
	Version 1.0
	Version 1.1 : change opengl debug layout

*/
/*---------------------------------------------------------*/

#include <stdio.h>
#include "GLTools.h"

#include "Vector3.h"
#include "Quaternion.h"

#define PI 3.1415926

#include "packet_control_interface.h"

/*------------ Function Control Channels ------------*/
double CH1 = 0, CH1_MAX = 60, CH1_MIN = -60, CH1_STEP = 0.3;
double CH2 = 0, CH2_MAX = 60, CH2_MIN = -60, CH2_STEP = 0.3;
int CMDCH1 = 0;
int CMDCH2 = 0;

/* --------------- uart declare --------------------*/
int uartOpen();
int uartSendSpeed(int16_t L, int16_t R);
int uartGetSpeed(int16_t *L, int16_t *R);
int uartSendPIDParams(float kp, float ki, float kd);
int uartGetSpeedWithDebug(int16_t *L, int16_t *R, int16_t *LError, float *LOutput);

/* --------------- variables --------------------*/
int16_t tL, tR;
int16_t mL, mR;
float Kp, Ki, Kd;

/* --------------- MainLoop functions --------------------*/
int function_exit()
{
	uartSendSpeed(0,0);
	return 0;
}

int function_init()
{
	tL = 0;
	tR = 0;
	Kp = 0;
	Ki = 0;
	Kd = 0;

	uartOpen();
	
	dataCount = 0;

	return 0;
}

int function_step(double time)	// time in s
{
	tL = (int)CH1;
	tR = (int)CH2;

	if (CMDCH1 == 1)
	{
		scanf("%d %d", &tL, &tR);
		CH1 = tL; CH2 = tR;
		CMDCH1 = 0;
	}
	if (CMDCH2 == 1)
	{
		float kp, ki, kd;
		scanf("%f %f %f", &kp, &ki, &kd);
		uartSendPIDParams(kp, ki, kd);
		CMDCH2 = 0;
	}

	int16_t eL;
	float oL;

	uartSendSpeed(tL, tR);
	//uartGetSpeedWithDebug(&mL, &mR, &eL, &oL);
	uartGetSpeed(&mL, &mR);

	dataCount++; if (dataCount >= MAXLOG) dataCount = 0;
	dataLog[0][dataCount] = tL;
	dataLog[1][dataCount] = mL;
	dataLog[2][dataCount] = oL;

	dataLog[3][dataCount] = tR;
	dataLog[4][dataCount] = mR;
		printf("t = %4d \tm = %4d \te = %4d \to = %5f\n",tL, mL, eL, oL);
		
	return 0;
}

/* --------------- OpenGL draw functions --------------------*/
int function_draw()
{
	drawPlot(dataCount - 100, dataCount, 0, -1, 0);	// tL
	drawPlot(dataCount - 100, dataCount, 1,	-1, 0);	// mL
	drawPlot(dataCount - 100, dataCount, 2,	-1, 0);	// oL

	drawPlot(dataCount - 100, dataCount, 3,	0,  0);	// tR
	drawPlot(dataCount - 100, dataCount, 4,	0,  0);	// mR
	return 0;
}

int function_draw2()
{
	return 0;
}

/*---- uart --------------------------------------------------*/
CPacketControlInterface *ddsInterface, *pmInterface;
int uartOpen()
{
	pmInterface =
		new CPacketControlInterface("dds", "/dev/ttyUSB0", 57600);
	ddsInterface =
		new CPacketControlInterface("pm", "/dev/ttyUSB1", 57600);
	if (!ddsInterface->Open())
		{   printf("SC0 not open\n");   return -1;  }
	if (!pmInterface->Open())
		{   printf("SC1 not open\n");   return -1;  }
	printf("---Establish Connection with MCUs---------------------\n");

	printf("---Initialize Actuator---------------------\n");
	enum class EActuatorInputLimit : uint8_t {
		LAUTO = 0, L100 = 1, L150 = 2, L500 = 3, L900 = 4
	};

	/* Override actuator input limit to 100mA */
	pmInterface->SendPacket(
			CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
			static_cast<const uint8_t>(EActuatorInputLimit::L900));

	/* Enable the actuator power domain */
	pmInterface->SendPacket(
			CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, 
			true);
				      
	/* Power up the differential drive system */
	ddsInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, true);

	/* Initialize the differential drive system */
	uint8_t pnStopDriveSystemData[] = {0, 0, 0, 0};
	ddsInterface->SendPacket(
			CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
			pnStopDriveSystemData,
			sizeof(pnStopDriveSystemData));
	printf("initialize complete\n");

	return 0;
}
int uartSendSpeed(int16_t L, int16_t R)
{
	uint8_t uartData[] = {
		reinterpret_cast<uint8_t*>(&L)[1],
		reinterpret_cast<uint8_t*>(&L)[0],
		reinterpret_cast<uint8_t*>(&R)[1],
		reinterpret_cast<uint8_t*>(&R)[0],
	};
	ddsInterface->SendPacket(
		CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
		uartData,
		sizeof(uartData));

	return 0;
}
int uartSendPIDParams(float kp, float ki, float kd)
{
	uint8_t uartData[] = {
		reinterpret_cast<uint8_t*>(&kp)[3],
		reinterpret_cast<uint8_t*>(&kp)[2],
		reinterpret_cast<uint8_t*>(&kp)[1],
		reinterpret_cast<uint8_t*>(&kp)[0],

		reinterpret_cast<uint8_t*>(&ki)[3],
		reinterpret_cast<uint8_t*>(&ki)[2],
		reinterpret_cast<uint8_t*>(&ki)[1],
		reinterpret_cast<uint8_t*>(&ki)[0],

		reinterpret_cast<uint8_t*>(&kd)[3],
		reinterpret_cast<uint8_t*>(&kd)[2],
		reinterpret_cast<uint8_t*>(&kd)[1],
		reinterpret_cast<uint8_t*>(&kd)[0],
	};
	ddsInterface->SendPacket(
		CPacketControlInterface::CPacket::EType::SET_DDS_PARAMS,
		uartData,
		sizeof(uartData));

	return 0;
}
int uartGetSpeed(int16_t *L, int16_t *R)
{
	int16_t getL, getR;
	ddsInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_DDS_SPEED);
	if(ddsInterface->WaitForPacket(200, 3)) 
	{
		if(ddsInterface->GetState() == CPacketControlInterface::EState::RECV_COMMAND)
		{
			//printf("received command\n");
			const CPacketControlInterface::CPacket& cPacket = ddsInterface->GetPacket();
			if (cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_DDS_SPEED)
			{
				//printf("packettype: 0x%x\n",(int)cPacket.GetType());
				if(cPacket.GetDataLength() == 4) 
				{
					const uint8_t* punPacketData = cPacket.GetDataPointer();
					reinterpret_cast<int16_t&>(getL) = punPacketData[0]<<8 | punPacketData[1];
					reinterpret_cast<int16_t&>(getR) =punPacketData[2]<<8 | punPacketData[3];
					*L = getL; *R = getR;
					return 0;
				} 
			}
		}
	}
	return -1;
}

int uartGetSpeedWithDebug(int16_t *L, int16_t *R, int16_t *LError, float *LOutput)
{
	int16_t getL, getR, getLError;
	float getLOutput;
	ddsInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_DDS_SPEED);
	if(ddsInterface->WaitForPacket(200, 3)) 
	{
		if(ddsInterface->GetState() == CPacketControlInterface::EState::RECV_COMMAND)
		{
			//printf("received command\n");
			const CPacketControlInterface::CPacket& cPacket = ddsInterface->GetPacket();
			if (cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_DDS_SPEED)
			{
				//printf("packettype: 0x%x\n",(int)cPacket.GetType());
				if(cPacket.GetDataLength() == 10) 
				{
					const uint8_t* punPacketData = cPacket.GetDataPointer();
					reinterpret_cast<int16_t&>(getL) = punPacketData[0]<<8 | punPacketData[1];
					reinterpret_cast<int16_t&>(getR) =punPacketData[2]<<8 | punPacketData[3];
					reinterpret_cast<int16_t&>(getLError) = punPacketData[4]<<8 | punPacketData[5];
					reinterpret_cast<int32_t&>(getLOutput) = punPacketData[6]<<24 | 
															punPacketData[7]<<16 |
															punPacketData[8]<<8 |
															punPacketData[9];
					*L = getL; *R = getR;
					*LError = getLError;
					*LOutput = getLOutput;
					return 0;
				} 
			}
		}
	}
	return -1;
}

/* --------------- draw obj --------------------*/
#ifndef BOX
#include "Box.h"
void Box::draw()
{
	Vector3 axis = this->q.getAxis();
	double ang = this->q.getAng();

	glTranslatef(this->l.x, this->l.y, this->l.z);
	glRotatef(ang*180/PI,axis.x,axis.y,axis.z);
	if ((this->x != 0) && (this->y != 0) && (this->z != 0))
	{
		glScalef(this->x, this->y, this->z);
		glutSolidCube(1);	
		glScalef(1/this->x, 1/this->y, 1/this->z);
	}
	glRotatef(-ang*180/PI,axis.x,axis.y,axis.z);
	glTranslatef(-this->l.x, -this->l.y, -this->l.z);
}
#endif

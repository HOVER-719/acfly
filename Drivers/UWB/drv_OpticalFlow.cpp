#include "drv_Uart5.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

typedef struct
{
	uint8_t id;
	uint8_t role;
	int pos_x:24;	int pos_y:24;	int pos_z:24;		
	int vel_x:24;	int vel_y:24;	int vel_z:24;	
	int dis_0:24;	int dis_1:24;	int dis_2:24;	int dis_3:24;	int dis_4:24;	int dis_5:24;	int dis_6:24;	int dis_7:24;
	float imuGyro[3];
	float imuAcc[3];
	uint8_t reserved1[12];
	int16_t angle[3];
	float q[4];
	uint8_t reserved2[4];
	uint32_t localTime;
	uint32_t systemTime;
	uint8_t reserved3[1];
	uint8_t eop[3];
	uint16_t voltage;
	uint8_t reserved4[5];
}__PACKED _Uwb;
static const unsigned char packet_ID[2] = { 0x55 , 0x01 };

static void OpticalFlow_Server(void* pvParameters)
{
	/*状态机*/
		_Uwb  Uwb;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
		if( Read_Uart5( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < sizeof(packet_ID) )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
				{
					rc_counter = 0;
					sum = 0;
				}
				else
				{
					++rc_counter;
					sum += rdata;
				}
			}
			else if( rc_counter < sizeof(packet_ID) + sizeof(_Uwb) )
			{	//接收数据
				( (unsigned char*)&Uwb )[ rc_counter - sizeof(packet_ID) ] = rdata;
				sum += rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( sum == rdata )
				{
					
					if( Uwb.eop[0]>200 || Uwb.eop[1]>200 )
						PositionSensorSetInavailable(default_optical_flow_index);
					else 
					{					
						vector3<double> pos, vel;
						pos.x = Uwb.pos_x * 0.1;	pos.y = Uwb.pos_y * 0.1;	pos.z = Uwb.pos_z * 0.1;
						vel.x = Uwb.vel_x * 0.01;	vel.y = Uwb.vel_y * 0.01;	vel.z = Uwb.vel_z * 0.01;
						if( Uwb.eop[2] > 200 )
							PositionSensorChangeDataType( default_optical_flow_index, Position_Sensor_DataType_s_xy );
						else
							PositionSensorChangeDataType( default_optical_flow_index, Position_Sensor_DataType_s_xyz );
						double eop_xy = sqrtf( Uwb.eop[0]*Uwb.eop[0] + Uwb.eop[1]*Uwb.eop[1] );
						PositionSensorUpdatePosition( default_optical_flow_index, pos, true, -1, eop_xy, Uwb.eop[2] );
					}
				}
				rc_counter = 0;
				sum = 0;
			}
		}
	}
}

void init_drv_OpticalFlow()
{
	//波特率19200
	SetBaudRate_Uart5( 115200, 2, 2 );
	//注册传感器
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_s_xy , \
													Position_Sensor_frame_ENU , \
													0.1, 100 );
	xTaskCreate( OpticalFlow_Server, "OpticalFlow", 1024, NULL, SysPriority_ExtSensor, NULL);
}
#include "drv_GPS.hpp"
#include "drv_Uart8.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"

struct GpsConfig
{
	/*搜星GNSS设置
		0:不变
		63:GPS+SBAS+Galileo+BeiDou+IMES+QZSS
		119:GPS+SBAS+Galileo+IMES+QZSS+GLONASS
	*/
	uint8_t GNSS_Mode[8];
	
	//延时时间
	float delay[2];
};

static const uint8_t ubloxInit[] = {
		0xb5, 0x62, 0x06, 0x24, 0x24, 0x00,  0x05, 0x00,  0x07, 0x03,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0x5d, 0xc9,	//NAV5 2g
		//0xb5, 0x62,  0x06, 0x24, 0x24, 0x00,  0x05, 0x00,  0x08, 0x03,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0x5e, 0xeb,	//NAV5 4g
		//0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x07,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4E,0xFD,
		0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,            // set rate to 10Hz
	
		0xB5,0x62,0x06,0x01,0xF0,0x0A,0x00,0x04,0x23,
		0xB5,0x62,0x06,0x01,0xF0,0x09,0x00,0x03,0x21,
		0xB5,0x62,0x06,0x01,0xF0,0x00,0x00,0xFA,0x0F,
		0xB5,0x62,0x06,0x01,0xF0,0x01,0x00,0xFB,0x11,
		0xB5,0x62,0x06,0x01,0xF0,0x0D,0x00,0x07,0x29,
		0xB5,0x62,0x06,0x01,0xF0,0x06,0x00,0x00,0x1B,
		0xB5,0x62,0x06,0x01,0xF0,0x02,0x00,0xFC,0x13,
		0xB5,0x62,0x06,0x01,0xF0,0x07,0x00,0x01,0x1D,
		0xB5,0x62,0x06,0x01,0xF0,0x03,0x00,0xFD,0x15,
		0xB5,0x62,0x06,0x01,0xF0,0x04,0x00,0xFE,0x17,
		0xB5,0x62,0x06,0x01,0xF0,0x0E,0x00,0x08,0x2B,
		0xB5,0x62,0x06,0x01,0xF0,0x0F,0x00,0x09,0x2D,
		0xB5,0x62,0x06,0x01,0xF0,0x05,0x00,0xFF,0x19,
		0xB5,0x62,0x06,0x01,0xF0,0x08,0x00,0x02,0x1F,//7
		0xB5,0x62,0x06,0x01,0xF1,0x00,0x00,0xFB,0x12,
		0xB5,0x62,0x06,0x01,0xF1,0x01,0x00,0xFC,0x14,
		0xB5,0x62,0x06,0x01,0xF1,0x03,0x00,0xFE,0x18,
		0xB5,0x62,0x06,0x01,0xF1,0x04,0x00,0xFF,0x1A,
		0xB5,0x62,0x06,0x01,0xF1,0x05,0x00,0x00,0x1C,
		0xB5,0x62,0x06,0x01,0xF1,0x06,0x00,0x01,0x1E,
		
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x30 ,0x00 ,0x45 ,0xC0,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x50 ,0x00 ,0x65 ,0x00,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x33 ,0x00 ,0x48 ,0xC6,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x31 ,0x00 ,0x46 ,0xC2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x00 ,0x00 ,0x15 ,0x60,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x10 ,0x00 ,0x2A ,0x8F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0E ,0x00 ,0x39 ,0xBE,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x08 ,0x00 ,0x33 ,0xB2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x09 ,0x00 ,0x34 ,0xB4,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0B ,0x00 ,0x36 ,0xB8,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0F ,0x00 ,0x3A ,0xC0,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0D ,0x00 ,0x38 ,0xBC,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x05 ,0x00 ,0x19 ,0x67,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x09 ,0x00 ,0x1D ,0x6F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x0B ,0x00 ,0x1F ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x02 ,0x00 ,0x16 ,0x61,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x06 ,0x00 ,0x1A ,0x69,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x07 ,0x00 ,0x1B ,0x6B,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x21 ,0x00 ,0x35 ,0x9F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x2E ,0x00 ,0x42 ,0xB9,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x08 ,0x00 ,0x1C ,0x6D,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x60 ,0x00 ,0x6B ,0x02,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x22 ,0x00 ,0x2D ,0x86,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x31 ,0x00 ,0x3C ,0xA4,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x04 ,0x00 ,0x0F ,0x4A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x40 ,0x00 ,0x4B ,0xC2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x61 ,0x00 ,0x6C ,0x04,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x09 ,0x00 ,0x14 ,0x54,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x34 ,0x00 ,0x3F ,0xAA,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x01 ,0x00 ,0x0C ,0x44,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x02 ,0x00 ,0x0D ,0x46,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x07 ,0x00 ,0x12 ,0x50,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x35 ,0x00 ,0x40 ,0xAC,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x32 ,0x00 ,0x3D ,0xA6,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x06 ,0x00 ,0x11 ,0x4E,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x03 ,0x00 ,0x0E ,0x48,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x30 ,0x00 ,0x3B ,0xA2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x24 ,0x00 ,0x2F ,0x8A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x23 ,0x00 ,0x2E ,0x88,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x20 ,0x00 ,0x2B ,0x82,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x21 ,0x00 ,0x2C ,0x84,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x11 ,0x00 ,0x1C ,0x64,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x12 ,0x00 ,0x1D ,0x66,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x30 ,0x00 ,0x3C ,0xA5,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x31 ,0x00 ,0x3D ,0xA7,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x10 ,0x00 ,0x1C ,0x65,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x15 ,0x00 ,0x21 ,0x6F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x11 ,0x00 ,0x1D ,0x67,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x13 ,0x00 ,0x1F ,0x6B,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x20 ,0x00 ,0x2C ,0x85,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x11 ,0x00 ,0x28 ,0x88,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x16 ,0x00 ,0x2D ,0x92,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x13 ,0x00 ,0x2A ,0x8C,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x04 ,0x00 ,0x1B ,0x6E,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x03 ,0x00 ,0x1A ,0x6C,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x12 ,0x00 ,0x29 ,0x8A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x01 ,0x00 ,0x18 ,0x68,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x06 ,0x00 ,0x1D ,0x72,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x27, 0x01, 0x00 ,0x32 ,0xb6,	
		
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate		
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51,           // set PVT MSG rate
		
//		//set baudrate to 115200(in RTCM)
//		0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72 ,			
		//set baudrate to 115200(in RTCM3)
		0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x23, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xda, 0x52 ,			
};

#define write_ubx(buf,i,x, cka,ckb) {buf[i++]=x; cka+=x; ckb+=cka;}

static uint8_t GNSS_cfg_header[] = { 0x06,0x3e, 60,0 ,0x00,0x00,0x20, 7  };
#define GNSS_cfg_header_length_pos 2
#define GNSS_cfg_header_N_pos 7
static const uint8_t GNSS_cfgs[][8] =
{
	{ 0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01 },
	{ 0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01 },
	{ 0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01 },
	{ 0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01 },
	{ 0x04,0x00,0x08,0x00,0x01,0x00,0x01,0x01 },
	{ 0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01 },
	{ 0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01 },
};

enum GPS_Scan_Operation
{
	//在指定波特率下发送初始化信息
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	
	//检查是否设置成功
	GPS_Check_Baud ,
	//在当前波特率下再次发送配置
	GPS_ResendConfig ,
	//GPS已检测存在
	GPS_Present ,
};

struct GPS_State_Machine
{
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	//0=
	uint8_t CK_A , CK_B;	//checksum
};
static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}
static inline uint16_t GPS_ParseByte( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//找包头
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != 0xb5 )
					state_machine->frame_datas_ind = 0;
			}
			else
			{
				if( r_data == 0x62 )	//header found
				{
					state_machine->read_state = 1;
					state_machine->frame_datas_ind = 0;
					state_machine->CK_A = state_machine->CK_B = 0;	//reset checksum
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//读Class ID和包长度
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			if( state_machine->frame_datas_ind == 4 )
			{
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[2]) + 4;
				if( state_machine->frame_datas_length > 4 && state_machine->frame_datas_length < 150 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 2:	//读包内容
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length )
			{
				//payload read completed
				state_machine->read_state = 3;
			}
			break;
		}
		case 3://校验
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length + 1 )
			{
				if( r_data != state_machine->CK_A )
					ResetRxStateMachine(state_machine);
			}
			else
			{
				ResetRxStateMachine(state_machine);
				if( r_data == state_machine->CK_B )
					return state_machine->frame_datas_length;				
			}
		}
	}
	return 0;
}

uint8_t gps0_sat = 0;
uint8_t gps0_fix = 0;
uint16_t gps0_hacc = 10000;	uint16_t gps0_vacc = 10000;
int32_t gps0_lat = 0;	int32_t gps0_lon = 0;	int32_t gps0_alt = 0;
static void GPS_Server(void* pvParameters)
{
//	//注册传感器
//		PositionSensorRegister( default_gps_sensor_index , \
//														Position_Sensor_Type_GlobalPositioning , \
//														Position_Sensor_DataType_sv_z , \
//														Position_Sensor_frame_ENU , \
//														0.2 , //延时
//														0 , //xy信任度
//														300 //z信任度
//													);
//	while(1)
//	{
//		os_delay(0.2);
//		vector3<double> pos, vel;
//		PositionSensorUpdatePositionGlobalVel( default_gps_sensor_index, pos, vel, true );
//	}
	
	
	
	//GPS识别状态
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud9600;
	//数据读取状态机
	__attribute__((aligned(4))) uint8_t frame_datas[150];
	//上次更新时间
	TIME last_update_time;
	
	//注册Rtk端口
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = Write_Uart8;
	rtk_port.lock = Lock_Uart8;
	rtk_port.unlock = Unlock_Uart8;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);
	
GPS_CheckBaud:
	while(1)
	{
		//更改指定波特率
		SetBaudRate_Uart8( current_GPS_Operation, 3, 0.1 );
		//发送配置
		Write_Uart8( ubloxInit, sizeof(ubloxInit), 3, 0.2 );
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
				current_GPS_Operation = GPS_Scan_Baud460800;
				break;
			case GPS_Scan_Baud460800:
				current_GPS_Operation = GPS_Scan_Baud115200;
				break;
			default:
			case GPS_Scan_Baud115200:
				current_GPS_Operation = GPS_Scan_Baud9600;
				break;
		}
		
		//更改波特率
		SetBaudRate_Uart8( 115200, 3, 0.1 );
		//清空接收缓冲区准备接收数据
		ResetRx_Uart8(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		while( RxChkStartTime.get_pass_time() < 2 )
		{
			uint8_t r_data;
			if( Read_Uart8( &r_data, 1, 0.5, 0.1 ) )
			{
				uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
				if( pack_length )
				{
					if( frame_datas[0]==0x01 && frame_datas[1]==0x07 )
					{	//已识别到PVT包
						//跳转到gps接收程序
						goto GPS_Present;
					}
				}
			}
		}
	}
	
GPS_Present:
	//识别到gps
	//再次发送配置
	Write_Uart8( ubloxInit, sizeof(ubloxInit), 3, 0.2 );
	os_delay(3.0);
	
	GpsConfig gps_cfg;
	if( ReadParamGroup( "GPS0Cfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{
		gps_cfg.GNSS_Mode[0] &= 0b1111111;
		if( gps_cfg.GNSS_Mode[0] != 0 )
		{	//发送GNSS配置
			
			//发送包头数据
			uint8_t buf[100];
			uint8_t ind = 2;	uint8_t cka = 0, ckb = 0;
			buf[0] = 0xb5;	buf[1] = 0x62;
			for( uint8_t i = 0; i < sizeof(GNSS_cfg_header); ++i )
				write_ubx( buf, ind, GNSS_cfg_header[i] , cka, ckb )
			
			//发送配置内容
			for( uint8_t i = 0; i < 7; ++i )
			{
				for( uint8_t k = 0; k < 8; ++k )
				{
					if( k != 4 )
						write_ubx( buf, ind, GNSS_cfgs[i][k] , cka, ckb )
					else							
					{
						if( gps_cfg.GNSS_Mode[0] & (1<<i) )
							write_ubx( buf, ind, 1 , cka, ckb )
						else
							write_ubx( buf, ind, 0 , cka, ckb )
					}
					
				}
			}
			
			//校验
			buf[ind++] = cka;	buf[ind++] = ckb;
			//发送
			Write_Uart8( buf, ind, 0.1, 0.1 );
		}
		
		//注册传感器
		PositionSensorRegister( default_gps_sensor_index , \
														Position_Sensor_Type_GlobalPositioning , \
														Position_Sensor_DataType_sv_xy , \
														Position_Sensor_frame_ENU , \
														gps_cfg.delay[0] , //延时
														30 , //xy信任度
														30 //z信任度
													);
	}
	else
	{	//注册传感器
		PositionSensorRegister( default_gps_sensor_index , \
														Position_Sensor_Type_GlobalPositioning , \
														Position_Sensor_DataType_sv_xy , \
														Position_Sensor_frame_ENU , \
														0.1 , //延时
														30 , //xy信任度
														30 //z信任度
													);
	}
	
	//开启Rtk注入
	RtkPort_setEna( rtk_port_ind, true );
	
	//gps状态
	bool gps_available = false;
	bool z_available = false;
	TIME GPS_stable_start_time(false);
	double gps_lat;
	TIME gps_update_TIME;
	
	//清空接收缓冲区准备接收数据
	ResetRx_Uart8(0.1);
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
	last_update_time = TIME::now();
	while(1)
	{
		uint8_t r_data;
		if( Read_Uart8( &r_data, 1, 2, 0.1 ) )
		{
			uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
			if( pack_length )
			{
				if( frame_datas[0]==0x01 && frame_datas[1]==0x07 )
				{
					last_update_time = TIME::now();
					struct UBX_NAV_PVT_Pack
					{
						uint8_t CLASS;
						uint8_t ID;
						uint16_t length;
						
						uint32_t iTOW;
						uint16_t y;
						uint8_t month;
						uint8_t day;
						uint8_t hour;
						uint8_t min;
						uint8_t sec;
						uint8_t valid;
						uint32_t t_Acc;
						int32_t nano;
						uint8_t fix_type;
						uint8_t flags;
						uint8_t flags2;
						uint8_t numSV;
						int32_t lon;
						int32_t lat;
						int32_t height;
						int32_t hMSL;
						uint32_t hAcc;
						uint32_t vAcc;
						int32_t velN;
						int32_t velE;
						int32_t velD;
						int32_t gSpeed;
						int32_t headMot;
						uint32_t sAcc;
						uint32_t headAcc;
						uint16_t pDOP;
						uint8_t res1[6];
						int32_t headVeh;
						uint8_t res2[4];
					}__attribute__((packed));
					UBX_NAV_PVT_Pack* pack = (UBX_NAV_PVT_Pack*)frame_datas;
					
//					vector3<double> velocity;						
//						vector3<double> position_Global;
//						PositionSensorChangeDataType( default_gps_sensor_index, Position_Sensor_DataType_sv_xy );	
//						PositionSensorUpdatePositionGlobalVel( default_gps_sensor_index, position_Global , velocity , true , 0.15f );
//					continue;
					gps0_sat = pack->numSV;
					if( pack->fix_type == 0 )
						gps0_fix= 1;
					else
					{
						if( (pack->flags & 3) == 3 )
						{
							switch( pack->flags >> 6 )
							{
								case 1:
									gps0_fix= 5;
									break;
								case 2:
									gps0_fix= 6;
									break;
								default:
									gps0_fix = pack->fix_type;
							}
						}
						else
							gps0_fix = pack->fix_type;
					}
					gps0_hacc = pack->hAcc*0.1f;
					gps0_vacc = pack->vAcc*0.1f;
					gps0_lat = pack->lat;	gps0_lon = pack->lon;	gps0_alt = pack->hMSL;
					if( ((pack->flags & 1) == 1) && (pack->fix_type == 0x03) && (pack->numSV >= 5) )
					{
						if( gps_available == false )
						{
							if( pack->hAcc < 2500 )
							{
								if( GPS_stable_start_time.is_valid() == false )
									GPS_stable_start_time = TIME::now();
								else if( GPS_stable_start_time.get_pass_time() > 3.0f )
								{
									gps_available = true;
									GPS_stable_start_time.set_invalid();
								}
							}
							else
								GPS_stable_start_time.set_invalid();
						}
						else
						{
							if( pack->hAcc > 3500 )
								gps_available = z_available = false;
						}
						
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}
					
					if( gps_available )
					{
						if( z_available )
						{
							if( pack->vAcc > 4500 )
								z_available = false;
						}
						else
						{
							if( pack->vAcc < 2500 )
							{
								gps_lat = pack->hMSL * 1e-1;
								z_available = true;
							}
						}
						double t = gps_update_TIME.get_pass_time_st();
						if( t > 1 )
							t = 1;						
						
						vector3<double> velocity;
						velocity.y = pack->velN * 0.1;	//North
						velocity.x = pack->velE * 0.1;	//East
						velocity.z = -pack->velD * 0.1;	//Up
						gps_lat += velocity.z * t;
						
						vector3<double> position_Global;
						position_Global.x = pack->lat * 1e-7;
						position_Global.y = pack->lon * 1e-7;
						position_Global.z = gps_lat;
//										if( gps->available == false && available == true )
//											position_Global.z = pack->hMSL * 1e-1;
//										else
//											position_Global.z = gps->position.z + get_pass_time( gps->last_update_time )*velocity.z;
						
						if( z_available )
							PositionSensorChangeDataType( default_gps_sensor_index, Position_Sensor_DataType_sv_xyz );
						else
							PositionSensorChangeDataType( default_gps_sensor_index, Position_Sensor_DataType_sv_xy );
						
						//地速越高gps信任度越高
						double xy_trustD = pack->hAcc * 0.1;
						double z_trustD = pack->vAcc * 0.1;
						PositionSensorUpdatePositionGlobalVel( default_gps_sensor_index, position_Global, velocity, true, -1, xy_trustD, z_trustD );
													
					}
					else
						PositionSensorSetInavailable( default_gps_sensor_index );
				}
			}
			
			if( last_update_time.get_pass_time() > 2 )
			{	//接收不到数据
				PositionSensorUnRegister( default_gps_sensor_index );
				//关闭Rtk注入
				RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}
		}
		else
		{	//接收不到数据
			PositionSensorUnRegister( default_gps_sensor_index );
			//关闭Rtk注入
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}
	}
}

void init_drv_GPS()
{
	//注册GPS参数	
	GpsConfig initial_cfg;
	initial_cfg.GNSS_Mode[0] = 0;
	initial_cfg.delay[0] = 0.1;
	
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_REAL32 ,
	};
	SName param_names[] = {
		"GPS0_GNSS" ,
		"GPS0_delay" ,
	};
	ParamGroupRegister( "GPS0Cfg", 1,2, param_types, param_names, (uint64_t*)&initial_cfg );
	
	xTaskCreate( GPS_Server, "GPS", 1024, NULL, SysPriority_ExtSensor, NULL);
}
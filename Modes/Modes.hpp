#pragma once

#include "Basic.hpp"
#include "mavlink.h"
#include "Receiver.hpp"

#define CMD_TYPE_MAVLINK (1<<4)
#define CMD_TYPE_MASK 0xf0
#define CMD_TYPE_PORT_MASK 0x0f
struct ModeMsg
{
	//高4位：1-mavlink消息
	//低4位：接收port
	uint8_t cmd_type;
	uint8_t sd_sysid;
	uint8_t sd_compid;
	uint8_t frame;
	uint32_t cmd;
	double params[8];
};
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT );

//模式功能选项
struct ModeFuncCfg
{
	//PA-按钮1解锁前功能（模式序号）
	uint8_t Bt1PAFunc1[8];
	uint8_t Bt1PAFunc2[8];
	uint8_t Bt1PAFunc3[8];
	uint8_t Bt1PAFunc4[8];
	uint8_t Bt1PAFunc5[8];
	uint8_t Bt1PAFunc6[8];
	
	//A-按钮1解锁后功能（1-定高模式 2-位置模式 3-运动模式 22-任务模式 23-返航模式）
	uint8_t Bt1AFunc1[8];
	uint8_t Bt1AFunc2[8];
	uint8_t Bt1AFunc3[8];
	uint8_t Bt1AFunc4[8];
	uint8_t Bt1AFunc5[8];
	uint8_t Bt1AFunc6[8];
	
	//任务执行按钮
	//2-4：对应按钮按下(100%)执行任务
	//12-14：对应按钮变化执行任务
	uint8_t MissionBt[8];
	
	//返航按钮
	//2-4：对应按钮按下(100%)返航
	//12-14：对应按钮变化返航
	uint8_t RTLBt[8];
	
	//安全按钮
	//0：无
	//2-4：对应按钮按下(100%)强制锁定电机
	//10：油门最下偏航最做强制锁定电机
	uint8_t SafeBt[8];
	
	//中位死区
	float NeutralZone[2];
	//位置速度响应曲线系数
	float PosVelAlpha[2];
	//姿态响应曲线系数
	float AttAlpha[2];
	
	//解锁使用0号航点
	uint8_t RstWp0[8];
}__PACKED;
#define is_AFunc_auto(x) (x>=20)
#define AFunc_AltHold 1
#define AFunc_PosHold 2
#define AFunc_Mission 22
#define AFunc_RTL 23
	
//获取遥控按钮对应的区域（0-5）
inline int8_t get_RcButtonZone( double rc, uint8_t current_zone )
{
	double st = 1.0/6*100;
	if( current_zone<=5 && rc>=current_zone*st-1 && rc<(current_zone+1)*st+1 )
		return current_zone;
	
	if( rc < 1*st )
		return 0;
	else if( rc < 2*st )
		return 1;
	else if( rc < 3*st )
		return 2;
	else if( rc < 4*st )
		return 3;
	else if( rc < 5*st )
		return 4;
	else
		return 5;
}

class Mode_Base;
void ModeRegister( Mode_Base* mode, uint8_t id );

enum ModeResult
{
	MR_OK = 0 ,
	MR_Err ,
};

class Mode_Base
{
	private:
		
	public:
		SName name;
		Mode_Base( SName name, uint8_t mode_id )
		{
			this->name = name;
			ModeRegister( this, mode_id );
		}		
		//模式主函数
		virtual ModeResult main_func( void* param1, uint32_t param2 ) = 0;
		//获取模式Mavlink模式标号
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, uint16_t* mav_mode, uint16_t* mav_main_mode, uint16_t* mav_sub_mode )
		{
			*mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
			*mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
			*mav_sub_mode = 0;
		}
};

//当前航点记录（用于断点续飞）
struct CurrentWpInf
{
	//当前航点序号
	uint32_t CurrentWp[2];
	//航线向量AB A-目标 B起点
	double line_x;
	double line_y;
	double line_z;
	//航线完成距离
	double line_fs;
}__PACKED;

void init_Modes();
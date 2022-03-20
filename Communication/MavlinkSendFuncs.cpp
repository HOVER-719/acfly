#include "MavlinkSendFuncs.hpp"
#include "mavlink.h"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "AC_Math.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "Receiver.hpp"
#include "Missions.hpp"

static bool Msg01_SYS_STATUS( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	float volt, current, RMPercent;
	get_MainBatteryInf( &volt, &current, 0, 0, &RMPercent );
	mavlink_msg_sys_status_pack_chan(
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		0 ,
		0 ,
		0 ,
		getCPULoad() * 10 ,
		volt*1000 ,
		current*100 ,
		RMPercent ,
		0 ,
		0 ,
		0 ,
		0 ,
		0 ,
		0
	);
	return true;
}

static bool Msg26_SCALED_IMU( uint8_t port , mavlink_message_t* msg_sd )
{
	IMU_Sensor sensor;
	uint8_t s_count = 0;
	vector3<float> acc, gyro, mag;
	if( GetAccelerometer( 0, &sensor ) )
	{
		++s_count;
		acc.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetGyroscope( 0, &sensor ) )
	{
		++s_count;
		gyro.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetMagnetometer( 0, &sensor ) )
	{
		++s_count;
		mag.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( s_count == 0 )
		return false;
	
	mavlink_msg_scaled_imu_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1e3 , 	//boot ms
		acc.x*(1000/GravityAcc) ,
		acc.y*(1000/GravityAcc) ,
		acc.z*(1000/GravityAcc) ,
		gyro.x*1000 ,
		gyro.y*1000 ,
		gyro.z*1000 ,
		mag.x*1000 ,
		mag.y*1000 ,
		mag.z*1000 );	
	return true;
}
static bool Msg29_SCALED_PRESSURE( uint8_t port , mavlink_message_t* msg_sd )
{
  extern float pressure_internal;
  extern float pressure_internal_temperature;	
	mavlink_msg_scaled_pressure_pack_chan(
			get_CommulinkSysId(),	//system id
			get_CommulinkCompId(),//component id
			port, //chan
			msg_sd,
			TIME::get_System_Run_Time()*1e3, //boot ms		
		  pressure_internal*1e-3,
		  0,
	    pressure_internal_temperature);
	return true;
}
static bool Msg116_SCALED_IMU2( uint8_t port , mavlink_message_t* msg_sd )
{
	IMU_Sensor sensor;
	uint8_t s_count = 0;
	vector3<float> acc, gyro, mag;
	if( GetAccelerometer( 1, &sensor ) )
	{
		++s_count;
		acc.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetGyroscope( 1, &sensor ) )
	{
		++s_count;
		gyro.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetMagnetometer( 1, &sensor ) )
	{
		++s_count;
		mag.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( s_count == 0 )
		return false;
	
	mavlink_msg_scaled_imu2_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1e6 , 	//time usec
		acc.x*(1000/GravityAcc) ,
		acc.y*(1000/GravityAcc) ,
		acc.z*(1000/GravityAcc) ,
		gyro.x*1000 ,
		gyro.y*1000 ,
		gyro.z*1000 ,
		mag.x*1000 ,
		mag.y*1000 ,
		mag.z*1000 );	
	return true;
}
static bool Msg129_SCALED_IMU3( uint8_t port , mavlink_message_t* msg_sd )
{
	IMU_Sensor sensor;
	uint8_t s_count = 0;
	vector3<float> acc, gyro, mag;
	if( GetAccelerometer( 2, &sensor ) )
	{
		++s_count;
		acc.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetGyroscope( 2, &sensor ) )
	{
		++s_count;
		gyro.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( GetMagnetometer( 2, &sensor ) )
	{
		++s_count;
		mag.set_vector(
			sensor.data_raw.x*sensor.sensitivity,
			sensor.data_raw.y*sensor.sensitivity,
			sensor.data_raw.z*sensor.sensitivity);
	}
	if( s_count == 0 )
		return false;
	
	mavlink_msg_scaled_imu3_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1e6 , 	//time usec
		acc.x*(1000/GravityAcc) ,
		acc.y*(1000/GravityAcc) ,
		acc.z*(1000/GravityAcc) ,
		gyro.x*1000 ,
		gyro.y*1000 ,
		gyro.z*1000 ,
		mag.x*1000 ,
		mag.y*1000 ,
		mag.z*1000 );	
	return true;
}

static bool Msg30_ATTITUDE( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	vector3<double> angular_rate;
	get_AngularRate_Ctrl(&angular_rate);
	
	double heading = airframe_quat.getYaw();
	if( heading < 0 )
		heading = 2*Pi + heading;
	mavlink_msg_attitude_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		airframe_quat.getRoll() ,
		airframe_quat.getPitch() ,
		heading ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

static bool Msg31_ATTITUDE_QUATERNION( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	vector3<double> angular_rate;
	get_AngularRate_Ctrl(&angular_rate);
	
	mavlink_msg_attitude_quaternion_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		airframe_quat.get_qw() ,
		airframe_quat.get_qx() ,
		airframe_quat.get_qy() ,
		airframe_quat.get_qz() ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

static bool Msg24_GPS_RAW_INT( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_MSStatus() != MS_Ready )
		return false;
		
	Position_Sensor gps_sensor;
	if( GetPositionSensor( default_gps_sensor_index, &gps_sensor ) )
	{
		extern uint8_t gps0_sat;
		extern uint8_t gps0_fix;
		extern uint16_t gps0_hacc;	extern uint16_t gps0_vacc;
		extern int32_t gps0_lat;	extern int32_t gps0_lon;	extern int32_t gps0_alt;
		mavlink_msg_gps_raw_int_pack_chan( 
			get_CommulinkSysId() ,	//system id
			get_CommulinkCompId() ,	//component id
			port	,	//chan
			msg_sd,
			TIME::get_System_Run_Time() * 1e6 ,	//usec
			(gps0_fix>1)?gps0_fix:1 ,	//fixtype
			gps0_lat  ,	//lat
			gps0_lon  ,	//lon
			gps0_alt ,	//alt
			0 ,	//eph
			0 ,	//epv
			0 ,	//vel
			0 ,	//cog
			gps0_sat ,	//satellites_visible
			0 ,	//alt_ellipsoid
			gps0_hacc ,	//h_acc
			gps0_vacc ,	//v_acc
			0 ,	//vel_acc
			0  //hdg_acc
		);		
		return true;
	}
	else
	{
		mavlink_msg_gps_raw_int_pack_chan( 
			get_CommulinkSysId() ,	//system id
			get_CommulinkCompId() ,	//component id
			port	,	//chan
			msg_sd,
			TIME::get_System_Run_Time() * 1e6 ,	//usec
			0 ,	//fixtype
			0  ,	//lat
			0  ,	//lon
			0 ,	//alt
			0 ,	//eph
			0 ,	//epv
			0 ,	//vel
			0 ,	//cog
			0 ,	//satellites_visible
			0 ,	//alt_ellipsoid
			0 ,	//h_acc
			0 ,	//v_acc
			0 ,	//vel_acc
			0  //hdg_acc
		);		
		return true;
	}
	return false;
}

static bool Msg33_GLOBAL_POSITION_INT( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	double lat = 0, lon = 0, alt = 0;
	
	PosSensorHealthInf2 global_posInf;
	if( get_OptimalGlobal_XY(&global_posInf) )
	{		
		map_projection_reproject( &global_posInf.mp, 
				global_posInf.PositionENU.x+global_posInf.HOffset.x, 
				global_posInf.PositionENU.y+global_posInf.HOffset.y,
				&lat, &lon );
	}
	
	PosSensorHealthInf1 z_posInf;
	if( get_OptimalGlobal_Z(&z_posInf) )
		alt = z_posInf.PositionENU.z + z_posInf.HOffset;
	
	double homeZ;
	if( getHomeLocalZ( &homeZ, 0.01 ) )
	{
		double heightAboveGround = z_posInf.PositionENU.z - homeZ;
		
		vector3<double> vel;
		get_VelocityENU_Ctrl(&vel);
		
		mavlink_msg_global_position_int_pack_chan( 
			get_CommulinkSysId() ,	//system id
			get_CommulinkCompId() ,	//component id
			port	,	//chan
			msg_sd,
			TIME::get_System_Run_Time() * 1e3 ,	//usec
			lat*1e7  ,	//lat
			lon*1e7  ,	//lon
			alt*10 ,	//alt
			heightAboveGround*10 ,	//Altitude above ground
			vel.y ,	//vel north
			vel.x ,	//vel east
			-vel.z ,	//vel down
			0xffff	//Vehicle heading
		);
		
		return true;
	}
	return false;
}

static bool Msg32_LOCAL_POSITION_NED( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_MSStatus() != MS_Ready )
		return false;
	
	vector3<double> Position;
	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;
	get_VelocityENU_Ctrl(&VelocityENU);
	
	mavlink_msg_local_position_ned_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		Position.y * 0.01f ,
		Position.x * 0.01f ,
		Position.z * -0.01f ,
		VelocityENU.y * 0.01f , 
		VelocityENU.x * 0.01f ,
		VelocityENU.z * -0.01f);
	return true;
}

static bool Msg34_RC_CHANNELS_SCALED( uint8_t port , mavlink_message_t* msg_sd )
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		if( rc.available )
		{
			mavlink_msg_rc_channels_scaled_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port , 	//chan
				msg_sd ,
				TIME::get_System_Run_Time() * 1000 , 	//boot ms
				0 ,		//port
				rc.available_channels>0 ? rc.data[0]*200 - 10000 : INT16_MAX ,
				rc.available_channels>1 ? rc.data[1]*200 - 10000 : INT16_MAX ,
				rc.available_channels>2 ? rc.data[2]*200 - 10000 : INT16_MAX ,
				rc.available_channels>3 ? rc.data[3]*200 - 10000 : INT16_MAX ,
				rc.available_channels>4 ? rc.data[4]*200 - 10000 : INT16_MAX ,
				rc.available_channels>5 ? rc.data[5]*200 - 10000 : INT16_MAX ,
				rc.available_channels>6 ? rc.data[6]*200 - 10000 : INT16_MAX ,
				rc.available_channels>7 ? rc.data[7]*200 - 10000 : INT16_MAX ,
				255	//rssi
				);
			return true;
		}
	}
	mavlink_msg_rc_channels_scaled_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		0 ,		//port
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		INT16_MAX ,
		255	//rssi
		);
	return true;
}

static bool Msg65_RC_CHANNELS( uint8_t port , mavlink_message_t* msg_sd )
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		if( rc.connected )
		{
			mavlink_msg_rc_channels_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port , 	//chan
				msg_sd ,
				TIME::get_System_Run_Time() * 1000 , 	//boot ms
				rc.raw_available_channels ,		//chan count
				rc.raw_available_channels > 0 ? rc.raw_data[0]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 1 ? rc.raw_data[1]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 2 ? rc.raw_data[2]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 3 ? rc.raw_data[3]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 4 ? rc.raw_data[4]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 5 ? rc.raw_data[5]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 6 ? rc.raw_data[6]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 7 ? rc.raw_data[7]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 8 ? rc.raw_data[8]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 9 ? rc.raw_data[9]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 10 ? rc.raw_data[10]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 11 ? rc.raw_data[11]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 12 ? rc.raw_data[12]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 13 ? rc.raw_data[13]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 14 ? rc.raw_data[14]*10 + 1000 : UINT16_MAX ,
				rc.raw_available_channels > 15 ? rc.raw_data[15]*10 + 1000 : UINT16_MAX ,
				UINT16_MAX,
				UINT16_MAX,
				255	//rssi
				);
			return true;
		}
	}
	mavlink_msg_rc_channels_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		0 ,		//chan count
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		UINT16_MAX,
		255	//rssi
		);
	return true;
}

static bool Msg74_VFR_HUD( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_MSStatus() != MS_Ready )
		return false;
	
	vector3<double> Position;
	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;
	get_VelocityENU_Ctrl(&VelocityENU);
	
	double speed = safe_sqrt( VelocityENU.x*VelocityENU.x + VelocityENU.y*VelocityENU.y );
	double hover_throttle = 0;
	get_hover_throttle(&hover_throttle,0.01);
	
	double homeZ;
	double heightAboveGround = 0;
	if( getHomeLocalZ( &homeZ, 0.01 ) )
		heightAboveGround = Position.z - homeZ;
	
	mavlink_msg_vfr_hud_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		speed*0.01 , 	//airspeed
		speed*0.01 ,		//groundspeed
		rad2degree( atan2f( VelocityENU.y, VelocityENU.x ) ) ,		//heading
		hover_throttle ,		//throttle
		heightAboveGround * 0.01f , 	//alt
		VelocityENU.z * 0.01f		//climb
		);
	return true;
}

static bool Msg42_MISSION_CURRENT( uint8_t port , mavlink_message_t* msg_sd )
{
	mavlink_msg_mission_current_pack_chan(
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port ,	//chan
		msg_sd,
		getCurrentMissionInd()
	);
	return true;
}
static bool Msg62_NAV_CONTROLLER_OUTPUT( uint8_t port , mavlink_message_t* msg_sd )
{
	//获取目标roll pitch角度
	double rol, pit;
	Attitude_Control_get_Target_RollPitch( &rol, &pit );
	
	//获取目标偏航角
	double yaw;
	Attitude_Control_get_TargetYaw(&yaw);
	double bearing = 0;
	vector3<double> AB;
	if( Position_Control_get_LineFlightABDistance(&AB,0) )
		bearing = atan2( -AB.y, -AB.x );
		
	
	//计算航点距离
	MissionInf current_mission;
	uint16_t current_mission_ind = 0;
	float wp_distance = 0;
	if( ReadCurrentMission( &current_mission, &current_mission_ind ) )
	{
		if( current_mission.cmd == MAV_CMD_NAV_WAYPOINT )
		{
			switch(current_mission.frame)
			{
				case MAV_FRAME_GLOBAL:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT:
				case MAV_FRAME_GLOBAL_INT:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
				{	//全球定位				
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) )
					{
						//获取指定经纬度平面坐标
						double x, y;
						map_projection_project( &global_inf.mp, current_mission.params[4], current_mission.params[5], &x, &y );
						x -= global_inf.HOffset.x;
						y -= global_inf.HOffset.y;
						wp_distance = safe_sqrt( sq(x - global_inf.PositionENU.x) + sq(y - global_inf.PositionENU.y) );
					}				
					break;
				}
				
				case MAV_FRAME_LOCAL_NED:
				{
					vector3<double> position;
					get_Position_Ctrl(&position);
					wp_distance = safe_sqrt( sq(current_mission.params[5]*100 - position.x) + sq(current_mission.params[4]*100 - position.y) );
					break;
				}
				
				case MAV_FRAME_LOCAL_ENU:
				{
					vector3<double> position;
					get_Position_Ctrl(&position);
					wp_distance = safe_sqrt( sq(current_mission.params[4]*100 - position.x) + sq(current_mission.params[5]*100 - position.y) );
					break;
				}
				
				case MAV_FRAME_LOCAL_OFFSET_NED:
				case MAV_FRAME_BODY_NED:
				case MAV_FRAME_BODY_FRD:
				case MAV_FRAME_BODY_OFFSET_NED:
				case MAV_FRAME_BODY_FLU:
				{
					wp_distance = safe_sqrt( sq(current_mission.params[4]*100) + sq(current_mission.params[5]*100) );
					break;
				}
				
				default:
					wp_distance = 0;
			}
		}
	}
	
	mavlink_msg_nav_controller_output_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		rad2degree(rol) , 	//nav roll
		rad2degree(pit) ,		//nav pitch
		Mod( rad2degree(yaw), 360 ) ,	//nav yaw
		rad2degree(bearing) ,		//nav bearing
		wp_distance*0.01f ,		//wp_dist
		0 , 	//current alt error
		0,		//current airspeed error
		0	//current crosstrack error on x-y plane
		);
	return true;
}

static bool Msg234_HIGH_LATENCY( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	bool inFlight;
	get_is_inFlight(&inFlight);
	
	extern bool GCS_is_MP;
	uint16_t mav_mode, mav_main_mode, mav_sub_mode;
	get_mav_mode( &mav_mode, &mav_main_mode, &mav_sub_mode );
	px4_custom_mode custom_mode;
	custom_mode.main_mode = mav_main_mode;
	if( (mav_mode&MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) && GCS_is_MP && mav_main_mode==PX4_CUSTOM_MAIN_MODE_AUTO )
		custom_mode.main_mode = 0;
	custom_mode.sub_mode = mav_sub_mode;
	
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	vector3<double> angular_rate;
	get_AngularRate_Ctrl(&angular_rate);
	double heading = airframe_quat.getYaw();
	if( heading < 0 )
		heading = 2*Pi + heading;
	
	double hover_throttle = 0;
	get_hover_throttle(&hover_throttle,0.01);
	
	uint8_t gps_sat = 0;
	uint8_t gps_fix = 0;
	int32_t gps_lat = 0;	int32_t gps_lon = 0;	int32_t gps_alt = 0;
	Position_Sensor gps_sensor;
	if( GetPositionSensor( default_gps_sensor_index, &gps_sensor ) )
	{
		extern uint8_t gps0_sat;
		extern uint8_t gps0_fix;
		extern uint16_t gps0_hacc;	extern uint16_t gps0_vacc;
		extern int32_t gps0_lat;	extern int32_t gps0_lon;	extern int32_t gps0_alt;
		gps_sat = gps0_sat;
		gps_fix = (gps0_fix>1)?gps0_fix:1;
		gps_lat = gps0_lat;	gps_lon = gps0_lon;	gps_alt = gps0_alt;
	}
	
	vector3<double> Position;
	get_Position_Ctrl(&Position);
	double homeZ;
	double heightAboveGround = 0;
	if( getHomeLocalZ( &homeZ, 0.01 ) )
		heightAboveGround = Position.z - homeZ;
	
	vector3<double> vel;
	get_VelocityENU_Ctrl(&vel);
	double Gs = safe_sqrt( vel.x*vel.x + vel.y*vel.y );
	
	float volt, current, RMPercent;
	get_MainBatteryInf( &volt, &current, 0, 0, &RMPercent );
	
	MissionInf current_mission;
	uint16_t current_mission_ind = 0;
	float wp_distance = 0;
	if( ReadCurrentMission( &current_mission, &current_mission_ind ) )
	{
		if( current_mission.cmd == MAV_CMD_NAV_WAYPOINT )
		{
			switch(current_mission.frame)
			{
				case MAV_FRAME_GLOBAL:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT:
				case MAV_FRAME_GLOBAL_INT:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
				{	//全球定位				
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) )
					{
						//获取指定经纬度平面坐标
						double x, y;
						map_projection_project( &global_inf.mp, current_mission.params[4], current_mission.params[5], &x, &y );
						x -= global_inf.HOffset.x;
						y -= global_inf.HOffset.y;
						wp_distance = safe_sqrt( sq(x - global_inf.PositionENU.x) + sq(y - global_inf.PositionENU.y) );
					}				
					break;
				}
				
				case MAV_FRAME_LOCAL_NED:
				{
					vector3<double> position;
					get_Position_Ctrl(&position);
					wp_distance = safe_sqrt( sq(current_mission.params[5]*100 - position.x) + sq(current_mission.params[4]*100 - position.y) );
					break;
				}
				
				case MAV_FRAME_LOCAL_ENU:
				{
					vector3<double> position;
					get_Position_Ctrl(&position);
					wp_distance = safe_sqrt( sq(current_mission.params[4]*100 - position.x) + sq(current_mission.params[5]*100 - position.y) );
					break;
				}
				
				case MAV_FRAME_LOCAL_OFFSET_NED:
				case MAV_FRAME_BODY_NED:
				case MAV_FRAME_BODY_FRD:
				case MAV_FRAME_BODY_OFFSET_NED:
				case MAV_FRAME_BODY_FLU:
				{
					wp_distance = safe_sqrt( sq(current_mission.params[4]*100) + sq(current_mission.params[5]*100) );
					break;
				}
				
				default:
					wp_distance = 0;
			}
		}
	}
	
	mavlink_msg_high_latency_pack_chan( 
		get_CommulinkSysId() ,	//system id
		get_CommulinkCompId() ,	//component id
		port , 	//chan
		msg_sd ,
		mav_mode > 0 ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav_mode : 0 ,	//base mode
		custom_mode.data ,	//custom mode
		inFlight ? MAV_LANDED_STATE_IN_AIR : MAV_LANDED_STATE_ON_GROUND ,	//landed state
		rad2degree( airframe_quat.getRoll() )*100 ,	//roll
		rad2degree( airframe_quat.getPitch() )*100 , //pitch
		rad2degree( heading )*100 ,	//heading
		hover_throttle ,	//throttle
		rad2degree( angular_rate.z )*100 ,	//heading sp
		gps_lat ,	//lat
		gps_lon ,	//lon
		heightAboveGround*0.01 ,	//alt_amsl
		heightAboveGround*0.01 ,	//alt setpoint
		Gs*0.01 ,	//air speed
		0 ,	//air speed setpoint
		Gs*0.01 ,	//ground speed
		vel.z*0.01 ,	//climb rate
		gps_sat ,	//gps nsat
		gps_fix ,	//gps fixtype
		RMPercent ,	//battery remaining
		0 ,	//temperature
		0 ,	//temperature air
		0 ,	//failsafe
		current_mission_ind ,	//wp_num
		wp_distance*0.01 	//wp_distance
	);	
	
	return true;
}

static bool Msg242_HOME_POSITION( uint8_t Port_index , mavlink_message_t* msg )
{
  double Altitude_Local=0;
	Position_Sensor GPS;
	vector2<double> homePLatLon,homePoint;
	if( getHomeLatLon(&homePLatLon) && getHomeLocalZ(&Altitude_Local) && getHomePoint(&homePoint))
	{
		const Port* port = get_Port( Port_index );
		if( port->write )
		{
			mavlink_msg_home_position_pack_chan(
					get_CommulinkSysId() ,	//system id
					get_CommulinkCompId() ,	//component id
					Port_index ,	//chan
					msg,
					homePLatLon.x * 1e7 ,	//latitude
					homePLatLon.y * 1e7 ,	//longitude
					GPS.position.z * 10 ,	//altitude,mm
					homePoint.x * 0.01 , //Local X position
					homePoint.y * 0.01 , //Local Y position
				  Altitude_Local * 0.01 , //Local Z position
				  0 , //World to surface normal and heading transformation of the takeoff position
				  0 , //Local X position of the end of the approach vector
				  0 , //Local Y position of the end of the approach vector
				  0 , //Local Z position of the end of the approach vector
				  TIME::get_System_Run_Time()*1e6
				);

			return true;	
		}	 	
	}
	return false;
}



bool (*const Mavlink_Send_Funcs[])( uint8_t port , mavlink_message_t* msg_sd ) = 
{
	/*000-*/	0	,
	/*001-*/	Msg01_SYS_STATUS	,
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	0	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	0	,
	/*021-*/	0	,
	/*022-*/	0	,
	/*023-*/	0	,
	/*024-*/	Msg24_GPS_RAW_INT	,
	/*025-*/	0	,
	/*026-*/	Msg26_SCALED_IMU	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	Msg29_SCALED_PRESSURE	,
	/*030-*/	Msg30_ATTITUDE	,
	/*031-*/	Msg31_ATTITUDE_QUATERNION	,
	/*032-*/	Msg32_LOCAL_POSITION_NED	,
	/*033-*/	Msg33_GLOBAL_POSITION_INT	,
	/*034-*/	Msg34_RC_CHANNELS_SCALED	,
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	0	,
	/*040-*/	0	,
	/*041-*/	0	,
	/*042-*/	Msg42_MISSION_CURRENT	,
	/*043-*/	0	,
	/*044-*/	0	,
	/*045-*/	0	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	0	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	Msg62_NAV_CONTROLLER_OUTPUT	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	Msg65_RC_CHANNELS	,
	/*066-*/	0	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	0	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	0	,
	/*074-*/	Msg74_VFR_HUD	,
	/*075-*/	0	,
	/*076-*/	0	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	0	,
	/*085-*/	0	,
	/*086-*/	0	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	0	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	0	,
	/*103-*/	0	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	0	,
	/*111-*/	0	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	Msg116_SCALED_IMU2	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	Msg129_SCALED_IMU3	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	0	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	0	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	0	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	0	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	0	,
	/*206-*/	0	,
	/*207-*/	0	,
	/*208-*/	0	,
	/*209-*/	0	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	0	,
	/*234-*/	Msg234_HIGH_LATENCY	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	Msg242_HOME_POSITION	,
	/*243-*/	0	,
	/*244-*/	0	,
	/*245-*/	0	,
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
};
const uint16_t Mavlink_Send_Funcs_Count = sizeof( Mavlink_Send_Funcs ) / sizeof( void* );
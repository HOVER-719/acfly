#include "drv_GPS.hpp"

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

static void GPS_Server(void* pvParameters)
{
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
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
					
					#define write_new_byte(buffer,i,byte) buffer[i++]=byte;CK_A+=byte; CK_B+=CK_A
					uint8_t buf[30];
					buf[0] = 'A';	buf[1] = 'C';
					int buffer_index = 2;
					write_new_byte( buf, buffer_index, 201 );
					write_new_byte( buf, buffer_index, 0 );
					
					float temp = pack->lat*1e-7f;
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[0] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[1] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[2] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[3] );
					
					temp = pack->lon*1e-7f;
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[0] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[1] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[2] );
					write_new_byte( buf, buffer_index, ((uint8_t*)&temp)[3] );
					
					buf[ buffer_index++ ] = CK_A;
					buf[ buffer_index++ ] = CK_B;
					Uart2_Send( buf, buffer_index );
				}
			}
		}
	}
}

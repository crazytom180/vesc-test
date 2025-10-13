#include "RC_vesc.h"

namespace vesc
{
    void Vesc::Set_Rpm(float target_rpm_)
    {
        vesc_motor_mode = vesc_erpm;
        target_rpm = target_rpm_;// 计算ERPM
    }
    void Vesc::Set_Current(float target_c_)
	{
		vesc_motor_mode = vesc_current;
		target_current = target_c_;
		
	}
    Vesc::Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_), motor::Motor()
    {
        // 初始化id
        if (id_ <= 255 && id_ != 0) id = id_;
        else Error_Handler();
        
        // 登记can设备
        CanHandler_Register();
        
        // vesc默认pid参数
       pid_spd.Pid_Mode_Init(true, false, 0);
		pid_spd.Pid_Param_Init(10, 0.54, 0, 0, 0.001, 0, 10000, 10000, 5000, 5000, 5000);// 1ms
		
		pid_pos.Pid_Mode_Init(false, false, 0);
		pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 1000, 1000, 500, 500, 500);// 1ms
    }
    
    
    
    void Vesc::CanHandler_Register()
    {
        if (can->hd_num > 8) Error_Handler();// 设备数量超过8

        can_frame_type = can::FRAME_EXT;// 扩展帧

         switch (vesc_motor_mode)
         {
         case vesc_current:
            tx_id = (SET_CURRENT_CMD<<8) | id;
            break;
        
         case vesc_erpm:
            tx_id = (SET_ERPM_CMD<<8) | id;
            break;
         
         default:
            break;
         }
  
        // 设置接收帧id
        rx_id = (GET_VALUES_CMD<<8) | id;

        if (can->tx_frame_num == 0)// can上还没有挂载帧
        {
            tx_frame_dx = 0;
            can->tx_frame_num = 1;// 第一个帧
            
            can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
            can->tx_frame_list[tx_frame_dx].id = tx_id;
            can->tx_frame_list[tx_frame_dx].dlc = 8;
            
            can->tx_frame_list[tx_frame_dx].hd_num = 1;
            can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
        }
        else// can上已经有帧
        {
           
				can->tx_frame_num++;// 帧加一
				tx_frame_dx = can->tx_frame_num - 1;// 索引后移一位
				
				can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
		
				can->tx_frame_list[tx_frame_dx].hd_num = 1;
				can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
			
		}
	}
	

    void Vesc::Tim_It_Process()
    {
        float temp_target_rpm = 0;// 目标速度
        
        if (vesc_motor_mode == vesc_erpm)// 速度模式
        {
            temp_target_rpm = target_rpm;
			pid_spd.Update_Target(temp_target_rpm);
			pid_spd.Update_Real(rpm);
        }
       
        else if(vesc_motor_mode == vesc_current)
		{
			
		}
    }
    
    
    
    
    void Vesc::Can_Tx_Process()
    {
        switch (vesc_motor_mode)
        {
        case vesc_current:
			tx_id = (SET_CURRENT_CMD<<8) | id;
            send_current = (int32_t)(target_current);
            can->tx_frame_list[tx_frame_dx].data[0] = send_current >> 24 & 0xFF;// 高8位
            can->tx_frame_list[tx_frame_dx].data[1] = send_current >> 16 & 0xFF;
            can->tx_frame_list[tx_frame_dx].data[2] = send_current >> 8 & 0xFF;
            can->tx_frame_list[tx_frame_dx].data[3] = send_current & 0xFF;
			for (uint8_t i = 4; i < 8; i++) 
			{
				can->tx_frame_list[tx_frame_dx].data[i] = 0;
			}
            can->tx_frame_list[tx_frame_dx].dlc = 8; // 数据长度8字节 
            break;
        case vesc_erpm:
			tx_id = (SET_ERPM_CMD<<8) | id;
            send_rpm = (int32_t)(target_rpm);
            can->tx_frame_list[tx_frame_dx].data[0] = send_rpm >> 24 & 0xFF;// 高8位
            can->tx_frame_list[tx_frame_dx].data[1] = send_rpm >> 16 & 0xFF;
            can->tx_frame_list[tx_frame_dx].data[2] = send_rpm >> 8 & 0xFF;
            can->tx_frame_list[tx_frame_dx].data[3] = send_rpm & 0xFF;
			for (uint8_t i = 4; i < 8; i++) 
			{
				can->tx_frame_list[tx_frame_dx].data[i] = 0;
			}
            can->tx_frame_list[tx_frame_dx].dlc = 8; // 数据长度8字节 
            break;
        default:
            break;
        }
    }
    
    
    void Vesc::Can_Rx_It_Process(uint8_t *rx_data)
    
    {
        current = ((int16_t)((rx_data[4] << 8) | rx_data[5])*0.1);

        rpm = (int32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);


	}
}
	
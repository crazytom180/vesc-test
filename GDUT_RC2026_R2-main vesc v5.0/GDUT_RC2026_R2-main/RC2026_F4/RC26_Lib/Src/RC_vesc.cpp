#include "RC_vesc.h"

namespace vesc {
    Vesc::Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_), motor::Motor() {
        // ��ʼ��ID��1-255��Ч��
        if (id_ <= 255 && id_ != 0) {
            id = id_;
        } else {
            Error_Handler();
        }
        
        // ע��CAN�豸
        CanHandler_Register();
        
      
       
    }
    
    void Vesc::CanHandler_Register() {
        if (can->hd_num > 8) {  // �豸��������
            Error_Handler();
        }

        can_frame_type = can::FRAME_EXT;  // ��չ֡
        UpdateTxId();  // ��ʼ������ID
        rx_id = (CAN_PACKET_STATUS<< 8) | id;
        // ע��CAN����֡
        if (can->tx_frame_num == 0) {  // ������֡
            tx_frame_dx = 0;
            can->tx_frame_num = 1;
            can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
            can->tx_frame_list[tx_frame_dx].id = tx_id;
            can->tx_frame_list[tx_frame_dx].dlc = 4;
            can->tx_frame_list[tx_frame_dx].hd_num = 1;
            can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
        } else {  // ����֡������֡
            can->tx_frame_num++;
            tx_frame_dx = can->tx_frame_num - 1;
            can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
            can->tx_frame_list[tx_frame_dx].id = tx_id;
            can->tx_frame_list[tx_frame_dx].dlc = 4;
            can->tx_frame_list[tx_frame_dx].hd_num = 1;
            can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
        }
    }

    void Vesc::Set_Rpm(float target_rpm_) {
        vesc_motor_mode = vesc_rpm;
        target_rpm = target_rpm_;
        UpdateTxId();  // ģʽ�л�ʱ���·���ID
        
    }
    //target_c=10A
    void Vesc::Set_Current(float target_c_) {
        vesc_motor_mode = vesc_current;
        target_current = target_c_;
        UpdateTxId();  // ģʽ�л�ʱ���·���ID
      
    }
//λ��ʽ�ǳ�������ʹ��
    void Vesc::Set_Pos(float target_pos_)
    {
        vesc_motor_mode = vesc_pos;
        target_pos = target_pos_;
        UpdateTxId();  // ģʽ�л�ʱ���·���ID
    }
    //duty=0.51(51%)
    void Vesc::Set_Duty(float target_duty_)
    {
        vesc_motor_mode = vesc_duty;
        target_duty = target_duty_;
        UpdateTxId();  // ģʽ�л�ʱ���·���ID
    }

    void Vesc::Tim_It_Process() {
       
    }

    void Vesc::Can_Tx_Process() {
        switch (vesc_motor_mode) {
            case vesc_current: {
                
                send_current = (int32_t)(target_current * 1000);  
                can->tx_frame_list[tx_frame_dx].data[0] = (send_current >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_current >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_current >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_current & 0xFF;
                break;
            }
            case vesc_duty: {
                
                send_duty = (int32_t)(target_duty * 100000);  
                can->tx_frame_list[tx_frame_dx].data[0] = (send_duty >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_duty >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_duty >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_duty & 0xFF;
                break;
            }
            case vesc_rpm: {
                // RPMֱֵ��ת��Ϊ32λ����
                send_rpm = (int32_t)target_rpm;
                // ���CAN���ݣ����ģʽ��
                can->tx_frame_list[tx_frame_dx].data[0] = (send_rpm >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_rpm >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_rpm >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_rpm & 0xFF;
                break;
            }
            case vesc_pos: {
                // POSֱֵ��ת��Ϊ32λ����
                send_pos = (int32_t)(target_pos*1000000);
                // ���CAN���ݣ����ģʽ��
                can->tx_frame_list[tx_frame_dx].data[0] = (send_pos >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_pos >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_pos >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_pos & 0xFF;
                break;
            }
            default:
                break;
        }
       
        can->tx_frame_list[tx_frame_dx].dlc = 4;
    }

    void Vesc::Can_Rx_It_Process(uint8_t *rx_data) {
       
        rpm = (int32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
        current = ((int16_t)((rx_data[4] << 8) | rx_data[5])) * 0.01f;  // ������������Ϊ0.01A/LSB
		duty = ((int16_t)((rx_data[6] << 8) | rx_data[7]))/1000.0f;
    }
	
 // ��������������CAN����ID
	void Vesc::UpdateTxId() {
		
            switch (vesc_motor_mode) {
                case vesc_current:
                    tx_id = (CAN_PACKET_SET_CURRENT << 8) | id;
                    break;
                case vesc_rpm:
                    tx_id = (CAN_PACKET_SET_RPM << 8) | id;
                    break;
				case vesc_pos:
                    tx_id = (CAN_PACKET_SET_POS << 8) | id;
                    break;
                case vesc_duty:
                    tx_id = (CAN_PACKET_SET_DUTY << 8) | id;
                    break;  
                default:
                    break;
            }
            // ����CAN֡��ID
            can->tx_frame_list[tx_frame_dx].id = tx_id;
        }
}
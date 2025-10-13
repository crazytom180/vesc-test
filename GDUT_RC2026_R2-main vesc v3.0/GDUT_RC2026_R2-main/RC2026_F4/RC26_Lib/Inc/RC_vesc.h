#pragma once
#include "RC_motor.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_pid.h"

#ifdef __cplusplus

typedef struct vescCanTxFrame {
    uint32_t id;               // ��չ֡ID��29λ��
    uint32_t dlc;              // ���ݳ��ȣ�VESC����1~4�ֽڣ�
    uint8_t data[8];           // ���ݻ���������һ���ֽ�Ϊָ�����ͣ�
    uint8_t hd_num;            // ���ص�VESC�豸������1��֡��Ӧ1��VESC��
    uint16_t hd_dx[1];         // VESC�豸������ÿ��ֻ֡��1���豸��
    can::CanFrameType frame_type;   // ǿ��Ϊ��չ֡FRAME_EXT
} vescCanTxFrame;

enum vesc_mode
{   vesc_current,
    vesc_erpm
};
	
namespace vesc
{
    
    class Vesc : public motor::Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
        Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_);
        virtual ~Vesc() {}
        void Set_Rpm(float target_rpm_);
        void Set_Current(float target_c_);
        pid::Pid pid_spd, pid_pos;
        
    protected:
        
        void CanHandler_Register() override;
        void Tim_It_Process() override;
        void Can_Tx_Process() override;
        void Can_Rx_It_Process(uint8_t *rx_data) override;
      
       
    
    
    
    private:
        uint8_t id;float gear_ratio = 1;int motor_polse = 7;float erpm = 0;float current = 0;float target_current = 0;int32_t send_current = 0;int32_t send_rpm = 0;
        const uint8_t SET_CURRENT_CMD = 0x01; // VESC���õ���ָ��
        const uint8_t SET_ERPM_CMD = 0x03;    // VESC���õ��ERPMָ��
        const uint8_t GET_VALUES_CMD = 0x09;  // ����״̬����ָ��
        vesc_mode vesc_motor_mode = vesc_current; // VESC����ģʽ��Ĭ�ϵ���ģʽ
    };
}
#endif

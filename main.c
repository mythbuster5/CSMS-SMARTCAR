#include "common.h"
#include "main.h"
void PIT_Interrupt(uint8 ch)
{   
    Control();
    GPIO_Turn(G1);
    GPIO_Turn(G2);
    GPIO_Turn(G3);
}

int main(void)
{
  while((1280*ex_clk_khz) != (256*ics_clk_khz));//ȷ��ʱ����������
  GPIO_Init(G1,GPO,LOW);
  Soft_Delay_ms(1000);
  GPIO_Turn(G1);
  GPIO_Init(G2,GPO,LOW);
  GPIO_Init(G3,GPO,HIGH);

  PIT_Init1(pit0,100000);                 
  PIT_SetCallback(PIT_Interrupt);	
  Disable_Interrupt(INT_PIT_CH0);
  Enable_Interrupt(INT_PIT_CH0); 
  
  /****************************************
  ����PWM���ĳ�ʼ��
  *****************************************/
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0);  //CON3ͨ������
  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0);  //CON3ͨ��ǰ��
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //��ʱ�ò���
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //��ʱ�ò���
  
  while(1)
  {
  }
}

void Control(){
  SetMotor(100);
}

/***********************
 ���÷�����
 ����t��������������
************************/

void Alarm(int t){
  GPIO_Init(I1,GPO,1);
  Soft_Delay_us(t);
  GPIO_Init(I1,GPO,0);
  Soft_Delay_us(t);//6 1136 1 1911
}

/***********************
  ���õ��
  ����  MotorDuty :ռ�ձ�
*************************/

void  SetMotor(int MotorDuty)
{
  //�Դ���ռ�ձȽ����޷���2000Ϊ�����ٶȣ�700Ϊ�����ٶ�
  if(MotorDuty > 3000)
  {
    MotorDuty = 3000 ;
  }
  if(MotorDuty <= 0) MotorDuty = 0;
  FTM_PWM_Duty(ftm2, ftm_ch0, MotorDuty);
 
}

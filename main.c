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
  while((1280*ex_clk_khz) != (256*ics_clk_khz));//确保时钟配置无误
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
  设置PWM波的初始化
  *****************************************/
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0);  //CON3通道后退
  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0);  //CON3通道前进
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  
  while(1)
  {
  }
}

void Control(){
  SetMotor(100);
}

/***********************
 设置蜂鸣器
 参数t：蜂鸣器半周期
************************/

void Alarm(int t){
  GPIO_Init(I1,GPO,1);
  Soft_Delay_us(t);
  GPIO_Init(I1,GPO,0);
  Soft_Delay_us(t);//6 1136 1 1911
}

/***********************
  设置电机
  参数  MotorDuty :占空比
*************************/

void  SetMotor(int MotorDuty)
{
  //对传入占空比进行限幅，2000为上限速度，700为下限速度
  if(MotorDuty > 3000)
  {
    MotorDuty = 3000 ;
  }
  if(MotorDuty <= 0) MotorDuty = 0;
  FTM_PWM_Duty(ftm2, ftm_ch0, MotorDuty);
 
}

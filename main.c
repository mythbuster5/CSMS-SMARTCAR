#include "common.h"
#include "main.h"
#include "OLED_0_96.h"
void PIT_Interrupt(uint8 ch)
{   
    Control();
    int x= Pin(C5);
    int y = Pin(PTC5);
    int z=Pin(PC5);
   // GPIO_Turn(G1);
   // GPIO_Turn(G2);
   // GPIO_Turn(G3);
    OLED_Show_String(40,40,20,20,1,"hahahaha",1);
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
  FTM_PWM_Init(ftm0,ftm_ch0,A0,50,75);  //舵机
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0);  //CON3通道后退
  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0);  //CON3通道前进
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  
  /***************************************
  设置AD的初始化
  ****************************************/
  ADC_Init(ADC0_SE1,ADC_12bit);//A1
  ADC_Init(ADC0_SE2,ADC_12bit);//A6
  ADC_Init(ADC0_SE3,ADC_12bit);//A7
  ADC_Init(ADC0_SE9,ADC_12bit);//C1
  ADC_Init(ADC0_SE10,ADC_12bit);//C2
  
  /***************************************
  拨码开关的初始化
  ****************************************/
  GPIO_Init(C5,GPI,1);
  GPIO_Init(H7,GPI,1);
  GPIO_Init(H5,GPI,1);
  GPIO_Init(H2,GPI,1);
  
  OLED_Init();
  //OLED_Display_On();
  OLED_Display_Config(1);
  
  
  
  while(1)
  {
  }
}

void Control(){
  if(Pin(C5)==1){
    GPIO_Turn(G1);
  }
  else{
  GPIO_Turn(G1);
  GPIO_Turn(G2);
  GPIO_Turn(G3);
  }
  SetMotor(80);
  SetSteer(75);
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
  if(MotorDuty > 500)
  {
    MotorDuty = 500 ;
  }
  if(MotorDuty <= 0) MotorDuty = 0;
  FTM_PWM_Duty(ftm2, ftm_ch0, MotorDuty);
 
}

/***********************
  设置舵机
  参数  SteerDuty :占空比
*************************/
void  SetSteer(int SteerDuty)
{
  //对传入占空比进行限幅
  
  if(SteerDuty > MAX_STEER_DUTY)
  {
    SteerDuty = MAX_STEER_DUTY;
  }
  
  if(SteerDuty < MIN_STEER_DUTY)
  {
    SteerDuty = MIN_STEER_DUTY;
  }
  FTM_PWM_Duty(ftm0,ftm_ch0,SteerDuty);
}


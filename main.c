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
  FTM_PWM_Init(ftm0,ftm_ch0,A0,50,75);  //���
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0);  //CON3ͨ������
  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0);  //CON3ͨ��ǰ��
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //��ʱ�ò���
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //��ʱ�ò���
  
  /***************************************
  ����AD�ĳ�ʼ��
  ****************************************/
  ADC_Init(ADC0_SE1,ADC_12bit);//A1
  ADC_Init(ADC0_SE2,ADC_12bit);//A6
  ADC_Init(ADC0_SE3,ADC_12bit);//A7
  ADC_Init(ADC0_SE9,ADC_12bit);//C1
  ADC_Init(ADC0_SE10,ADC_12bit);//C2
  
  /***************************************
  ���뿪�صĳ�ʼ��
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
  if(MotorDuty > 500)
  {
    MotorDuty = 500 ;
  }
  if(MotorDuty <= 0) MotorDuty = 0;
  FTM_PWM_Duty(ftm2, ftm_ch0, MotorDuty);
 
}

/***********************
  ���ö��
  ����  SteerDuty :ռ�ձ�
*************************/
void  SetSteer(int SteerDuty)
{
  //�Դ���ռ�ձȽ����޷�
  
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


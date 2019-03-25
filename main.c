#include "common.h"
#include "main.h"
#include "OLED_0_96.h"
#include <math.h>

void PIT_Interrupt(uint8 c)
{
    speed=FTM_Pulse_Get(ftm1);
    FTM_Count_Clean(ftm1);
    choose_speed();
    count++;
    if(count>2000)
    {
      count=2000;
    }
    if(Pin(I0))
    {
    stop_count++;
    }
    if((stop_count>5)&&(count==2000))
    {
     // GPIO_Init(I1,GPO,1);
      stop_flag=1;
    }
    
    
   /* if(Pin(I0)&&(count==2000))
    {
     // GPIO_Init(I1,GPO,1);
      stop_flag=1;
    }*/
    else
    {
      GPIO_Init(I1,GPO,0);
    }
    Control();
}
/*

  --是否进环
  ----是否丢线
  ------弯道与直道
*/

int main(void)
{
  
/***********************1.中断初始化******************************************/
  PIT_Init_ms(pit0,5); //5ms读取一次ADC             
  PIT_SetCallback(PIT_Interrupt);	
  Disable_Interrupt(INT_PIT_CH0);
  Enable_Interrupt(INT_PIT_CH0); 
  

/***********************2.PWM初始化******************************************/
  //舵机初始化
  FTM_PWM_Init(ftm0,ftm_ch0,A0,50,75); 
  //电机初始化
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0);  //CON3通道后退
  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0);  //CON3通道前进
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0);  //暂时用不上
  
/***********************3.编码器初始化***************************************/
  FTM_Pulse_Init(ftm1,FTM_PS_1,TCLK1);
  GPIO_Init(E1,GPI,HIGH);
  
/***********************4.AD初始化******************************************/
  ADC_Init(ADC0_SE1,ADC_12bit);//A1
  ADC_Init(ADC0_SE2,ADC_12bit);//A6
  ADC_Init(ADC0_SE3,ADC_12bit);//A7
  ADC_Init(ADC0_SE9,ADC_12bit);//C1
  ADC_Init(ADC0_SE10,ADC_12bit);//C2
  
/***********************5.拨码开关初始化*************************************/
  GPIO_Init(C5,GPI,1);
  GPIO_Init(H7,GPI,1);
  GPIO_Init(H5,GPI,1);
  GPIO_Init(H2,GPI,1);
  
 /***********************6.OLED初始化******************************************/
  OLED_Init();
  //OLED_Display_On();
  OLED_Display_Config(1);
 /***********************7.蓝牙初始化******************************************/
  UART_Init(uart2,115200,RXTX_B0B1);
   GPIO_Init(I0,GPI,1);
  while(1)
  {
/********************************************************************************
   Wave[0]=75+Angle;
   Wave[1]=AD[0];
   Wave[2]=AD[1];
   Wave[3]=AD[2];
   Wave[4]=AD[3];
   Wave[5]=AD[4];
   Wave[6]=measured_speed;
   Send_Wave(Wave,7);
********************************************************************临时注释*/
     //OLED_Show_Duty();
    
/***********************速度PID测试代码***************************************
    OLED_Speed();
    Wave[0]=100+100*sin(2*3.1415/200*count);
    Wave[0]=200;
    Wave[2]=speed;
    Send_Wave(Wave,7);
*******************************************************************************/
   //OLED_Speed();

   // OLED_Speed();
  
    OLED_Show_AD();
    
    
   //OLED_Show_Duty();
     // OLED_Show_flag();
    if (DIR!=0) 
    {
      Soft_Delay_ms(3500);
      DIR=0;
    }
    /*
    if(r+s==2||l+s==2)// r,s,l 
    {
      //Soft_Delay_ms(3500);
      r=0;l=0;s=0;
    }
    */
  }
}
void Control(){
 // offset =(int)1000.0*(AD1 - AD2)*1.0/(AD1+ AD2+1);
  
   
   AD1[0]=ADC_Read(ADC0_SE1);//A1 AD1
   AD2[0]=ADC_Read(ADC0_SE3);//A7 AD2
   AD3[0]=ADC_Read(ADC0_SE2);//A6 AD3  左一路
   AD4[0]=ADC_Read(ADC0_SE9);//C1 AD4 右一路
   AD5[0]=ADC_Read(ADC0_SE10);//C2 AD5
   AD_fresh();

  /*
   AD[0]=ADC_Read(ADC0_SE1);//A1 AD1    R2
   AD[1]=ADC_Read(ADC0_SE3);//A7 AD2   L2
   AD[2]=ADC_Read(ADC0_SE2);//A6 AD3   L1
   AD[3]=ADC_Read(ADC0_SE9);//C1 AD4   R1
   AD[4]=ADC_Read(ADC0_SE10);//C2 AD5  M
   */
   

  /*
  IF条件句，将环岛与其它部分分开处理，IF里是环岛
  ELSE里是正常跑道范围
  */

   if(speed<5)
   {
    AD0_level=AD[0];
    AD1_level=AD[1];
    AD2_level=AD[2];
    AD3_level=AD[3];
    AD4_level=AD[4];
   }
    last_last_offset1=last_offset1;
    last_offset1=offset1;
    offset1 = 100*(AD[0]-AD[1])/(AD[0]+AD[1]);
    d_offset1=offset1-last_offset1;
    Sum_offset1=offset1+last_offset1+last_last_offset1;
   
    last_last_offset=last_offset;
    last_offset=offset;
    offset = 100*(AD[3]-AD[2])/(AD[2]+AD[3]);
    d_offset=offset-last_offset;
    Sum_offset=offset+last_offset+last_last_offset;
    
   level=0.4*(AD2_level+AD3_level);
   circle_flag=1000*(AD[2]-AD[3])/(AD[2]+AD[3]+1);
   if((DIR==0))
   {
     if((circle_flag<-90)&&(AD[2]>1.4*AD2_level)&&(AD[3]>1.4*AD3_level))//右侧环岛
    {
      test1=0;       //    对环岛的检测可能不稳定
      DIR=RIGHT;
    //  Motor=60;
    }
    if((circle_flag>90)&&(AD[3]>1.4*AD3_level)&&(AD[2]>1.4*AD2_level))//左侧环岛
    {
    
      test1=0;
      DIR=LEFT;
     // Motor=60;
    }

    /*
    if(l+s==2||r+s==2)
    {
      DIR=0;
    }
    */
   }
  /*if((AD[4]>1.4*AD4_level)&&((AD[4]<1.75*AD4_level)))
    //环岛中点设置标志位，直道后清除标志位  1.75改为1.3   降低阈值，提前检测环岛，用于提高速度
  {
    /*
    if(l+s==2||r+s==2)
    {
      r_flag=0;
    }
    else
    {
      r_flag=1;

    }
    */
  /*  if(DIR!=0)
    {
      Motor=speed_selection[index][4];
       GPIO_Init(I1,GPO,1);//进环蜂鸣器
    }
  }*/
    if(AD[4]>1.85*AD4_level)
  {
    r_flag=1;
    test1=1;
  }
  /********************************** YQZ对算法修正后*********************/
  if(((AD[0]>1.4*AD0_level)||(AD[1]>1.4*AD1_level))&&((AD[2]>1.5*AD2_level)||(AD[3]>1.5*AD3_level))&&(r_flag>0))
    //此IF用于判断是否是环岛,进入环岛后不得再次标记标志位
    //环岛的判断条件加强：R1(L1)、R2(L2)、M满足条件才判断为环岛
    //十字弯可能只满足上面的两个条件
  {
    //r_flag=1;
    
    if((DIR==RIGHT)) //L1<R1,即右侧环岛
    {
      //GPIO_Init(I1,GPO,1);
     
        r=1; 
        //Angle=5;
        //offset = 100*(AD[3]-AD[2])/(AD[2]+AD[3]);
        Angle = 5*offset1+1.5*d_offset1+0.0*Sum_offset1;
        if(AD[1]<0.6*AD1_level)
        {
          r_flag=0;
          DIR=0;
        //  test1=0;
        }
        if(Angle<0)
        {
          Angle=0;
        }
        Motor =speed_selection[index][0];//进环减速
    }
    else if((DIR==LEFT))//L1>R1，即左侧环岛
    {
      //GPIO_Init(I1,GPO,1);
        l=1; 
        //Angle=-5;
        //offset = 100*(AD[3]-AD[2])/(AD[2]+AD[3]);
        Angle = 1.4*offset1+1.5*d_offset1+0.0*Sum_offset1;
        if(AD[0]<0.6*AD0_level)
        {
          r_flag=0;
          DIR=0;
        }
        if(Angle>0)
        {
          Angle=0;
        }
     }
    Motor =speed_selection[index][0];//进环减速
    GPIO_Init(I1,GPO,1);
  }
  else                 //如果不是环岛，换入正常状况（即弯道或直道或丢线）
  {
    test1=0;
   // flag=0;
    //GPIO_Init(I1,GPO,0);
    /************************丢线处理*********************************************/
   if(AD[2]+AD[3]<level&&AD[2]>AD[3])//丢线状况1
    {
      Angle=-20;
      Motor=speed_selection[index][1];
      r_flag = 0;
      DIR=0;
      if((r==1)||(l==1))
        {
          s=1;
        }
      
    }
    else if(AD[2]+AD[3]<level&&AD[2]<AD[3])//丢线状况2
    {
      Angle=20;
      Motor=speed_selection[index][1];
      r_flag = 0;
      DIR=0;
      if((r==1)||(l==1))
        {
          s=1;
        }
    }
  
    else//如果不丢线
    {
      
      if(abs(AD[2]-AD[3])>0.25*(AD2_level+AD3_level))//弯道状态
      {   
       // Angle = 0.175*offset+0.00*Sum_offset+1.95*d_offset;           重要  0档速度最快 （0，1最稳定）
        //Angle = 0.2*offset+0.00*Sum_offset+2.0*d_offset;
       Angle=P[index]*offset+D[index]*d_offset;
        Motor=speed_selection[index][2];
        if((r==1)||(l==1))
        {
          s=1;
        }
       GPIO_Init(I1,GPO,0);
      //Alarm(1136);
      }
      else                                  //直道状态
      {
         if((r==1)||(l==1))
        {
          s=1;
        }
        //Angle=0.018*offset+0.17*d_offset;
        //Angle = 0.12*offset+0.09*d_offset;//第一版PID，还不错
        Angle = 0.080*offset +1.4*d_offset;
        
        
        if(Angle>10)
        {
          Angle=4;
        }
        else if(Angle<-10)
        {
          Angle=-4;
        }

      //else{}
    
      
        Motor=speed_selection[index][3];//直道速度
     }

    }
  
  }
  if((AD[0]<50)&&(AD[1]<50)&&(AD[2]<50)&&(AD[3]<50)&&(AD[4]<50))
  {
      Angle=0;
      Motor=3;
  }
 if(stop_flag==1)
  {
    Motor=speed_selection[index][4];
  }
  SetSteer(75+Angle);
  SetMotor(speed_control(Motor));

   
/***********************速度PID测试代码****************************************
   count++;
   if(count<400)
   {
     Motor=70;
   }
   if((400<count)&&(count<800))
   {
   Motor=10;
   }
   if(count==800)
   {
   count=0;
   }
   
      SetMotor(speed_control(Motor));
  // SetMotor(100);
  // SetMotor(speed_control(200));
*****************************************************************************/
}

/******************************************************************************
 设置蜂鸣器
 参数t：蜂鸣器半周期
*******************************************************************************/

void Alarm(int t)
{
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
/*******对传入占空比进行限幅，500为上限速度，0为下限速度***********************/
  if(MotorDuty > 500)
  {
    MotorDuty = 500 ;
  }
  if(MotorDuty <= 0) MotorDuty = 0;
  FTM_PWM_Duty(ftm2, ftm_ch0, MotorDuty);
 
}

/******************************************************************************
  设置舵机
  参数  SteerDuty :占空比
  notes：75为中间值，左右值不可超过50、100
  代码已保护
*******************************************************************************/
void  SetSteer(int SteerDuty)
{
/****************************对传入占空比进行限幅******************************/
  
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

/************注意：此函数无用了，速度在中断函数中获得***************************
  设置编码器
  无需传值
  return：空转返回motorduty10倍偏多
  warning:不可使用speed1及fpm0，否则舵机无法正常工作
*******************************************************************************/
/*uint16_t GetSpeed()
{
  pulse = 0;
  pulse = FTM_Pulse_Get(ftm1);
 FTM_Count_Clean( ftm1);
  return pulse;
}*/

void OLED_Show_AD()
{
  char string0[10]={0,0,0,0,0,0,0,0,0,0};
  char string1[10]={0,0,0,0,0,0,0,0,0,0};
  char string2[10]={0,0,0,0,0,0,0,0,0,0};
  char string3[10]={0,0,0,0,0,0,0,0,0,0};
  char string4[10]={0,0,0,0,0,0,0,0,0,0};
  char string5[10]={0,0,0,0,0,0,0,0,0,0};
  sprintf(string0,"%s%d","R2:",AD[0]);
  sprintf(string1,"%s%d","L2:",AD[1]);
  sprintf(string2,"%s%d","L1:",AD[2]);
  sprintf(string3,"%s%d","R1:",AD[3]);
  sprintf(string4,"%s%d","M:",AD[4]);
  sprintf(string5,"%s%d","I0:",Pin(I0));
  OLED_Show_String(15,15,0,15,1,string0,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,0,30,1,string1,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,0,45,1,string2,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,15,1,string3,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,30,1,string4,0); 
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,45,1,string5,0); 
  OLED_Refresh_Gram();
}
void OLED_Show_Duty()
{

 // char string0[10]={0,0,0,0,0,0,0,0,0,0};
  char string1[10]={0,0,0,0,0,0,0,0,0,0};
  //sprintf(string0,"%s%s","M:","75");
  sprintf(string1,"%s%4f","D:",speed_control(Motor));
  OLED_Show_String(15,15,60,45,1,string1,0);
  OLED_Refresh_Gram();
}

void Send_Wave(uint8*waveaddr,uint32 wavesize)
{
  #define CMD_WARE 3
  uint8_t cmdf[2] = { CMD_WARE, ~CMD_WARE };      /* 帧头 */
  uint8_t cmdr[2] = { ~CMD_WARE, CMD_WARE };      /* 帧尾*/
  UART_Putbuff(uart2, cmdf, sizeof(cmdf) );      /* 发送帧头 */
  UART_Putbuff(uart2, waveaddr, wavesize );      /* 发送数据 */
  UART_Putbuff(uart2, cmdr, sizeof(cmdr) );     /* 发送帧尾 */
}

/***********************************
*  Instruction:电感值滑动平均滤波
************************************/

void AD_fresh()
{
  for(;AD_count>0;AD_count--)
  {
  AD1[count]=AD1[AD_count-1];
  AD2[count]=AD2[AD_count-1];
  AD3[count]=AD3[AD_count-1];
  AD4[count]=AD4[AD_count-1];
  AD5[count]=AD5[AD_count-1];
  }
  if(AD_count==1)
  {
    AD_count=5;
  }

  if(AD1[0]*AD1[1]*AD1[2]*AD1[3]*AD1[4]==0)
  {
   AD[0]=AD1[0];
   AD[1]=AD2[0];
   AD[2]=AD3[0];
   AD[3]=AD4[0];
   AD[4]=AD5[0];
  }
  else
  {
   AD[0]=0.2*(AD1[0]+AD1[1]+AD1[2]+AD1[3]+AD1[4]);
   AD[1]=0.2*(AD2[0]+AD2[1]+AD2[2]+AD2[3]+AD2[4]);
   AD[2]=0.2*(AD3[0]+AD3[1]+AD3[2]+AD3[3]+AD3[4]);
   AD[3]=0.2*(AD4[0]+AD4[1]+AD4[2]+AD4[3]+AD4[4]);
   AD[4]=0.2*(AD5[0]+AD5[1]+AD5[2]+AD5[3]+AD5[4]);
  }
}
 
double speed_control(double expected_speed)
{
    float P=15;
    float I=3.0;
    float D=0.5;
    float Duty;
    if(speed>120)
    {
    expected_speed=110;              //暂时人为限速
    }
    last_last_speed_error=last_speed_error;
    last_speed_error=speed_error;
    speed_error=expected_speed-speed;
    d_error=speed_error-last_speed_error;
    Duty=expected_speed+P*speed_error+D*d_error+I*(speed_error+last_speed_error+last_last_speed_error);
    return Duty;
}

/***********************************************
 * instruction:速度显示
 * ********************************************/

void OLED_Speed()
{
  char string0[12]={0,0,0,0,0,0,0,0,0,0,0,0};
  sprintf(string0,"%s%d","speed:",speed);
  OLED_Show_String(15,15,30,30,1,string0,0);
  OLED_Refresh_Gram();
}

void OLED_Show_flag()
{
  char string0[10]={0,0,0,0,0,0,0,0,0,0};
  char string1[10]={0,0,0,0,0,0,0,0,0,0};
  char string2[10]={0,0,0,0,0,0,0,0,0,0};
  char string3[10]={0,0,0,0,0,0,0,0,0,0};
  char string4[10]={0,0,0,0,0,0,0,0,0,0};
  sprintf(string0,"%s%d","R2:",AD[0]);
  sprintf(string1,"%s%d","L2:",AD[1]);
  sprintf(string2,"%s%d","sum:",sum);
  sprintf(string3,"%s%d","flag:",flag);
  sprintf(string4,"%s%d","M:",AD[4]);
  sprintf(string5,"%s%d","DIR:",DIR);
  OLED_Show_String(15,15,0,15,1,string0,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,0,30,1,string1,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,0,45,1,string2,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,15,1,string3,0);
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,30,1,string4,0); 
  OLED_Refresh_Gram();
  OLED_Show_String(15,15,60,45,1,string5,0); 
  OLED_Refresh_Gram();
}

/**********************************************
 *  instrction:用于速度选择 
 *  operation：拨码开关
 * ********************************************/
void choose_speed()
{
  //拨码开关示例
  if(Pin(C5)&&(!Pin(H7))&&(!Pin(H5))&&(!Pin(H2)))
  { 
    index=1;
  }
else if((!Pin(C5))&&(Pin(H7))&&(!Pin(H5))&&(!Pin(H2)))
  { 
    index=2;
  }
 else if((!Pin(C5))&&(!Pin(H7))&&(Pin(H5))&&(!Pin(H2)))
  { 
    index=3;
  }
 else if((!Pin(C5))&&(!Pin(H7))&&(!Pin(H5))&&(Pin(H2)))
  { 
    index=4;
  }
  else
  {
    index=0;
  }
}

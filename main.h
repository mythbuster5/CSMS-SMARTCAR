#define MAX_STEER_DUTY 100
#define MIN_STEER_DUTY 50
#define RIGHT 1
#define LEFT 2
uint16 AD[5]={0};
uint8 Wave[7];
uint16_t GetSpeed(void);
uint16 AD[5];
uint16 AD_count=5;
uint16 AD1[5];
uint16 AD2[5];
uint16 AD3[5];
uint16 AD4[5];
uint16 AD5[5];
uint16 AD0_level;
uint16 AD1_level;
uint16 AD2_level;
uint16 AD3_level;
uint16 AD4_level;
void PIT0_ISR();
void Control();
void SetMotor(int MotorDuty);//���õ���ٶ�
void  SetSteer(int SteerDuty);//���ö��ƫ���
void OLED_Show_AD();
void Send_Wave(uint8*waveaddr,uint32 wavesize);
void OLED_Show_Duty();
void Alarm(int t);
void choose_speed();
void AD_fresh();
void OLED_Speed();
void OLED_Show_flag();
void keyCheck();                
double speed_control(double expected_speed);
int level=0;
int speed=0;
int Motor = 0;
int pulse;
int r_flag;//������־λ
int i=0;
int DIR;
int test1=0;
int r,l,s;//rΪ�ҽ�����l�������sΪ����״̬
int speed_error;
int last_speed_error;
int last_last_speed_error;
int d_error;
int sum;
int index=0;//速度矩阵的索引
int flag=0;
int count=0;
int circle_flag;
int stop_flag;
double offset=0;
double offset1=0;
double last_offset=0;
double last_offset1=0;
double last_last_offset=0;
double last_last_offset1=0;
double d_offset=0;
double d_offset1=0;
int stop_count=0;
float Angle=0;

double Sum_offset=0;
double Sum_offset1=0;

/*******************************************************************************
*************************关于速度的注释*****************************************
***************************1.进环速度*******************************************
***************************2.丢线速度*******************************************
***************************3.弯道速度*******************************************
***************************4.直道速度*******************************************
***************************5.进环减速*******************************************
*******************************************************************************/
int speed_selection[5][5]={50,10,58,75,0,
                           50,10,60,77,0,    //speed[1]
                           50,10,55,73,0,    //speed[2]
                           50,10,50,75,0,    //speed[3]
                           50,10,58,95,0};  //speed[4

double P[5]={0.175,0.175,0.18,0.175,0.2};
double D[5]={1.95,1.95,1.95,1.95,2.0};

/*******************************************************************************
 吴院                      30,30,40,65,50,   
 验证                      20,30,50,75,50,    
 过的                      55,30,55,85,50,    
 速度                      45,30,60,95,55};  
*******************************************************************************/
char string5[10];

void PIT0_ISR();
void Control();
void SetMotor(int MotorDuty);//设置电机速度
void  SetSteer(int SteerDuty);//设置舵机偏向角
#define MAX_STEER_DUTY 100
#define MIN_STEER_DUTY 50
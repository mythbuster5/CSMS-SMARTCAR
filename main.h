void PIT0_ISR();
void Control();
void SetMotor(int MotorDuty);//���õ���ٶ�
void  SetSteer(int SteerDuty);//���ö��ƫ���
#define MAX_STEER_DUTY 100
#define MIN_STEER_DUTY 50
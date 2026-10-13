#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

typedef struct
{
    float q[4]; // ��Ԫ������ֵ
    float gyro[3];  // ���ٶ�
    float accel[3]; // ���ٶ�
	    float temp;			// �¶�
    // λ��
    float roll;
    float pitch;
    float yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;

void INS_Init(void);
void IMU_task(void * argument);

#endif // IMU_TEMP_CTRL_H

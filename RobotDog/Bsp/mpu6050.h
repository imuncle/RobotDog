#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_ADDRESS 0xD0
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_SAMPLE_RATE_REG		0X19	//陀螺仪采样频率分频器
#define MPU_CFG_REG 0x1A
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_INT_PIN_CFG 0X37

#define MPU_WHO_I_AM	0x75	//陀螺仪ID

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值，X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值，X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值，Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值，Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值，Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值，Z轴低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值，X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值，X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值，Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值，Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值，Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值，Z轴低8位寄存器

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/*计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */

struct Axisf
{
	float x;
	float y;
	float z;
};

struct Axisi
{
	int x;
	int y;
	int z;
};

struct MPU6500_t
{
	struct Axisf gyro;
	struct Axisf attitude;
	struct
	{
		int yaw;
		int last_yaw;
		int encoder_yaw;
		int yaw_count;
		int pitch;
		int roll;
	} angle;
};

typedef struct
{
	struct Axisf     bias;
	int       isBiasValueFound;
	int       isBufferFilled;
	struct Axisi   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

extern BiasObj	gyroBiasRunning;
extern struct MPU6500_t mpu6500;

int MPU6050_Init(void);
int MPU6050_ReadData(void);
void imuDataHandle(void);
void imuUpdate(struct Axisf gyro);

#endif

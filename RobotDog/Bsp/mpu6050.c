#include "mpu6050.h"
#include "soft_i2c.h"
#include "math.h"

#define DEG2RAD		0.017453293f	/*度转弧度*/
#define RAD2DEG		57.29578f		/*弧度转度*/

struct MPU6500_t mpu6500;

BiasObj	gyroBiasRunning;
int gyroBiasFound = 0;

float Kp = 1.0f;		/*比例增益*/
float Ki = 0.5f;		/*积分增益*/

float q0 = 1.0f;	/*四元数*/
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

uint8_t imu_data[14]={0};

struct Axisf gyroBias;

volatile uint32_t last_update, now_update;

int MPU6050_Init(void)
{
	uint8_t temp_data = 0;
	if(IIC_ReadData(MPU6050_ADDRESS, MPU_WHO_I_AM, &temp_data, 1) == 0)
	{
		if(temp_data != 0x68)
		{
			return 0xff;
		}
	}
	else
	{
		return 0xff;
	}
	
	if(IIC_WriteData(MPU6050_ADDRESS,MPU_PWR_MGMT1_REG,0x01) == 0xff)    //½â³ýÐÝÃß×´Ì¬
	{
		return 0xff;
	}
	if(IIC_WriteData(MPU6050_ADDRESS,MPU_SAMPLE_RATE_REG,0x07) == 0xff)//cyq£º07 ¸üÐÂÆµÂÊ1khz
	{
		return 0xff;
	}
	if(IIC_WriteData(MPU6050_ADDRESS,MPU_CFG_REG,6) == 0xff)
	{
		return 0xff;
	}
	if(IIC_WriteData(MPU6050_ADDRESS,MPU_GYRO_CFG_REG,0x18) == 0xff) //Á¿³Ì500¡ã/s
	{
		return 0xff;
	}   
	return 0;
}

int MPU6050_ReadData(void)
{
	if(IIC_ReadData(MPU6050_ADDRESS, MPU_ACCEL_XOUTH_REG, imu_data, 14) == 0xff)
	{
		return 0xff;
	}
	else
	{
		return 0;
	}
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, struct Axisf* varOut, struct Axisf* meanOut)
{
	uint32_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/*传感器查找偏置值*/
static int sensorsFindBiasValue(BiasObj* bias)
{
	int foundbias = 0;

	if (bias->isBufferFilled)
	{
		
		struct Axisf mean;
		struct Axisf variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = 1;
			bias->isBiasValueFound= 1;
		}else
			bias->isBufferFilled=0;
	}
	return foundbias;
}

/**
 * 计算陀螺方差
 */
int processGyroBias(int16_t gx, int16_t gy, int16_t gz, struct Axisf *gyroBiasOut)
{
	static int count = 0;
	gyroBiasRunning.buffer[count].x = gx;
	gyroBiasRunning.buffer[count].y = gy;
	gyroBiasRunning.buffer[count].z = gz;
	count++;
	if(count == 1024)
	{
		count = 0;
		gyroBiasRunning.isBufferFilled = 1;
	}

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

void imuDataHandle(void)
{
	short gyrox,gyroy,gyroz;
	float fgyrox, fgyroy, fgyroz;
	float gyro_sensitivity = 16.384f;

	gyrox = (imu_data[8]<<8)|imu_data[9];
	gyroy = (imu_data[10]<<8)|imu_data[11];
	gyroz = (imu_data[12]<<8)|imu_data[13];
	
	gyroBiasFound = processGyroBias(gyrox, gyroy, gyroz, &gyroBias);
	
	fgyrox = -(float)(gyrox-gyroBias.x)/gyro_sensitivity;
	fgyroy = (float)(gyroy-gyroBias.y)/gyro_sensitivity;
	fgyroz = (float)(gyroz-gyroBias.z)/gyro_sensitivity;
	
	mpu6500.gyro.x = 0.8f*fgyrox + 0.2f*mpu6500.gyro.x;
	mpu6500.gyro.y = 0.8f*fgyroy + 0.2f*mpu6500.gyro.y;
	mpu6500.gyro.z = 0.8f*fgyroz + 0.2f*mpu6500.gyro.z;
	
	if(mpu6500.gyro.x < 0.1f && mpu6500.gyro.x > -0.1f) mpu6500.gyro.x = 0;
	if(mpu6500.gyro.y < 0.1f && mpu6500.gyro.y > -0.1f) mpu6500.gyro.y = 0;
	if(mpu6500.gyro.z < 0.1f && mpu6500.gyro.z > -0.1f) mpu6500.gyro.z = 0;
}

void imuUpdate(struct Axisf gyro)
{
	float normalise;
	float halfT;
	
	now_update = HAL_GetTick(); //单位ms
	halfT = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	gyro.x *= DEG2RAD;	/*度转弧度*/
	gyro.y *= DEG2RAD;
	gyro.z *= DEG2RAD;
	
	/* 使用一阶龙格库塔更新四元数 */
	q0 += (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z) * halfT;
	q1 += ( q0 * gyro.x + q2 * gyro.z - q3 * gyro.y) * halfT;
	q2 += ( q0 * gyro.y - q1 * gyro.z + q3 * gyro.x) * halfT;
	q3 += ( q0 * gyro.z + q1 * gyro.y - q2 * gyro.x) * halfT;
	
	/* 对四元数进行归一化处理 */
	normalise = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= normalise;
	q1 /= normalise;
	q2 /= normalise;
	q3 /= normalise;
	
	/* 由四元数求解欧拉角 */
	mpu6500.attitude.z = atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * RAD2DEG;	//yaw
	mpu6500.attitude.y = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * RAD2DEG;	//roll
	mpu6500.attitude.x = -asinf(-2*q1*q3 + 2*q0*q2) * RAD2DEG;	//pitch
	
	mpu6500.angle.yaw = mpu6500.attitude.z;
	mpu6500.angle.pitch = mpu6500.attitude.x;
	mpu6500.angle.roll = mpu6500.attitude.y;
}

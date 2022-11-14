#include <math.h>
#include "mpu6050.h"

#define RAD2DEG 57.295779513082320876798154814105  // 1 radian = 57.29.. degree is defined as a constant
#define WHOAMI 0x75                                // MPU6050 whoami register adress is defined as a constant
#define PWR_MGMT 0x6B                              // power management register defined as a constant  
#define SMPLRT_DIV 0x19         
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define MPU6050_ADDR 0xD0
const uint16_t I2C_TIMEOUT = 100;
const double Acc_Z_corrector = 14418.0;
uint32_t timer;


//Kalman filter KalmanX parameter is defined
Filter_t FilterX = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f};

//Kalman filter KalmanY parameter is defined
Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};

uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;


    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT);    //checking device id with  WHO_AM_I constant

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        
        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}

void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
}

void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG;
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);
}

double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)
{
    double rate = newRate - Filter->bias;
    Filter->angle += dt * rate;

    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;

    double S = Filter->P[0][0] + Filter->R_MEASURE;
    double K[2];
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;

    double y = newAngle - Filter->angle;
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;

    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle;
};
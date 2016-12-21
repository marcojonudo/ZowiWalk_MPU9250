//--------------------------------------------------------------
//-- Get data from accelerometer, gyroscope and magnetometer.
//-- Calculation of the yaw angle using quaternions (Madgwick's Filter)
//--------------------------------------------------------------
//-- Marco Martínez Ávila
//-- Based on Kris Winer's code from GitHub
//--------------------------------------------------------------

#ifndef MPU9250_h
#define MPU9250_h

#include <stdint.h>

#define MPU9250_ADDRESS     0x68
#define WHO_AM_I_MPU9250    0x75 // Should return 0x71
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define USER_CTRL           0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNTH         0x72
#define FIFO_R_W            0x74

#define AK8963_ADDRESS      0x0C
#define AK8963_WHO_AM_I     0x00 // should return 0x48
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_CNTL         0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX         0x10  // Fuse ROM x-axis sensitivity adjustment value

#define aRes 0.000061037015914f
#define gRes 0.007629627227783f
#define mRes 1.464888477325439f

#define magBias0 180.35f
#define magBias1 -104.66f
#define magBias2 71.38f
#define magScale0 1.04f
#define magScale1 1.01f
#define magScale2 0.95f

class MPU9250
{
  public:
    void getAccelData(float * ax, float * ay, float * az);
    void getGyroData(float * gx, float * gy, float * gz);
    void getMagData(float * mx, float * my, float * mz);
    void initMPU();
    bool dataReady();

  private:
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    void readAccelData(int16_t * destination);
    void readGyroData(int16_t * destination);
    void readMagData(int16_t * destination);
    void calibrateAccelGyro(float * dest1, float * dest2);
    void initMPU9250();
    void initAK8963(float * destination);

  private:
    float magCalibration[3] = {0, 0, 0};
    float accelBias[3] = {0, 0, 0};
    float gyroBias[3] = {0, 0, 0};
};

#endif
